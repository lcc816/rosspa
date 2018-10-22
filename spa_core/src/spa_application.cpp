#include <spa_core/spa_application.h>
#include <functional>
#include <fstream>

namespace spa
{

SpaApplication::SpaApplication(uint64_t id, ComponentType type, const std::string &uri) :
  cuuid(id),
  componentType(type),
  xtedsUri(uri),
  operatingMode(SPA_OPMODE_INITIALIZING),
  probeServer(nh, nodeName + "/spa_probe", boost::bind(&SpaApplication::probeCallback, this, _1), false)
{
}

void SpaApplication::init()
{
  nodeName = ros::this_node::getName(); // set node name
  setXuuid();

  discoveryClient = nh.serviceClient<spa_core::Hello>("spa_sm_l/hello");
  beatServer = nh.advertiseService(nodeName + "/heartbeat", &SpaApplication::beatCallback, this);
  // probeServer = ProbeActionServer(nh, nodeName + "/spa_probe", boost::bind(&SpaApplication::probeCallback, this, _1), false);
  probeServer.start();
  xtedsServer = nh.advertiseService(nodeName + "/xteds", &SpaApplication::xtedsRegisterCallback, this);

  // create a thread to start spinning in the background
  spin_thread = std::thread(boost::bind(&SpaApplication::spinThread, this));

  spa_core::Hello hello;
  hello.request.nodeName = nodeName;
  hello.request.cuuid = cuuid;
  hello.request.componentType = componentType;

  ros::Rate rate(1);
  while (ros::ok())   // Waiting to be discovered by the SM-L
  {
    if (discoveryClient.call(hello))
    {
      ROS_INFO("%s: discovered!", nodeName.c_str());
      break;
    }
    else {
      ROS_ERROR("%s: failed to be discovered, retry!", nodeName.c_str());
      rate.sleep();
    }
  }

  // All is fine, call user's initial function.
  appInit();
  operatingMode = SPA_OPMODE_FULLY_OPERATIONAL;
}

void SpaApplication::setXuuid()
{
  xuuid = 0xFFFFFFFF;
}

uint32_t SpaApplication::getUptime()
{
  return 0;
}

uint64_t SpaApplication::getXuuid()
{
  return xuuid;
}

void SpaApplication::registerRequest()
{

}

void SpaApplication::registerCommand()
{

}


void SpaApplication::registerNotification()
{

}

void SpaApplication::issueQuery()
{
}

bool SpaApplication::xtedsRegisterCallback(spa_core::SpaXteds::Request &req, spa_core::SpaXteds::Response &res)
{
  std::ifstream fin(xtedsUri.c_str());
  if (!fin.is_open())
  {
    ROS_ERROR("%s: unable to read xTEDS", nodeName.c_str());
    return false;
  }

  fin >> res.xteds;
  fin.close();

  return true;
}

void SpaApplication::probeCallback(const spa_core::SpaProbeGoalConstPtr &goal)
{
  ros::Rate rate(1 / (goal->replyPeriod));
  spa_core::SpaProbeFeedback feedback;
  spa_core::SpaProbeResult result;

  feedback.dialogId = goal->dialogId;
  feedback.cuuid = cuuid;
  feedback.xuuid = xuuid;

  int16_t count = goal->replyCount;
  if (count)
  {
    while (ros::ok() && count--)
    {
      feedback.uptime = getUptime();
      feedback.faultIndicator = 0;
      feedback.operatingMode = operatingMode;
      probeServer.publishFeedback(feedback);
      rate.sleep();
    }
  }
  else
  {
    while (ros::ok())
    {
      feedback.uptime = getUptime();
      feedback.faultIndicator = 0;
      feedback.operatingMode = operatingMode;
      probeServer.publishFeedback(feedback);
      rate.sleep();
    }
  }

  result.resultMode = operatingMode;
  probeServer.setSucceeded(result);
}

bool SpaApplication::beatCallback(spa_core::SpaProbe::Request &req, spa_core::SpaProbe::Response &res)
{
  res.dialogId = req.dialogId;
  res.uptime = getUptime();
  res.cuuid = cuuid;
  res.xuuid = xuuid;
  res.faultIndicator = 0;
  res.operatingMode = operatingMode;
}

}
