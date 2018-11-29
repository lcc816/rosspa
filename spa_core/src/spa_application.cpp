#include <spa_core/spa_application.h>
#include <functional>
#include <fstream>
#include <sstream>

namespace spa
{

SpaApplication::SpaApplication(const uuid_t &id, ComponentType type, const std::string &uri) :
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
  spin_thread = std::thread(boost::bind(&SpaApplication::spinThreadCallback, this));

  spa_core::Hello hello;
  hello.request.nodeName = nodeName;
  hello.request.cuuid = cuuid.serialize();
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
  using namespace std;
  // load xteds file to a string
  ifstream stream(xtedsUri.c_str(), ios::binary);
  if (!stream.is_open())
  {
    ROS_ERROR("cannot set xteds uuid: %s", xtedsUri.c_str());
  }
  else
  {
    stringstream buffer;
    buffer << stream.rdbuf();
    string data(buffer.str());
    // set xuuid
    uuid_create_sha1_from_name(&xuuid, NameSpace_DNS, data.c_str(), data.size());
  }
  stream.clear();
  stream.close();
}

uint32_t SpaApplication::getUptime()
{
  return 0;
}

uuid_t SpaApplication::getXuuid()
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

  std::stringstream buffer;
  buffer << fin.rdbuf();
  res.xteds = buffer.str();
  fin.clear();
  fin.close();

  return true;
}

void SpaApplication::probeCallback(const spa_core::SpaProbeGoalConstPtr &goal)
{
  ros::Rate rate(1 / (goal->replyPeriod));
  spa_core::SpaProbeFeedback feedback;
  spa_core::SpaProbeResult result;

  feedback.dialogId = goal->dialogId;
  feedback.cuuid = cuuid.serialize();
  feedback.xuuid = xuuid.serialize();

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
  else // if repyCount == 0, forever feedback
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
  res.cuuid = cuuid.serialize();
  res.xuuid = xuuid.serialize();
  res.faultIndicator = 0;
  res.operatingMode = operatingMode;
}

}
