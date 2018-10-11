#include <spa/spa_application.h>
#include <functional>

namespace spa
{

SpaApplication::SpaApplication(uint64_t id, uint8_t type, std::string &uri) :
  cuuid(id),
  componentType(type),
  xtedsUri(uri),
  operatingMode(SPA_OPMODE_INITIALIZING)
{
}

void SpaApplication::init()
{
  nodeName = ros::this_node::getName(); // set node name
  setXuuid();

  discoveryClient = nh.serviceClient<spa_sm_l::Hello>("local/hello");
  beatServer = nh.advertiseService(nodeName + "/heartbeat", &SpaApplication::beatCallback, this);
  probeServer = std::ref(ProbeActionServer(nh, nodeName + "/spa_probe", \
               boost::bind(&SpaApplication::probeCallback, this, _1), false));
  probeServer.start();
  xtedsServer = nh.advertiseService(nodeName + "/xteds", &SpaApplication::xtedsRegisterCallback, this);

  spa_sm_l::Hello hello;
  hello.request.nodeName = nodeName;
  hello.request.cuuid = cuuid;
  hello.request.componentType = componentType;

  ros::Rate rate(2);
  while (ros::ok())   // Waiting to be discovered by the SM-L
  {
    if (discoveryClient.call(hello))
    {
      ROS_INFO("%s: discovered!", nodeName.c_str());
      break;
    }
    else {
      ROS_INFO("%s: failed to be discovered, retry!", nodeName.c_str());
      rate.sleep();
    }
  }

  // All is fine, call user's initial functions.
  appInit();
  operatingMode = SPA_OPMODE_FULLY_OPERATIONAL;
}

void SpaApplication::shutdown()
{
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

bool SpaApplication::xtedsRegisterCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  FILE *fp;
  if (!(fp = fopen(xtedsUri.c_str(), "rb")))
  {
    std::cout << nodeName << ": unable to get xTEDS\n";
    return false;
  }
  char c = fgetc(fp);
  while (!feof(fp))
  {
    res.message += c;
    c = fgetc(fp);
  }
  fclose(fp);

  return true;
}

void SpaApplication::probeCallback(const spa_core::SpaPorbeGoalConstPtr &goal)
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
    while (ros::ok())
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
    while (ros::ok() && count--)
    {
      feedback.uptime = 0;
      feedback.faultIndicator = 0;
      feedback.operatingMode = operatingMode;
      beatServer.publishFeedback(feedback);
      rate.sleep();
    }
  }

  result.resultMode = operatingMode;
  beatServer.setSucceeded(result);
}

void beatCallback(spa_core::SpaProbe::Request &req, spa_core::SpaProbe::Response &res)
{
  res.dialogId = req.dialogId;
  res.uptime = getUptime();
  res.cuuid = cuuid;
  res.xuuid = xuuid;
  res.faultIndicator = 0;
  res.operatingMode = operatingMode;
}

}
