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
  nodeName = ros::this_node::getName(); // set node name
  setXuuid();

  regClient = nh.serviceClient<spa_sm_l::Hello>("local/hello");
  beatServer = std::ref(BeatActionServer(nh, nodeName + "/beat_action", \
               boost::bind(&SpaApplication::beatCallback, this, _1), false));
  xtedsServer = nh.serviceServer<std_srvs::Trigger>(nodeName + "/xteds", SpaApplication::xtedsRegisterCallback);
}

void SpaApplication::run()
{
  operatingMode = SPA_OPMODE_FULLY_OPERATIONAL;
  beatServer.start();
}

void SpaApplication::shutdown()
{
}

void SpaApplication::setXuuid()
{
  xuuid = 0xFFFFFFFF;
}

uint64_t SpaApplication::getXuuid()
{
  return xuuid;
}

void SpaApplication::registerRequest()
{
  spa_sm_l::Hello hello;
  hello.request.cuuid = cuuid;
  hello.request.componentType = componentType;

  ros::Rate rate(10);
  while (ros::ok())
  {
    if (regClient.call(hello))
    {
      ROS_INFO("%s: registered!", nodeName.c_str());
      break;
    }
    else {
      ROS_INFO("%s: failed to register, retry!", nodeName.c_str());
      rate.sleep();
    }
  }
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

void SpaApplication::beatCallback(const spa_core::SpaPorbeGoalConstPtr &goal)
{
  ros::Rate rate(goal->replyPeriod);
  spa_core::SpaProbeFeedback feedback;
  spa_core::SpaProbeResult result;

  feedback.dialogId = 0;
  feedback.cuuid = cuuid;
  feedback.xuuid = xuuid;

  while (ros::ok())
  {
    feedback.uptime = 0;
    feedback.faultIndicator = 0;
    feedback.operatingMode = operatingMode;
    beatServer.publishFeedback(feedback);
    rate.sleep();
  }

  result.resultMode = operatingMode;
  beatServer.setSucceeded(result);
}

}
