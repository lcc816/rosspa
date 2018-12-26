#include <spa_core/spa_application.h>
#include <functional>
#include <fstream>
#include <sstream>

namespace spa
{

SpaApplication::SpaApplication(const std::string &id, const ComponentType type, const std::string &uri) :
  cuuid(id),
  componentType(type),
  xtedsUri(uri),
  operatingMode(SPA_OPMODE_INITIALIZING)
{
}

void SpaApplication::init()
{
  // set starting time
  startTime = ros::Time::now();
  // set node name
  nodeName = ros::this_node::getName();
  setXuuid();

  discoveryClient = nh.serviceClient<spa_msgs::Hello>("spa_sm_l/hello");
  beatServer = nh.advertiseService(nodeName + "/heartbeat", &SpaApplication::beatCallback, this);
  xtedsServer = nh.advertiseService(nodeName + "/xteds", &SpaApplication::xtedsRegisterCallback, this);
  queryClient = std::make_shared<SpaQueryType>("spa_ls/spa_query", false);

  // create a thread to start spinning in the background
  spin_thread = std::thread(boost::bind(&SpaApplication::spinThreadCallback, this));

  spa_msgs::Hello hello;
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
    uuid_t temp;
    uuid_create_sha1_from_name(&temp, NameSpace_DNS, data.c_str(), data.size());
    xuuid = temp.toString();
  }
  stream.clear();
  stream.close();
}

uint32_t SpaApplication::getUptime()
{
  ros::Duration uptime(ros::Time::now() - startTime);
  return uptime.sec;
}

std::string SpaApplication::getXuuid()
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

void SpaApplication::issueQuery(const std::string &query, const QueryType type)
{
  spa_msgs::SpaQueryGoal goal;
  goal.nodeName = nodeName;
  goal.dialogId = 0;
  goal.queryType = type;
  goal.query = query;
  
  queryClient->sendGoal(goal);
}

bool SpaApplication::xtedsRegisterCallback(spa_msgs::SpaXteds::Request &req, spa_msgs::SpaXteds::Response &res)
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

bool SpaApplication::beatCallback(spa_msgs::SpaProbe::Request &req, spa_msgs::SpaProbe::Response &res)
{
  res.dialogId = req.dialogId;
  res.uptime = getUptime();
  res.cuuid = cuuid;
  res.xuuid = xuuid;
  res.faultIndicator = 0;
  res.operatingMode = operatingMode;
}

}
