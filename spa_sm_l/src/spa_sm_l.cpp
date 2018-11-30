#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_msgs/Hello.h>
#include <spa_msgs/SpaProbe.h> // for heartbeat of components
#include <spa_msgs/SpaRequestLsProbe.h>
#include <iostream>
#include <mutex>
#include <map>
#include <memory>
#include <thread>
#include <algorithm>

namespace spa
{

class SpaLocalManager
{
public:
  SpaLocalManager();
  ~SpaLocalManager() {ros::shutdown(); monitorThread.join();}
  void run();
  void showCurrentComponents();
private:
  bool discoverCallback(spa_msgs::Hello::Request& req, spa_msgs::Hello::Response& res);
  void monitorThreadCallback();

  // relate to ROS
  ros::NodeHandle nh;
  ros::ServiceServer discoverServer;
  ros::ServiceClient beatClient;
  std::mutex comListMutex;
  std::map<const std::string, ComponentInfo> components;
  std::thread monitorThread;
  ros::ServiceClient requestProbeClient;
};

SpaLocalManager::SpaLocalManager()
{
  discoverServer = nh.advertiseService("spa_sm_l/hello", &SpaLocalManager::discoverCallback, this);
  requestProbeClient = nh.serviceClient<spa_msgs::SpaRequestLsProbe>("spa_ls/request_probe");
}

void SpaLocalManager::run()
{
  monitorThread = std::thread(boost::bind(&SpaLocalManager::monitorThreadCallback, this));
}

bool SpaLocalManager::discoverCallback(spa_msgs::Hello::Request& req, spa_msgs::Hello::Response& res)
{
  // 要不要用 status 返回 xTEDS 的注册情况?
  res.status = 0;
  uuid_t uuid;
  uuid.deserialize(req.cuuid);
  ROS_INFO("discovered: %s\ncuuid = %s, type = %ld", req.nodeName.c_str(), \
           uuid.toString().c_str(), (long int)req.componentType);

  // Notify Lookup Service to request the xTEDS.
  spa_msgs::SpaRequestLsProbe srv;
  srv.request.nodeName = req.nodeName;
  if (requestProbeClient.call(srv))
  {
      ROS_INFO("registered xTEDS of %s", req.nodeName.c_str());
  }
  else
  {
    ROS_ERROR("failed to register xTEDS!");
    return false;
  }

  ComponentInfo com(uuid, ComponentType(req.componentType));
  comListMutex.lock();
  components[req.nodeName] = com;
  comListMutex.unlock();
  
  res.status = 1;
  return true;
}

void SpaLocalManager::showCurrentComponents()
{
  comListMutex.lock();
  for (auto &p : components)
  {
    ROS_INFO("Node Name: %s\nCUUID: %s\nType: %d\nOperating Mode: %d", \
      p.first.c_str(), p.second.cuuid.toString().c_str(), p.second.componentType, \
      p.second.operatingMode);
  }
  comListMutex.unlock();
}

void SpaLocalManager::monitorThreadCallback()
{
  spa_msgs::SpaProbe srv;
  ros::Rate rate(5);
  int retry = 3;
  while (ros::ok())
  {
    comListMutex.lock();
    for (std::pair<const std::string, ComponentInfo> &p : components)
    {
      beatClient = nh.serviceClient<spa_msgs::SpaProbe>(p.first + "/heartbeat");
      while (!beatClient.call(srv) && retry)
      {
        // delay and retry
        usleep(1000);
        retry--;
      }
      if (retry > 0)
      {
        p.second.operatingMode = OperatingMode(srv.response.operatingMode);
      }
      else
      {
        components.erase(p.first);
      }
    }
    comListMutex.unlock();
    rate.sleep();
  }

}

} // namespace spa

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spa_sm_l");
  spa::SpaLocalManager sml;
  sml.run();
  ROS_INFO("Ready to discover components.");
  ros::spin();

  return 0;
}
