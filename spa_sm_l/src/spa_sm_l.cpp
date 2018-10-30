#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_core/Hello.h>
#include <spa_core/SpaProbe.h> // for heartbeat of components
#include <actionlib/client/simple_action_client.h>
#include <spa_core/SpaProbeAction.h>
#include <spa_core/SpaRequestLsProbe.h>
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
  ~SpaLocalManager() {ros::shutdown(); componentsMonitor.join();}
  void run();
  void showCurrentComponents();
private:
  bool discoverCallback(spa_core::Hello::Request& req, spa_core::Hello::Response& res);
  void monitorThread();

  // relate to ROS
  ros::NodeHandle nh;
  ros::ServiceServer discoverServer;
  ros::ServiceClient beatClient;
  std::mutex comListMutex;
  std::map<const std::string, ComponentInfo> components;
  std::thread componentsMonitor;
  ros::ServiceClient probeClient;
};

SpaLocalManager::SpaLocalManager()
{
  discoverServer = nh.advertiseService("spa_sm_l/hello", &SpaLocalManager::discoverCallback, this);
  probeClient = nh.serviceClient<spa_core::SpaRequestLsProbe>("spa_ls/request_probe");
}

void SpaLocalManager::run()
{
  componentsMonitor = std::thread(boost::bind(&SpaLocalManager::monitorThread, this));
}

bool SpaLocalManager::discoverCallback(spa_core::Hello::Request& req, spa_core::Hello::Response& res)
{
  // 要不要用 status 返回 xTEDS 的注册情况?
  res.status = 0;

  ROS_INFO("discovered: %s\ncuuid = %ld, type = %ld", req.nodeName.c_str(), \
           (long int)req.cuuid, (long int)req.componentType);

  spa_core::SpaRequestLsProbe srv;
  srv.request.nodeName = req.nodeName;
  if (probeClient.call(srv))
  {
      ROS_INFO("registered xTEDS of %s", req.nodeName.c_str());
  }
  else
  {
    ROS_ERROR("failed to register xTEDS!");
    return false;
  }

  ComponentInfo com(req.cuuid, ComponentType(req.componentType));
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
    ROS_INFO("Node Name: %s\nCUUID: %ld\nType: %d\nOperating Mode: %d", \
             p.first.c_str(), p.second.cuuid, p.second.componentType, p.second.operatingMode);
  }
  comListMutex.unlock();
}

void SpaLocalManager::monitorThread()
{
  spa_core::SpaProbe srv;
  ros::Rate rate(5);
  int retry = 3;
  while (ros::ok())
  {
    comListMutex.lock();
    for (std::pair<const std::string, ComponentInfo> &p : components)
    {
      beatClient = nh.serviceClient<spa_core::SpaProbe>(p.first + "/heartbeat");
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
