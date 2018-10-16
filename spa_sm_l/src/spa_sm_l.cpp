#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_core/Hello.h>
#include <spa_core/SpaProbe.h> // for heartbeat of components
#include <actionlib/client/simple_action_client.h>
#include <spa_core/SpaProbeAction.h>
#include <spa_core/SpaLookupServiceProbe.h>
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
};

SpaLocalManager::SpaLocalManager()
{
  discoverServer = nh.advertiseService("local/hello", &SpaLocalManager::discoverCallback, this);
  beatClient = nh.serviceClient<spa_core::SpaLookupServiceProbe>("spa_lookup_service_probe");
}

void SpaLocalManager::run()
{
  componentsMonitor = std::thread(boost::bind(&SpaLocalManager::monitorThread, this));
}

bool SpaLocalManager::discoverCallback(spa_core::Hello::Request& req, spa_core::Hello::Response& res)
{
  res.status = 1;

  ComponentInfo com(req.cuuid, ComponentType(req.componentType));
  comListMutex.lock();
  components[req.nodeName] = com;
  comListMutex.unlock();

  ROS_INFO("discovered: %s", req.nodeName.c_str());
  ROS_INFO("\tcuuid = %ld, type = %ld", (long int)req.cuuid, (long int)req.componentType);
  ROS_INFO("sending back acknowledge message");
  return true;
}

void SpaLocalManager::showCurrentComponents()
{
  comListMutex.lock();
  for (auto &p : components)
  {
    ROS_INFO("Name: %s\n", p.first.c_str());
    ROS_INFO("CUUID: %ld", p.second.cuuid);
    ROS_INFO("Type: %d", p.second.componentType);
    ROS_INFO("Operating Mode: %d", p.second.operatingMode);
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
  ROS_INFO("Ready to discovery components.");
  ros::spin();

  return 0;
}
