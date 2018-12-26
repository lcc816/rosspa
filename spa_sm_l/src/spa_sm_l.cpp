#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_msgs/Hello.h>
#include <spa_msgs/SpaProbe.h> // for heartbeat of components
#include <spa_msgs/SpaRequestLsProbe.h>
#include <std_srvs/Empty.h>
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
  bool showCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
  {
    showCurrentComponents(); return true;
  }

  // relate to ROS
  ros::NodeHandle nh;
  ros::ServiceServer discoverServer;
  ros::ServiceServer showServer;
  ros::ServiceClient beatClient;
  std::mutex comListMutex;
  std::map<const std::string, ComponentInfo> components;
  std::thread monitorThread;
  ros::Publisher reqLsProbePub;
};

SpaLocalManager::SpaLocalManager()
{
  discoverServer = nh.advertiseService("spa_sm_l/hello", &SpaLocalManager::discoverCallback, this);
  showServer = nh.advertiseService("spa_sm_l/show", &SpaLocalManager::showCallback, this);
  reqLsProbePub = nh.advertise<spa_msgs::SpaRequestLsProbe>("spa_ls/request_ls_probe", 100);
}

void SpaLocalManager::run()
{
  monitorThread = std::thread(boost::bind(&SpaLocalManager::monitorThreadCallback, this));
}

bool SpaLocalManager::discoverCallback(spa_msgs::Hello::Request& req, spa_msgs::Hello::Response& res)
{
  // 要不要用 status 返回 xTEDS 的注册情况?
  res.status = 0;
  std::string uuid = req.cuuid;
  ROS_INFO("discovered: %s\n                                cuuid = %s, type = %ld", \
          req.nodeName.c_str(), uuid.c_str(), (long int)req.componentType);

  // Notify Lookup Service to request the xTEDS.
  spa_msgs::SpaRequestLsProbe msg;
  msg.nodeName = req.nodeName;
  reqLsProbePub.publish(msg);

  // Added to discovered component list
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
    ROS_INFO("Node Name: %s"
      "\n                                CUUID: %s"
      "\n                                Type: %d"
      "\n                                Operating Mode: %d\n", \
      p.first.c_str(), p.second.cuuid.c_str(), p.second.componentType, \
      p.second.operatingMode);
  }
  comListMutex.unlock();
}

void SpaLocalManager::monitorThreadCallback()
{
  spa_msgs::SpaProbe srv;
  ros::Duration duration(5);
  while (ros::ok())
  {
    comListMutex.lock();
    for (std::pair<const std::string, ComponentInfo> &p : components)
    {
      int retry = 3;
      beatClient = nh.serviceClient<spa_msgs::SpaProbe>(p.first + "/heartbeat");
      while (!beatClient.call(srv) && retry)
      {
        // delay for 0.5s and retry
        ros::Duration(0.5).sleep();
        retry--;
      }
      if (retry > 0)
      {
        p.second.operatingMode = OperatingMode(srv.response.operatingMode);
      }
      else
      {
        ROS_INFO("component %s is offline!", p.first.c_str());
        components.erase(p.first);
      }
    }
    comListMutex.unlock();
    duration.sleep();
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
