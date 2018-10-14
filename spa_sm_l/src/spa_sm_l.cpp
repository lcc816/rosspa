#include <ros/ros.h>
#include <spa_sm_l/Hello.h>
#include <spa_core/spa_common.h>
#include <actionlib/client/simple_action_client.h>
#include <spa_core/SpaProbeAction.h> // for heartbeat of components
#include <mutex>
#include <list>
#include <memory>

namespace spa
{

class SpaLocalManager
{
public:
  SpaLocalManager();
private:
  bool discoverCallback(spa_sm_l::Hello::Request& req, spa_sm_l::Hello::Response& res);

  // relate to ROS
  ros::NodeHandle nh;
  ros::ServiceServer discoverServer;
  ros::ServiceClient probeClient;
  std::mutex comListMutex;
  std::list<ComponentInfo> components;
};

SpaLocalManager::SpaLocalManager()
{
  discoverServer = nh.advertiseService("local/hello", SpaLocalManager::discoverCallback, this);
  probeClient = nh.serviceClient<spa_ls::SpaLookupServiceProbe>("spa_lookup_service_probe");
}

bool SpaLocalManager::discoverCallback(spa_sm_l::Hello::Request& req, spa_sm_l::Hello::Response& res)
{
  res.status = 1;

  ComponentInfo com(req.nodeName, req.cuuid, req.componentType);
  comListMutex.lock();
  components.push_back(com);
  comListMutex.unlock();

  ROS_INFO("discovered: cuuid = %ld, type = %ld", (long int)req.cuuid, (long int)req.componentType);
  ROS_INFO("sending back acknowledge message");
  return true;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spa_sm_l");
  SpaLocalManager sml;
  ROS_INFO("Ready to discovery components.");
  ros::spin();

  return 0;
}
