#include <ros/ros.h>
#include <spa_sm_l/Hello.h>
#include <spa/component.h>
#include <list>
#include <memory>

namespace spa
{

class SpaLocalManager : public Component
{
public:
  SpaLocalManager(int argc, char **argv, std::string name);
private:
  bool discoverCallback(spa_sm_l::Hello::Request& req, spa_sm_l::Hello::Response& res);

  ///< relate to ROS
  ros::NodeHandle nh;
  ros::ServiceServer discoverServer;
  ros::ServiceClient probeClient;

  std::list<Component> components;
};

SpaLocalManager::SpaLocalManager(int argc, char **argv, std::string name)
{
  ros::init(argc, argv, "spa_sm_l");
  discoverServer = nh.advertiseService("local/hello", SpaLocalManager::discoverCallback);
  probeClient = nh.serviceClient<spa_ls::SpaLookupServiceProbe>("spa_lookup_service_probe");
}

bool SpaLocalManager::discoverCallback(spa_sm_l::Hello::Request& req, spa_sm_l::Hello::Response& res)
{
  res.status = 1;

  ComponentInfo com(req.nodeName, req.cuuid, req.componentType);
  components.push_back(com);

  ROS_INFO("request: cuuid = %ld, type = %ld", (long int)req.cuuid, (long int)req.componentType);
  ROS_INFO("sending back acknowledge message");
  return true;
}

}

int main(int argc, char **argv)
{

  SpaLocalManager sml;
  ROS_INFO("Ready to discovery components.");
  ros::spin();

  return 0;
}
