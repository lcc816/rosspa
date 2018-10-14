#ifndef SPA_APPLICATION_H
#define SPA_APPLICATION_H

#include <string>
#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_core/SpaProbe.h>
#include <actionlib/server/simple_action_server.h>
#include <spa_core/SpaPobeAction.h>
#include <std_srvs/Trigger.h>

namespace spa
{
class SpaApplication
{
public:
  SpaApplication(uint64_t id, uint8_t type, std::string &uri);
  virtual void run() = 0;
  void init();
  virtual void appInit() = 0;
  void shutdown();
  virtual void appShutdown() = 0;
  void setXuuid();
  uint64_t getXuuid();
  uint32_t getUptime();
  void registerRequest();
  void registerCommand();
  void registerNotification();
  void issueQuery();

private:
  ///< Response to LS for the XTEDS file.
  void xtedsRegisterCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  ///< Response to SPA probe request
  void probeCallback(const spa_core::SpaPorbeGoalConstPtr &goal);
  ///< for heartbeat probe from the SM-L
  void beatCallback(spa_core::SpaProbe::Request &req, spa_core::SpaProbe::Response &res);

  uint64_t cuuid;
  ComponentType componentType;
  std::string xtedsUri;
  uint64_t xuuid;
  OperatingMode operatingMode;

  // relate to ROS
  std::string nodeName;
  ros::Nodehandle nh;
  ros::ServiceClient discoveryClient;
  ros::ServiceServer beatServer;
  typedef actionlib::SimpleActionServer<spa_core::SpaPorbe> ProbeActionServer;
  ProbeActionServer probeServer;
  ros::ServiceServer xtedsServer;
};

}

#endif // SPA_APPLICATION_H
