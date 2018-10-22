#ifndef SPA_APPLICATION_H
#define SPA_APPLICATION_H

#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_core/SpaProbe.h>
#include <spa_core/Hello.h>
#include <actionlib/server/simple_action_server.h>
#include <spa_core/SpaProbeAction.h>
#include <spa_core/SpaXteds.h>
#include <string>
#include <thread>

namespace spa
{
class SpaApplication
{
public:
  SpaApplication(uint64_t id, ComponentType type, const std::string &uri);
  virtual ~SpaApplication() {}
  virtual void run() = 0;
  void init();
  virtual void appInit() = 0;
  void shutdown() {ros::shutdown(); spin_thread.join();}
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
  bool xtedsRegisterCallback(spa_core::SpaXteds::Request &req, spa_core::SpaXteds::Response &res);
  ///< Response to SPA probe request
  void probeCallback(const spa_core::SpaProbeGoalConstPtr &goal);
  ///< for heartbeat probe from the SM-L
  bool beatCallback(spa_core::SpaProbe::Request &req, spa_core::SpaProbe::Response &res);
  ///< thread to start spinning in the backroud
  void spinThread() {ros::spin();}

  uint64_t cuuid;
  ComponentType componentType;
  std::string xtedsUri;
  uint64_t xuuid;
  OperatingMode operatingMode;

  // relate to ROS
  std::string nodeName;
  ros::NodeHandle nh;
  ros::ServiceClient discoveryClient;
  ros::ServiceServer beatServer;
  typedef actionlib::SimpleActionServer<spa_core::SpaProbeAction> ProbeActionServer;
  ProbeActionServer probeServer;
  ros::ServiceServer xtedsServer;
  std::thread spin_thread;
};

} // namespace spa

#endif // SPA_APPLICATION_H
