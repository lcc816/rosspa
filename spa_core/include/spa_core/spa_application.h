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
  SpaApplication(const uuid_t &id, ComponentType type, const std::string &uri);
  virtual ~SpaApplication() {}
  void init();
  virtual void appInit() = 0;
  virtual void run() = 0;
  void shutdown() { ros::shutdown(); spin_thread.join(); appShutdown(); }
  virtual void appShutdown() = 0;

  void setXuuid();
  uuid_t getXuuid();
  uint32_t getUptime();
  void registerRequest();
  void registerCommand();
  void registerNotification();
  void issueQuery();

private:
  ///< Response to LS for the XTEDS file.
  bool xtedsRegisterCallback(spa_core::SpaXteds::Request &req, spa_core::SpaXteds::Response &res);
  ///< for heartbeat probe from the SM-L
  bool beatCallback(spa_core::SpaProbe::Request &req, spa_core::SpaProbe::Response &res);
  ///< thread to start spinning in the backroud
  void spinThreadCallback() {ros::spin();}

  uuid_t cuuid;
  ComponentType componentType;
  std::string xtedsUri;
  uuid_t xuuid;
  OperatingMode operatingMode;

  // relate to ROS
  std::string nodeName;
  ros::NodeHandle nh;
  ros::ServiceClient discoveryClient;
  ros::ServiceServer beatServer;
  ros::ServiceServer xtedsServer;
  std::thread spin_thread;
};

} // namespace spa

#endif // SPA_APPLICATION_H
