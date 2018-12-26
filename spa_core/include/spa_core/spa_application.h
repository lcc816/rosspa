#ifndef SPA_APPLICATION_H
#define SPA_APPLICATION_H

#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_msgs/SpaProbe.h>
#include <spa_msgs/Hello.h>
#include <actionlib/client/simple_action_client.h>
#include <spa_msgs/SpaQueryAction.h>
#include <spa_msgs/SpaXteds.h>
#include <string>
#include <thread>

namespace spa
{
class SpaApplication
{
public:
  SpaApplication(const std::string &id, const ComponentType type, const std::string &uri);
  virtual ~SpaApplication() { shutdown(); }
  void init();
  virtual void appInit() = 0;
  virtual void run() = 0;
  void shutdown() { ros::shutdown(); spin_thread.join(); appShutdown(); }
  virtual void appShutdown() = 0;

  void setXuuid();
  std::string getXuuid();
  uint32_t getUptime();
  void registerRequest();
  void registerCommand();
  void registerNotification();
  void issueQuery(const std::string &query, const QueryType type);

private:
  ///< Response to LS for the XTEDS file.
  bool xtedsRegisterCallback(spa_msgs::SpaXteds::Request &req, spa_msgs::SpaXteds::Response &res);
  ///< for heartbeat probe from the SM-L
  bool beatCallback(spa_msgs::SpaProbe::Request &req, spa_msgs::SpaProbe::Response &res);
  ///< thread to start spinning in the backroud
  void spinThreadCallback() {ros::spin();}

  ros::Time startTime;
  std::string cuuid;
  ComponentType componentType;
  std::string xtedsUri;
  std::string xuuid;
  OperatingMode operatingMode;

  // relate to ROS
  std::string nodeName;
  ros::NodeHandle nh;
  ros::ServiceClient discoveryClient;
  ros::ServiceServer beatServer;
  ros::ServiceServer xtedsServer;
  typedef actionlib::SimpleActionClient<spa_msgs::SpaQueryAction> SpaQueryType;
  std::shared_ptr<SpaQueryType> queryClient;
  std::thread spin_thread;
};

} // namespace spa

#endif // SPA_APPLICATION_H
