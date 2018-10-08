#ifndef SPA_APPLICATION_H
#define SPA_APPLICATION_H

#include <string>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <spa_core/SpaPobeAction.h>

namespace spa
{
class SpaApplication
{
public:
  SpaApplication(uint64_t id, uint8_t type, std::string &uri);
  void run();
  void shutdown();
  void setXuuid();
  uint64 getXuuid();
  void registerRequest();
  void issueQuery();

private:
  ///< Response to LS for the XTEDS file.
  void xtedsRegisterCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  ///< Response to SPA probe request
  void beatCallback(const spa_core::SpaPorbeGoalConstPtr &goal);

  uint64_t cuuid;
  enum ComponentType
  {
    SPA_CMPTYPE_UNKNOWN = 0,
    SPA_CMPTYPE_LS = 1,
    SPA_CMPTYPE_SM_L = 2,
    SPA_CMPTYPE_SM_X = 3,
    SPA_CMPTYPE_SPW = 4,
    SPA_CMPTYPE_USB = 5,
    SPA_CMPTYPE_I2C = 6,
    SPA_CMPTYPE_OP = 7
  };
  ComponentType componentType;
  std::string xtedsUri;
  uint64_t xuuid;
  enum OperatingMode
  {
    SPA_OPMODE_INITIALIZING = 0,
    SPA_OPMODE_FULLY_OPERATIONAL = 5,
    SPA_OPMODE_DEPENDENCY_FAILURE = 7,
    SPA_OPMODE_LINK_ANOMALY = 11,
    SPA_OPMODE_PACKET_ANOMALY = 13,
    SPA_OPMODE_DELIVERY_ANOMALY = 15,
    SPA_OPMODE_TEMPERATURE_ANOMALY = 21,
    SPA_OPMODE_VOLTAGE_ANOMALY = 23,
    SPA_OPMODE_CURRENT_ANOMALY = 27,
    SPA_OPMODE_PRESSURE_ANOMALY = 29,
    SPA_OPMODE_TIME_DISTRIBUTION_ANOMALY = 31,
    SPA_OPMODE_INTERNAL_SELFTEST_ANOMALY = 37,
    SPA_OPMODE_APPLICATION_LEVEL_ANOMALY = 39,
    SPA_OPMODE_USER_DEFINED_ANOMALY = 41
  };
  OperatingMode operatingMode;

  // relate to ROS
  std::string nodeName;
  ros::Nodehandle nh;
  ros::ServiceClient regClient;
  typedef actionlib::SimpleActionServer<spa_core::SpaPorbe> BeatActionServer;
  BeatActionServer beatServer;
  ros::ServiceServer xtedsServer;
};

typedef SpaApplication::ComponentType ComponentType;

}

#endif // SPA_APPLICATION_H
