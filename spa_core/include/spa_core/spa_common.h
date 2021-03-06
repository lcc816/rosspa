#ifndef SPA_COMMON_H
#define SPA_COMMON_H

#include <string>
#include <spa_core/uuid.h>

namespace spa 
{

enum ComponentType
{
  SPA_CMPTYPE_UNKNOWN = 0,
  SPA_CMPTYPE_LS,
  SPA_CMPTYPE_SM_L,
  SPA_CMPTYPE_SM_X,
  SPA_CMPTYPE_LCL,
  SPA_CMPTYPE_SPW,
  SPA_CMPTYPE_USB,
  SPA_CMPTYPE_I2C,
  SPA_CMPTYPE_OPT
};

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

enum QueryType
{
  SPA_REQREG_CURRENT = 0,
  SPA_REQREG_CURRENT_AND_FUTURE = 5,
  SPA_REQREG_CURRENT_FUTURE_AND_CANCELLATIONS = 7,
  SPA_REQREG_CANCEL = 11
};

enum ReplyType
{
  REGISTRATION = 0,
  CANCELLATION
};

struct ComponentInfo
{
  ComponentInfo(const std::string &uuid, const ComponentType type) :
    cuuid(uuid),
    componentType(type),
    operatingMode(SPA_OPMODE_INITIALIZING)
  {}

  ComponentInfo() {}

  std::string cuuid;
  ComponentType componentType;
  OperatingMode operatingMode;
};

}

#endif // SPA_COMMON_H
