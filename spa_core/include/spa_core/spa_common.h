#ifndef SPA_COMMON_H
#define SPA_COMMON_H

#include <string>

namespace spa 
{

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

struct CompomnentInfo
{
  CompomnentInfo(std::string name, uint64_t uuid, ComponentType type) :
    nodeName(name),
    cuuid(uuid),
    componentType(type)
  {}

  CompomnentInfo() {}

  std::string nodeName;
  uint64_t cuuid;
  ComponentType componentType;
};

}

#endif // SPA_COMMON_H
