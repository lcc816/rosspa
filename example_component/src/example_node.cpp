#include <spa_core/spa_application.h>

namespace spa
{

class MyApplication : public SpaApplication
{
public:
  MyApplication(const uuid_t &id, ComponentType type, const std::string &uri) :
    SpaApplication(id, type, uri)
  {}
  ~MyApplication() {}
  void appInit();
  void run();
  void appShutdown() {}
};

void MyApplication::appInit()
{

}

void MyApplication::run()
{
  //setXuuid();
  init();

  ros::Rate rate(0.2);
  while (ros::ok())
  {
    ROS_INFO("I'm running!");
    rate.sleep();
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_component");

  spa::uuid_t cuuid = {0x01234567, 0x89ab, 0xcdef, 0x01, 0x23, {0x45, 0x67, 0x89, 0xab, 0xcd, 0xef}};
  spa::MyApplication myApp(cuuid, spa::SPA_CMPTYPE_UNKNOWN, "/home/lcc/catkin_ws/src/rosspa/example_component/xteds/Thermometer_Demo.xml");
  myApp.run();
}
