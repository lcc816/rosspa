#include <spa_core/spa_application.h>

namespace spa
{

class MyApplication : public SpaApplication
{
public:
  MyApplication(uint64_t id, ComponentType type, const std::string &uri) :
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
//    ros::spinOnce();
    rate.sleep();
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_component");

  spa::MyApplication myApp(123456, spa::SPA_CMPTYPE_UNKNOWN, "/home/lcc/catkin_ws/src/rosspa/example_component/xteds/Thermometer_Demo.xml");
  myApp.run();
}
