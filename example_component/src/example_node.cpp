#include <ros/ros.h>
#include <spa_core/spa_application.h>

namespace spa
{

class MyApplication : public SpaApplication
{
public:
  MyApplication(uint64_t id, ComponentType type, const std::string &uri) :
    SpaApplication(id, type, uri)
  {}
  void run();
  void appInit();
  void appShutdown();
};

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

void MyApplication::appInit()
{

}

void MyApplication::appShutdown()
{

}

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");

  spa::MyApplication myApp(123456, spa::SPA_CMPTYPE_UNKNOWN, "../xteds/Thermometer_Demo.xml");
  myApp.run();
}
