#include <ros/ros.h>
#include <spa_core/spa_application.h>

namespace spa
{

class MyApplication : public SpaApplication
{
public:
  MyApllication(uint64_t id, uint8_t type, std::string &uri) :
    SpaApplicatin(id, type, uri)
  {}
  void run();
  void appInit();
  void appShutdown();
};

void MyApplication::run()
{
  //setXuuid();
  init();
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

  spa::MyApplication myApp(123456, spa::PA_CMPTYPE_UNKNOWN, "../xteds/Thermometer_Demo.xml");
  myApp.run();
}
