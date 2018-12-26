#include <spa_core/spa_application.h>
#include <ros/package.h>

namespace spa
{

class MyApplication : public SpaApplication
{
public:
  MyApplication(const std::string &id, ComponentType type, const std::string &uri) :
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
  init();

  ros::Rate rate(0.2);
  std::string query(
    "<SpaQuery msgType=\"Notification\">\n"
    " <Variable>\n"
    "   <Attribute name=\"kind\" operand=\"eq\" value=\"Temperature\">\n"
    "   <Attribute name=\"units\" operand=\"eq\" value=\"degC\">\n"
    " </Variable>\n"
    "</SpaQuery>"
  );
  issueQuery(query, SPA_REQREG_CURRENT);
  while (ros::ok())
  {
    ROS_INFO("I'm running!");
    rate.sleep();
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");

  std::string cuuid("00112233445566778899aabbccddee"); // length = 32
  std::string uri(ros::package::getPath("example_component") + "/xteds/Thermometer_Demo.xml");
  spa::MyApplication myApp(cuuid, spa::SPA_CMPTYPE_UNKNOWN, uri);
  myApp.run();
}
