#include <spa_core/spa_application.h>
#include <ros/package.h>
#include <spa_msgs/SpaData.h>

namespace spa
{

class MyApplication : public SpaApplication
{
public:
  MyApplication(const std::string &id, ComponentType type, const std::string &uri) :
    SpaApplication(id, type, uri)
  {
    providerName.clear();
    topicName.clear();
  }
  ~MyApplication() {}
  void appInit();
  void run();
  void appShutdown() {}

private:
  void showDataCb(const spa_msgs::SpaData &msg);

  void queryDoneCb( const actionlib::SimpleClientGoalState &state, 
                    const spa_msgs::SpaQueryResultConstPtr &result) 
  {
    ROS_INFO("query done!");
  }

  void queryActiveCb() {}

  void queryFeedbackCb(const spa_msgs::SpaQueryFeedbackConstPtr &feedback)
  {
    providerName = feedback->nodeName;
    ROS_INFO("found matching nodes: %s", providerName.c_str());
    topicName = providerName + '/' + feedback->interfaceName + '/' + feedback->messageName;
  }

  ros::Subscriber sub;
  // data provider
  std::string providerName;
  // message topic
  std::string topicName;
};

void MyApplication::appInit()
{

}

void MyApplication::run()
{
  init(); // 自动处理发现和注册过程

  ros::Rate rate(0.2);
  std::string query(
    "<SpaQuery msgType=\"Notification\">\n"
    " <Variable>\n"
    "   <Attribute name=\"kind\" operand=\"eq\" value=\"Temperature\"/>\n"
    "   <Attribute name=\"units\" operand=\"eq\" value=\"degC\"/>\n"
    "   <Attribute name=\"dataType\" operand=\"eq\" value=\"FLOAT32\"/>\n"
    " </Variable>\n"
    "</SpaQuery>"
  );
  ROS_INFO("publish query...");
  issueQuery(query, SPA_REQREG_CURRENT, 
    boost::bind(&MyApplication::queryDoneCb, this, _1, _2),
    boost::bind(&MyApplication::queryActiveCb, this),
    boost::bind(&MyApplication::queryFeedbackCb, this, _1));
  waitForQueryResult(); // 等待查询结果返回
  if (!topicName.empty())
  {
    ROS_INFO("subscribe to %s", topicName.c_str());
    sub = nh.subscribe(topicName, 10, &MyApplication::showDataCb, this);
  }
  while (ros::ok())
  {
    rate.sleep();
  }
}

void MyApplication::showDataCb(const spa_msgs::SpaData &msg)
{
  if (msg.payloadLength == 4)
  {
    float temp;
    memcpy(&temp, &msg.payload.front(), 4);
    ROS_INFO("current temperature is %f", temp);
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");

  std::string cuuid("00112233445566778899aabbccddeeff"); // length = 32
  std::string uri(ros::package::getPath("example_component") + "/xteds/Thermometer_Demo.xml");
  spa::MyApplication myApp(cuuid, spa::SPA_CMPTYPE_UNKNOWN, uri);
  myApp.run();
}
