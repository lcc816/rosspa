#include <spa_core/spa_application.h>
#include <ros/package.h>
#include <spa_msgs/SpaCommand.h>
#include <iostream>
#include <cstring>

namespace spa
{

class MyApplication : public SpaApplication
{
public:
    MyApplication(const std::string &id, ComponentType type, const std::string &uri) :
        SpaApplication(id, type, uri),
    {
        providerName.clear();
        topicName.clear();
        std::memset(speedXYZ, 0, sizeof speedXYZ);
    }
    ~MyApplication() {}
    void appInit();
    void run();
    void appShutdown() {}

private:
    void queryDoneCb( const actionlib::SimpleClientGoalState &state,
                      const spa_msgs::SpaQueryResultConstPtr &result)
    {
        if (!result->resultMode)
            ROS_INFO("query done!");
        else
            ROS_ERROR("fail to query!");
    }

    void queryActiveCb() {}

    void queryFeedbackCb(const spa_msgs::SpaQueryFeedbackConstPtr &feedback)
    {
        providerName = feedback->nodeName;
        ROS_INFO("found matching nodes: %s", providerName.c_str());
        topicName = providerName + '/' + feedback->interfaceName + '/' + feedback->messageName;
    }

    ros::Publisher pub;
    // service provider
    std::string providerName;
    // message topic
    std::string topicName;
    // speeds array
    int32_t speedXYZ[3];
};

void MyApplication::appInit()
{

}

void MyApplication::run()
{
    init(); // 自动处理发现和注册过程

    ros::Rate rate(0.2);
    std::string query(
        "<SpaQuery msgType=\"Command\">\n"
        " <Variable>\n"
        "   <Attribute name=\"name\" operand=\"eq\" value=\"speedX\"/>\n"
        "   <Attribute name=\"kind\" operand=\"eq\" value=\"WheelSpeed\"/>\n"
        "   <Attribute name=\"dataType\" operand=\"eq\" value=\"INT32\"/>\n"
        " </Variable>\n"
        " <Variable>\n"
        "   <Attribute name=\"name\" operand=\"eq\" value=\"speedY\"/>\n"
        "   <Attribute name=\"kind\" operand=\"eq\" value=\"WheelSpeed\"/>\n"
        "   <Attribute name=\"dataType\" operand=\"eq\" value=\"INT32\"/>\n"
        " </Variable>\n"
        " <Variable>\n"
        "   <Attribute name=\"name\" operand=\"eq\" value=\"speedZ\"/>\n"
        "   <Attribute name=\"kind\" operand=\"eq\" value=\"WheelSpeed\"/>\n"
        "   <Attribute name=\"dataType\" operand=\"eq\" value=\"INT32\"/>\n"
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
        ROS_INFO("subscribe to %s", providerName.c_str());
        pub = nh.advertise<spa_msgs::SpaCommand>(topicName, 100);
    }
    while (ros::ok())
    {
        std::cout << "Input speeds of x y z: ";
        std::cin >> speedXYZ[0] >> speedXYZ[1] >> speedXYZ[2];
        spa_msgs::SpaCommand cmd;
        cmd.payloadLength = 12;
        cmd.interfaceId = 1;
        cmd.messageId = 1;
        std::memcpy(cmd.payload, speedXYZ, sizeof speedXYZ);
        pub.publish(cmd);
        rate.sleep();
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flywheel_control");

    std::string cuuid("6b33ec36c23152aea4ee0f49d954ebb3"); // length = 32
    std::string uri(ros::package::getPath("example_component") + "/xteds/flywheel_control.xml");
    spa::MyApplication myApp(cuuid, spa::SPA_CMPTYPE_UNKNOWN, uri);
    myApp.run();
}
