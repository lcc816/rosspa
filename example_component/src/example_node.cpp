#include <ros/ros.h>
#include <spa_core/spa_application.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<spa_sm_l::Hello>("local/hello");
  spa_sm_l::Hello hello;
  hello.request.cuuid = 123456;
  hello.request.componentType = 1;

  ros::Rate rate(10);
  while (ros::ok())
  {
    if(client.call(hello))
    {
      ROS_INFO("example_node registered!");
      break;
    }
    else
    {
      ROS_INFO("failed to registeration, retry.");
      rate.sleep();
    }
  }
}
