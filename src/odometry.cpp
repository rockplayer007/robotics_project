#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
//rostopic echo /speed_L -b bag_1.bag -p > file.csv

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Speed LEFT: [%f]", msg->data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odometer");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("speed_L", 1000, chatterCallback);

  ros::spin();


  return 0;
}
