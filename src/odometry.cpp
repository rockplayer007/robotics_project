#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "project/floatStamped.h"

#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>



void callback(const project::floatStamped::ConstPtr& msg1, const project::floatStamped::ConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f) and (%f)", msg1->data, msg2->data);
}

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Speed LEFT: [%f]", msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry");

  ros::NodeHandle n; 

  message_filters::Subscriber<project::floatStamped> sub1(n, "speedL_stamped", 1);
  message_filters::Subscriber<project::floatStamped> sub2(n, "speedR_stamped", 1);

  //ros::Subscriber sub = n.subscribe("speed_L", 1000, chatterCallback);

  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<project::floatStamped, project::floatStamped> MySyncPolicy;
  
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  //ROS_INFO ("Received two messages");
  sync.registerCallback(boost::bind(&callback, _1, _2));


  //message_filters::TimeSynchronizer<project::floatStamped, project::floatStamped> sync(sub1, sub2, 1000);
  //sync.registerCallback(boost::bind(&callback, _1, _2));
  //ROS_INFO ("Received two messages");

  ros::spin();

  return 0;
}

