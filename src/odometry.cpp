#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "project/floatStamped.h"

#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<message_filters/cache.h>
#include <dynamic_reconfigure/server.h>
#include <project/configsConfig.h>
#include<iostream>
#include <malloc.h>

float prev_angSpeed=0;
int n_iterations=0;
float start_angle=0;
float updated_angle=0;
double current_Time=0;
double previous_time=0;
float prev_x=0;
float prev_y=0;


using namespace message_filters;
using namespace std;

#define Baseline 1.3  /*m*/
#define front_rear_dist 1.765 /*m*/
#define steering_factore 18


void publish_odom(float x,float y, float angle){
  /*c'è da pubblicare il topic a riguardo. Per ora stampo solo le cose*/
  cout<<"x: "<<x;
  cout<<"  y: "<<y;
  cout<<"  angle: "<<angle <<"\n";
  /*float time = current_Time - previous_time;
  cout<<"time:"<<time;*/
}

void do_diff_drive_odometry(float speed,float angular_speed,int numb,float*start_angle,float*updated_angle,float interval){
   *start_angle = *updated_angle;
   *updated_angle = *start_angle + prev_angSpeed*interval;
   prev_angSpeed=angular_speed;
   float x,y;
   x = prev_x + speed *interval*cos(*start_angle);
   y= prev_y+ speed*interval*sin(*start_angle);
   prev_x=x;
   prev_y=y;
   publish_odom(x,y,*updated_angle);
}

void do_diff_drive_kinematics(float r_wheel_speed, float l_wheel_speed, int numb,float start_angle,float updated_angle,double interval){
  int speed, angular_speed;
  speed = ( r_wheel_speed +l_wheel_speed)/2;
  angular_speed = (r_wheel_speed -  l_wheel_speed) / Baseline;
  do_diff_drive_odometry(speed,angular_speed,numb,&start_angle,&updated_angle,interval);
}

double update_time(double *previous_time, double*current_Time){
   *previous_time =  *current_Time;
   *current_Time = (double) ros::Time::now().toSec();
   double interval = *current_Time - *previous_time;
   
   return interval;
}


void parameters(project::configsConfig &config, uint32_t level){
  ROS_INFO("odometry set is %s starting from: [%d - %d]", 
            config.diff_acker ? "differntial":"ackerman", 
            config.x,
            config.y);

            prev_x = config.x;
            prev_y = config.y
            
            ROS_INFO ("%d",level);

}


void callback(const project::floatStamped::ConstPtr& msg1, const project::floatStamped::ConstPtr& msg2,const project::floatStamped::ConstPtr &msg3)
{
  float speed_right = msg1->data;
  float speed_left = msg2->data;
  do_diff_drive_kinematics(speed_right , speed_left,n_iterations,start_angle,updated_angle,update_time(&previous_time,&current_Time));
  n_iterations++;
  //ROS_INFO ("Right wheel velocity:(%f), left wheel velocity:(%f), steering angle:(%f)", msg1->data, msg2->data,msg3->data);
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "odometry");
  dynamic_reconfigure::Server<project::configsConfig> server;
  dynamic_reconfigure::Server<project::configsConfig>::CallbackType f;

  f = boost::bind(&parameters, _1, _2);
  server.setCallback(f);

  ros::NodeHandle n;

  message_filters::Subscriber<project::floatStamped> sub1(n, "speedR_stamped", 1);
  message_filters::Subscriber<project::floatStamped> sub2(n, "speedL_stamped", 2);
  message_filters::Subscriber<project::floatStamped> sub3(n,"steer_stamped",3);

  //ros::Subscriber sub = n.subscribe("speed_L", 1000, chatterCallback);

  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped,> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<project::floatStamped, project::floatStamped,project::floatStamped> MySyncPolicy;
  
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2,sub3);
  //ROS_INFO ("Received two messages");
  sync.registerCallback(boost::bind(&callback, _1, _2,_3));


  //message_filters::TimeSynchronizer<project::floatStamped, project::floatStamped> sync(sub1, sub2,sub3 1000);
  //sync.registerCallback(boost::bind(&callback, _1, _2,_3));
  //ROS_INFO ("Received two messages");

  ros::spin();

  return 0;
}

