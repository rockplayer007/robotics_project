#include "ros/ros.h"
#include "project/floatStamped.h"
#include "project/custom_msg.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <project/configsConfig.h>
#include<iostream>
#include <cmath> 
#include <malloc.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define Baseline 1.3  /*unit of measure: meter */
#define front_rear_dist 1.765 /*unit of measure: meter  */
#define steering_factore 18

using namespace message_filters;
using namespace std;

class Odometer{
       
public:
       ros::NodeHandle n;
       //for publishing 
       ros::Publisher odom_std = n.advertise<nav_msgs::Odometry>("odom_std", 50);
       ros::Publisher odom_custom = n.advertise<project::custom_msg>("odom_custom", 50);
       project::custom_msg myMessage;


       tf::TransformBroadcaster odom_broadcaster;

       message_filters::Subscriber<project::floatStamped> sub1;
       message_filters::Subscriber<project::floatStamped> sub2;
       message_filters::Subscriber<project::floatStamped> sub3;
       float prev_angSpeed;
       float start_angle;
       float updated_angle;
       double current_Time;
       double previous_time;
       float  prev_x;
       float  prev_y;
       bool choice;

       Odometer(){
         /* we're assuming that when we start the bag the vehicole is stationary (and so the initial speed is zero). 
         The initial angle is consider as zero radiant. The initial position is reconfigurable at any time the user want
         to do it.
          */
               prev_angSpeed=0; 
               start_angle=0;
               updated_angle=0;
               current_Time=0;
               previous_time=0;
               sub1.subscribe(this->n,"speedR_stamped",1);
               sub2.subscribe(this->n,"speedL_stamped",1);
               sub3.subscribe(this->n,"steer_stamped",1);
               }

       float filterDegAngle(float degAngle){
        if(degAngle>360)
           degAngle= degAngle-360;
        return degAngle;
      }

      float filterRadAngle(float radAngle){
        if(radAngle>2*M_PI)
          radAngle=radAngle-(2*M_PI);
        return radAngle;
      }
      
       float radiantToDegree(float radiantAngle){
         float result;
         result = ((radiantAngle*180)/M_PI);
         return result;
       }

       string typeSource(){
         if(choice)
           return "differential drive";
         return "ackerman steering";
       }

       void publish_odom(float x,float y, float angle){
            cout<<"x: "<<x;
            cout<<"  y: "<<y;
            cout<<"  angle degree: "<<radiantToDegree(angle);
            cout<<"  angle radiants: "<<angle<< "\n";
           
            
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = ros::Time::now(); //change
            odom_trans.header.frame_id = "car_odometry";
            odom_trans.child_frame_id = "car";
            
            odom_trans.transform.translation.x = x; // setting x and y
            odom_trans.transform.translation.y = y; 
            odom_trans.transform.translation.z = 0.0; //needs to be 0.0
            odom_trans.transform.rotation = odom_quat;


            Odometer::odom_broadcaster.sendTransform(odom_trans);
            
            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now(); //change
            odom.header.frame_id = "car_odometry";

            //set the position
            odom.pose.pose.position.x = x; //setting x and y
            odom.pose.pose.position.y = y; 
            odom.pose.pose.position.z = 0.0; //needs to be 0.0
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            
            odom.child_frame_id = "car";
            odom.twist.twist.linear.x = 0; //decide if to change
            odom.twist.twist.linear.y = 0;//decide if to change
            odom.twist.twist.angular.z = 0.0;//decide if to change
            

            //publish the message
            Odometer::odom_std.publish(odom);


            myMessage.x = x;
            myMessage.y = y;
            myMessage.theta = radiantToDegree(angle) ;
            myMessage.source = typeSource();

            Odometer::odom_custom.publish(myMessage);


       }

       void do_diff_drive_odometry(float speed,float angular_speed,float start_angle,float updated_angle,float interval){
                    Odometer::updated_angle = Odometer::start_angle +Odometer::prev_angSpeed*interval;
                    Odometer::updated_angle=filterRadAngle(Odometer::updated_angle);
                    Odometer::prev_angSpeed=angular_speed;
                    float x,y;
                    x = prev_x + speed *interval*cos(start_angle);
                    y= prev_y+ speed*interval*sin(start_angle);
                    Odometer::prev_x=x;
                    Odometer::prev_y=y;
                    Odometer::start_angle = Odometer::updated_angle;

                    publish_odom(x,y,updated_angle);
       }
       
       float degToRad(float degAngle){
         float result = (degAngle*M_PI)/180;
         return result;
       }

       void do_ackermann_kinematics(float r_wheel_speed, float l_wheel_speed,float steer,float update_angle,float start_angle,double interval){
         float speedF  = (r_wheel_speed+l_wheel_speed)/2;
         float real_steer = degToRad(steer / steering_factore);
         float angular_speed = (speedF* sin(real_steer)) / front_rear_dist;
         float speed = (front_rear_dist / tan(real_steer))* angular_speed;
         do_diff_drive_odometry(speed,angular_speed,start_angle,update_angle,interval);
       }

       void do_diff_drive_kinematics(float r_wheel_speed, float l_wheel_speed,float start_angle,float updated_angle,double interval){
                    float speed, angular_speed;
                    speed = ( r_wheel_speed +l_wheel_speed)/2;
                    angular_speed = (r_wheel_speed -  l_wheel_speed) / Baseline;
                    do_diff_drive_odometry(speed,angular_speed,start_angle,updated_angle,interval);
                  }

        double update_time(double *previous_time, double*current_Time){
                    *previous_time =  *current_Time;
                    *current_Time = (double) ros::Time::now().toSec();
                    double interval = *current_Time - *previous_time;
                    return interval;
                  }

       

        void set(void (*f)  (const project::floatStamped::ConstPtr&, const project::floatStamped::ConstPtr&, const project::floatStamped::ConstPtr&, Odometer*)) {
          typedef message_filters::sync_policies::ApproximateTime<project::floatStamped, project::floatStamped,project::floatStamped> MySyncPolicy;
          message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2,sub3);
          //ROS_INFO ("Received two messages");
          sync.registerCallback(boost::bind(f, _1, _2,_3,this));
          ros::spin();
        }

      };

      void parameters(project::configsConfig &config, uint32_t level,float* prev_x, float *prev_y,bool*choice){
                    ROS_INFO("odometry set is %s starting from: [%d - %d]", 
                              config.diff_acker ? "differntial":"ackerman", 
                              config.x,
                              config.y);

                              *prev_x = config.x;
                              *prev_y = config.y;
                              *choice= config.diff_acker;
                               }

      
      void callback(const project::floatStamped::ConstPtr& msg1, const project::floatStamped::ConstPtr& msg2,const project::floatStamped::ConstPtr &msg3,Odometer*od)
        {
          float speed_right = msg1->data;
          
          float speed_left = msg2->data;

          float steer = msg3->data;

          if(od->choice)
             od->do_diff_drive_kinematics(speed_right , speed_left,od->start_angle,od->updated_angle,od->update_time(&(od->previous_time),&(od->current_Time)));
          else
             od->do_ackermann_kinematics(speed_right,speed_left,steer,od->updated_angle,od->start_angle,od->update_time(&(od->previous_time),&(od->current_Time)));

        }


      int main(int argc, char** argv){
            ros::init(argc, argv, "odometry");
            Odometer odometerObj;
            dynamic_reconfigure::Server<project::configsConfig> server;
            dynamic_reconfigure::Server<project::configsConfig>::CallbackType f;
            f = boost::bind(&parameters, _1, _2,&odometerObj.prev_x,&odometerObj.prev_y,&odometerObj.choice);
            server.setCallback(f);
            odometerObj.set(callback);
            return 0;

          }    
