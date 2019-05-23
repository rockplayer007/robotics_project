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

#define Baseline 1.3  /*m*/
#define front_rear_dist 1.765 /*m*/
#define steering_factore 18


using namespace message_filters;
using namespace std;

class Odometer{
       
public:
       ros::NodeHandle n;
       message_filters::Subscriber<project::floatStamped> sub1;
       message_filters::Subscriber<project::floatStamped> sub2;
       message_filters::Subscriber<project::floatStamped> sub3;
       ros::Timer timer1;
       float prev_angSpeed=0; 
       int n_iterations=0;
       float start_angle=0;
       float updated_angle=0;
       double current_Time=0;
       double previous_time=0;
       float  prev_x;
       float  prev_y;
       bool choice;
       Odometer(){
               prev_angSpeed=0; 
               n_iterations=0;
               start_angle=0;
               updated_angle=0;
               current_Time=0;
               previous_time=0;
               prev_x;
               prev_y;
               sub1.subscribe(this->n,"speedR_stamped",1);
               sub2.subscribe(this->n,"speedL_stamped",1);
               sub3.subscribe(this->n,"steer_stamped",1);
       }
      
       float radiantToDegree(float radiantAngle){
         float result;
         result = ((radiantAngle*180)/3.14);
         return result;
       }

       void publish_odom(float x,float y, float angle){
           /*c'è da pubblicare il topic a riguardo. Per ora stampo solo le cose*/
            cout<<"x: "<<x;
            cout<<"  y: "<<y;
            cout<<"  angle: "<<radiantToDegree(angle) <<"\n";
            /*float time = current_Time - previous_time;
            cout<<"time:"<<time;*/
       }

       void do_diff_drive_odometry(float speed,float angular_speed,int numb,float start_angle,float updated_angle,float interval){
                    //(start_angle) = (updated_angle);
                    /*cout<<"start_angle:"<<*start_angle;*/
                    //cout<<" update_angle:"<<Odometer::updated_angle<<"\n";
                    Odometer::updated_angle = Odometer::start_angle +Odometer::prev_angSpeed*interval;
                    //cout<<"updated angle:"<<updated_angle;
                    /*cout<<"updated angle:"<<*updated_angle;
                    cout<<"start_angle"<<*start_angle<<"\n";*/
                    Odometer::prev_angSpeed=angular_speed;
                    float x,y;
                    x = prev_x + speed *interval*cos(start_angle);
                    y= prev_y+ speed*interval*sin(start_angle);
                    Odometer::prev_x=x;
                    Odometer::prev_y=y;
                    //cout<<" updated angle:"<<Odometer::updated_angle<<"\n";
                    Odometer::start_angle = Odometer::updated_angle;

                    publish_odom(x,y,updated_angle);
       }
       
       float degToRad(float degAngle){
         float result = (degAngle*3,14)/180;
         return result;
       }

       void do_ackermann_kinematics(float r_wheel_speed, float l_wheel_speed,float steer, int numb,float update_angle,float start_angle,double interval){
         float speedF  = (r_wheel_speed+l_wheel_speed)/2;
         float real_steer = degToRad(steer / steering_factore);
         float angular_speed = (speedF* sin(real_steer)) / front_rear_dist;
         float speed = (front_rear_dist / tan(real_steer))* angular_speed;
         do_diff_drive_odometry(speed,angular_speed,numb,start_angle,update_angle,interval);
       }

       void do_diff_drive_kinematics(float r_wheel_speed, float l_wheel_speed, int numb,float start_angle,float updated_angle,double interval){
                    float speed, angular_speed;
                    speed = ( r_wheel_speed +l_wheel_speed)/2;
                    //cout<<"speed:"<<speed<<"\n";
                    angular_speed = (r_wheel_speed -  l_wheel_speed) / Baseline;
                    //float diff (r_wheel_speed-l_wheel_speed);
                  // cout<<"ang speed: "<<angular_speed<<"\n";
                    //cout<<"baseline:"<<Baseline<<"\n";
                    /*cout<<"start_angle:"<<start_angle;
                    cout<<" update_angle:"<<updated_angle<<"\n";*/
                    do_diff_drive_odometry(speed,angular_speed,numb,start_angle,updated_angle,interval);
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
          ROS_INFO ("Received two messages");
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
                              //cout<<"x:"<<prev_x<< "y:"<<prev_y<<"\n";

                              ROS_INFO ("%d",level);
                              }
      void callback(const project::floatStamped::ConstPtr& msg1, const project::floatStamped::ConstPtr& msg2,const project::floatStamped::ConstPtr &msg3,Odometer*od)
        {
          float speed_right = msg1->data;
          float speed_left = msg2->data;
          float steer = msg3->data;
          //cout<<"speed_right:"<<speed_right;
          //cout<<"speed_left:"<<speed_left<<"\n";
          if(od->choice)
             od->do_diff_drive_kinematics(speed_right , speed_left,od->n_iterations,od->start_angle,od->updated_angle,od->update_time(&(od->previous_time),&(od->current_Time)));
          else
             od->do_ackermann_kinematics(speed_right,speed_left,steer, od->n_iterations,od->updated_angle,od->start_angle,od->update_time(&(od->previous_time),&(od->current_Time)));
          od->n_iterations++;
          //ROS_INFO ("Right wheel velocity:(%f), left wheel velocity:(%f), steering angle:(%f)", msg1->data, msg2->data,msg3->data);
        }

     //da finire il passaggio dell'indirizzo del metodo "callback" alla funzione set()

      int main(int argc, char** argv){
            ros::init(argc, argv, "odometry");
            Odometer odometerObj;
            dynamic_reconfigure::Server<project::configsConfig> server;
            dynamic_reconfigure::Server<project::configsConfig>::CallbackType f;
            f = boost::bind(&parameters, _1, _2,&odometerObj.prev_x,&odometerObj.prev_y,&odometerObj.choice);
            server.setCallback(f);
            odometerObj.set(callback);
            /*message_filters::TimeSynchronizer<project::floatStamped, project::floatStamped> sync(odometerObj.sub1, odometerObj.sub2,odometerObj.sub3, 1000);
            sync.registerCallback(boost::bind(&(callback), _1, _2,_3,odometerObj));
            ROS_INFO ("Received two messages");*/
            return 0;
          }    
