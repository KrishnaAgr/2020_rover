#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include<std_msgs/Float32MultiArray.h>
// #include <controls/con.h>
// #include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <cmath>
#include<nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/LaserScan.h>

// #include <iostream.h>

ros::Subscriber quat_subscriber;
// const Geodesic &geod = Geodesic::WGS84();
float initia = 0;
float flag = 0, flag1 = 0;
float p_angle = 0, cur_x = 0, cur_y = 0;
float des_x , des_y ;
std::vector <float> las;
char dir = 'L';


void des_cb(const geometry_msgs::Pose2D hh)
{

  des_x = hh.x;
  des_y = hh.y;

}
void MsgCallback(const nav_msgs::Odometry msg)
{
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  geometry_msgs::Quaternion q;
  q = msg.pose.pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  // the tf::Quaternion has a method to acess roll pitch and yaw
  p_angle = yaw;
  // rpy.y = pitch;
  // rpy.z = yaw;
  cur_x = msg.pose.pose.position.x;
  cur_y = msg.pose.pose.position.y;
  // this Vector is then published:
  // rpy_publisher.publish(rpy);
  // ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}
float len = 0;
void lasercb(const sensor_msgs::LaserScan msg)
{
  las = msg.ranges;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "val");
  ros::NodeHandle nh_;

  ros::Publisher rf = nh_.advertise<std_msgs::Float64>("/rover/corner_rf_wheel_rf_controller/command", 1);
  ros::Publisher lf = nh_.advertise<std_msgs::Float64>("/rover/corner_lf_wheel_lf_controller/command", 1);
  ros::Publisher rm = nh_.advertise<std_msgs::Float64>("/rover/bogie_right_wheel_rm_controller/command", 1);
  ros::Publisher lm = nh_.advertise<std_msgs::Float64>("/rover/bogie_left_wheel_lm_controller/command", 1);
  ros::Publisher rb = nh_.advertise<std_msgs::Float64>("/rover/corner_rb_wheel_rb_controller/command", 1);
  ros::Publisher lb = nh_.advertise<std_msgs::Float64>("/rover/corner_lb_wheel_lb_controller/command", 1);

  quat_subscriber = nh_.subscribe("rover/odom", 1000, MsgCallback);
  ros::Subscriber sub_dest = nh_.subscribe("/rover/cmd_pose", 10, des_cb);

  // des_x = -20.096200, des_y = 15.331000;
  ros::Publisher cew = nh_.advertise<geometry_msgs::Pose2D>("ce", 1);
  ros::Publisher currr = nh_.advertise<geometry_msgs::Pose2D>("posi", 1);

  ros :: Subscriber approval = nh_.subscribe("/rover/scan", 1, lasercb);
  ros :: Publisher appr = nh_.advertise<std_msgs::Float64>("onn", 1);
  // ros :: Publisher s1 = nh_.advertise<std_msgs::Float64>("stage1", 1);

  ros::Publisher lfc = nh_.advertise<std_msgs::Float64>("/rover/bogie_left_corner_lf_controller/command", 1);
  ros::Publisher lbc = nh_.advertise<std_msgs::Float64>("/rover/rocker_left_corner_lb_controller/command", 1);
  ros::Publisher rbc = nh_.advertise<std_msgs::Float64>("/rover/rocker_right_corner_rb_controller/command", 1);
  ros::Publisher rfc = nh_.advertise<std_msgs::Float64>("/rover/bogie_right_corner_rf_controller/command", 1);

  float vel = 10, s_angle = 0, sep = 0.3;


  ros::Rate loop_rate(10);


  while (ros::ok()) {

    ros::spinOnce();
    std_msgs::Float64 msg;

    float dist = sqrt((des_x - cur_x) * (des_x - cur_x) + (des_y - cur_y) * (des_y - cur_y));
    float anglew = atan( (des_y - cur_y) / (des_x - cur_x) );

    anglew = anglew * 180.0 / M_PI;
    p_angle = p_angle * 180.0 / (float)M_PI ;

    if (des_x - cur_x > 0 && anglew > 0)
    {
      anglew = -180 + anglew;
    }
    else
    {
      if (anglew < 0 && des_x - cur_x > 0)
        anglew = 180 + anglew;

    }

    geometry_msgs::Pose2D curr;
    geometry_msgs::Pose2D coor;

    coor.x=cur_x;
    coor.y=cur_y;
    coor.theta=des_x;
    cew.publish(coor);

    curr.x = anglew;
    curr.y = p_angle;
    curr.theta = p_angle + anglew - p_angle;
    currr.publish(curr);


    float datalef, datarig;
    // if (dir == ' ')
    // {
    //   int flag = 0;
    //   char e = ' ';
    //   for (int i = 180 - 15; i < 180 + 15; ++i)
    //   {
    //     if (las[i] < 2.0 && las[i]>0.01)
    //     {
    //       flag == 1;
    //       if (i < 180)
    //       {
    //         e = 'L';
    //       }
    //       else
    //         e = 'R';
    //     }

    //   }
    //   if (flag == 0)
    //   {
    //     datalef = 10;
    //     datarig = -10;
    //   }
    //   else
    //   {
    //     if (e == 'R')
    //     {
    //       datalef = -10 + sep * s_angle / 2;
    //       datarig = -10 - sep * s_angle / 2;
    //       dir = 'R';
    //     }
    //     else
    //     {
    //       datalef = 10 + sep * s_angle / 2;
    //       datarig = 10 - sep * s_angle / 2;
    //       dir = 'L';
    //     }

    //   }
    // }
    // else
    // {


    //   float diff = anglew - p_angle;
    //   if (diff > 75)
    //     diff = 75;
    //   if (diff < -75)
    //     diff = -75;
    //   int flag1 = 0;
    //   if (abs(diff) < 15)
    //   {
    //     dir == ' ';
    //     continue;
    //   }

    //   for (int i = 180 + diff - 15; i <= 180 + diff + 15; ++i)
    //   {
    //     if (las[i] < 2.0 && las[i]>0.01)
    //     {
    //       flag1 = 1;
    //     }
    //   }
    //   if (flag1 == 1)
    //   {
    //     int q = 0;
    //     for (int i = 180 - 15; i < 180 + 15; ++i)
    //     {
    //       /* code */if (las[i] < 2.0 && las[i]>0.01)
    //       {
    //         q = 1;
    //       }
    //     }
    //     if (q == 0) {
    //       datalef = 10;
    //       datarig = -10;
    //     }
    //     else
    //     {
    //       datalef = (diff / abs(diff)) * 10 + sep * s_angle / 2;
    //       datarig = (diff / abs(diff)) * 10 - sep * s_angle / 2;
    //       // continue;
    //     }
    //   }
    //   else
    //   {
    //     datalef = -1 * (diff / abs(diff)) * 10 + sep * s_angle / 2;
    //     datarig = -1 * (diff / abs(diff)) * 10 - sep * s_angle / 2;
    //   }



    // }

          datalef = -10 + sep * s_angle / 2;
          datarig = -10 - sep * s_angle / 2;
         





    if (dist < 3 && cur_y > 2)
    {
      datalef = 0;
      datarig = 0;
    }
    msg.data = datalef;
    lf.publish(msg);
    lb.publish(msg);
    lm.publish(msg);
    msg.data = datarig;
    rm.publish(msg);
    rf.publish(msg);
    rb.publish(msg);



    // }
    loop_rate.sleep();
  }
  return 0;
}
//