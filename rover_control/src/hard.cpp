#include <ros/ros.h>
#include <std_msgs/Float64.h>
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
// #include <iostream.h>

ros::Subscriber quat_subscriber;
int flag = 0;
float p_angle = 0, cur_x = 0, cur_y = 0;
float des_x = 10, des_y = 10;

void des_cb(const geometry_msgs::Pose hh)
{
  des_x=hh.position.x;
  des_y=hh.position.y;
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
  ros::Publisher pp = nh_.advertise<geometry_msgs::Pose2D>("anglel", 1);
  ros::Publisher currr = nh_.advertise<geometry_msgs::Pose2D>("posi", 1);
  ros::Subscriber sub_dest = nh.subscribe("/rover/cmd_pose",10,des_cb); 


  ros::Publisher lfc = nh_.advertise<std_msgs::Float64>("/rover/bogie_left_corner_lf_controller/command", 1);
  ros::Publisher lbc = nh_.advertise<std_msgs::Float64>("/rover/rocker_left_corner_lb_controller/command", 1);
  ros::Publisher rbc = nh_.advertise<std_msgs::Float64>("/rover/rocker_right_corner_rb_controller/command", 1);
  ros::Publisher rfc = nh_.advertise<std_msgs::Float64>("/rover/bogie_right_corner_rf_controller/command", 1);

  float vel = 20, s_angle = 0, sep = 0.3;
  
  // float angle=90 *  M_PI/180.0;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    std_msgs::Float64 msg;

    // msg.data=p_angle;
    // pp.publish(msg);
    float dist = sqrt((des_x - cur_x) * (des_x - cur_x) + (des_y - cur_y) * (des_y - cur_y));
    float anglew = atan( (des_y - cur_y) / (des_x - cur_x) ) * 180.0 / M_PI;
    // angle=angle*M_PI/180.0;
    // std::cout<<"angle: "<<angle<<endl;
    p_angle = p_angle * 180.0 / M_PI;
    if (p_angle < 0)
    {
      p_angle += 180;
    }
    else
    {
      p_angle -= 180;
    }

    if (des_x - cur_x > 0)
    {
      // if(anglew<0)
      //   anglew+=180;

    }
    else
    {
      if (anglew < 0)
        anglew = 180 + anglew;
      else
        anglew = -180 + anglew;
    }


    geometry_msgs::Pose2D poss;
    geometry_msgs::Pose2D curr;

    curr.x = cur_x;
    curr.y = p_angle;
    curr.theta = p_angle + anglew - p_angle;
    currr.publish(curr);


    poss.x = cur_x;
    poss.y = cur_y;
    poss.theta = p_angle;
    pp.publish(poss);
    // if (abs(anglew-p_angle)<5)
    //    {
    //      sep
    //    }
    float datalef, datarig;
    float diff =  anglew - p_angle;
    float forr = 0;
    if(abs(anglew)>165)
    {
      if(anglew<0)
      {
        if(flag==1)
        {
          if(p_angle>165)
            diff=8;
        }
      }
      else
      {
        if(flag==1)
        {
          if(p_angle<-165)
            diff=-8;
        }
      }
    }
    if (abs(diff) < 10 || dist < 1)
    {
      forr = 1;
      flag = 1;

    }
    else
    {
      if (flag == 1) {
        if (diff < 0) {
          datalef = 1 * vel + sep * 10 / 2;
          datarig = (1 * vel - sep * 10 / 2);
        }
        else
        {
          datalef = -1 * vel + sep * 10 / 2;
          datarig = -1 * vel - sep * 10 / 2;
        }
      }
      else
      {
        datalef = 1 * vel + sep * 10 / 2;
        datarig = (1 * vel - sep * 10 / 2);
      }

    }
    if (forr == 1)
    {
      if (dist < 0.3)
      {
        datarig = 0;
        datalef = 0;

      }
      else {
        datarig = -10;
        datalef = 10;
      }
    }
    // datalef = -1 * vel + sep * 10 / 2;
    //      datarig = -1 * vel - sep * 10 / 2;

    msg.data = datalef;
    lf.publish(msg);
    lb.publish(msg);
    lm.publish(msg);
    msg.data = datarig;
    rm.publish(msg);
    rf.publish(msg);
    rb.publish(msg);



    loop_rate.sleep();
  }
  return 0;
}
