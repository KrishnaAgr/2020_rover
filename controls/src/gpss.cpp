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
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix>
double lat_des, lon_des, lon_cur, lat_cur, curx, cury;

void deslat(sensor_msgs::NavSatFix msg)
{
	lat_des = msg.latitude;
	lon_des = msg.longitude;

}
void curlat(sensor_msgs::NavSatFix msg)
{
	lat_cur = msg.latitude;
	lon_cur = msg.longitude;

}
void current(const nav_msgs::Odometry msg)
{
	curx = msg.pose.pose.position.x;
	cury = msg.pose.pose.position.y;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "gpss");
	ros::NodeHandle nh_;

	ros::Subscriber co = nh_.subscribe("rover/cur_lat", 1000, curlat);
	ros::Subscriber doo = nh_.subscribe("rover/des_lat", 1000, deslat);
	ros::Subscriber po = nh_.subscribe("rover/odom", 1000, current);
	ros::Publisher pub = nh_.advertise<geometry_msgs::Pose>("/rover/cmd_pose", 10);

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ros::spinOnce();
		int zone;
		bool northp;
		float x1, y1, x2, y2;
		float yfin, xfin;
		UTMUPS::Forward(lat_des, lon_des, zone, northp, x1, y1);
		UTMUPS::Forward(lat_cur, lon_cur, zone, northp, x2, y2);
		yfin = y1 - y2 + cury;
		xfin = x1 - x2 + curx;
		geometry_msgs::Pose hh;
		hh.position.x = xfin;
		hh.position.y = yfin;
		loop_rate.sleep();


	}
	return 0;
}
