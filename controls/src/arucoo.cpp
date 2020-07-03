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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
// #include <iostream>
// #include <ros/ros.h>
#include <std_msgs/Header.h>
// #include <sensor_msgs/Image.h
float pix_x = -1, pix_y = -1;
float flag = 0;
image_transport::Publisher image_pub_;
cv::Mat input;


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	input = cv_ptr->image;
}


void appc(const geometry_msgs::Pose2D hh)
{

	pix_x = hh.x;
	pix_y = hh.y;
}


void appcb(const std_msgs::Float64 s)
{
	flag = s.data;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "cenn");
	ros::NodeHandle nh_;
	// ros::Subscriber sub_dest = nh_.subscribe("/rover/cmd_pose", 10, des_cb);
	ros::Subscriber app = nh_.subscribe("/acuroo", 1, appc);
	ros::Subscriber image_sub = nh_.subscribe("/rover/camera/image_raw", 1, imageCb);
	ros::Publisher centt = nh_.advertise <geometry_msgs::Pose2D>("/rover/cmd_pose", 10);
	ros::Publisher appr = nh_.advertise<std_msgs::Float64>("/approva", 1);

	ros :: Publisher s3 = nh_.advertise<std_msgs::Float64>("stage3", 1);


	ros::Publisher rf = nh_.advertise<std_msgs::Float64>("/rover/corner_rf_wheel_rf_controller/command", 1);
	ros::Publisher lf = nh_.advertise<std_msgs::Float64>("/rover/corner_lf_wheel_lf_controller/command", 1);
	ros::Publisher rm = nh_.advertise<std_msgs::Float64>("/rover/bogie_right_wheel_rm_controller/command", 1);
	ros::Publisher lm = nh_.advertise<std_msgs::Float64>("/rover/bogie_left_wheel_lm_controller/command", 1);
	ros::Publisher rb = nh_.advertise<std_msgs::Float64>("/rover/corner_rb_wheel_rb_controller/command", 1);
	ros::Publisher lb = nh_.advertise<std_msgs::Float64>("/rover/corner_lb_wheel_lb_controller/command", 1);
	// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ros::spinOnce();

		if (pix_x == -1 || flag==0) {
			continue;
		}
		std_msgs::Float64 msg;
		msg.data = 1;
		s3.publish(msg);
		float cen_x = 180, cen_y = 120;
		float vel = 10, s_angle = 0, sep = 0.3;

		float datalef, datarig;
		float diff =  cen_x - pix_x;
		float forr = 0;

		if (abs(diff) < 10 )
		{
			forr = 1;
		}
		else
		{

			if (diff < 0) {
				datalef = 1 * vel + sep * 10 / 2;
				datarig = (1 * vel - sep * 10 / 2);
			}
			else
			{
				datalef = -1 * vel + sep * 10 / 2;
				datarig = -1 * vel - sep * 10 / 2;
			}

			// else
			// {
			// 	datalef = 1 * vel + sep * 10 / 2;
			// 	datarig = (1 * vel - sep * 10 / 2);
			// }

		}

		if (forr == 1)
		{
			// if (dist < 3)
			// {
			// 	datarig = 0;
			// 	datalef = 0;

			// }

			datarig = -20;
			datalef = 20;
			// cv::Mat inputImage;

			// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			// std::vector<int> ids;
			// std::vector<std::vector<cv::Point2f> > corners;
			// cv::aruco::detectMarkers(input, dictionary, corners, ids);
			// if (ids.size() > 1)
			// {
			// 	float cenxx = ids[0];
			// 	float cenyy = ids[1];
			// 	geometry_msgs::Pose2D h;
			// 	h.x = cenxx;
			// 	h.y = cenyy;
			// 	std_msgs::Float64 ms;
			// 	ms.data = 0;

			// 	appr.publish(ms);

			// 	centt.publish(h);
			// }
		}
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f> > corners;
			cv::aruco::detectMarkers(input, dictionary, corners, ids);
			if (ids.size() > 1)
			{
				float cenxx = ids[0];
				float cenyy = ids[1];
				geometry_msgs::Pose2D h;
				h.x = cenxx;
				h.y = cenyy;
				std_msgs::Float64 ms;
				ms.data = 0;

				appr.publish(ms);

				centt.publish(h);
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



	}

	loop_rate.sleep();

	return 0;
}

