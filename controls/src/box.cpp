#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// #include <iostream>
// #include <ros/ros.h>
#include <std_msgs/Header.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opena/Centre.h>
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
// #include <iostream>
// #include <ros/ros.h>
#include <std_msgs/Header.h>

static const std::string OPENCV_WINDOW = "Image window";


// image_transport::ImageTransport it_;
// image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
cv::Mat input;
float flag1 = 0;

// public: ImageConverter(): it_(nh_)
//     {
//       // Subscrive to input video feed and publish output video feed
//       image_sub_ = it_.subscribe("/magnus/camera/image_raw_mouse_left", 1, &ImageConverter::imageCb, this);
//     }
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
void appcb(const std_msgs::Float64 msg)
{
    flag1 = msg.data;
}
// open cv shit

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image");
    // ImageConverter ic;
    ros::NodeHandle nh_;
    ros::Subscriber image_sub = nh_.subscribe("/rover/camera/image_raw", 1, imageCb);
    ros::Subscriber approval = nh_.subscribe("onn", 1, appcb);
    ros::Publisher centre_pub = nh_.advertise<geometry_msgs::Pose2D>("acuroo", 1);

    ros :: Publisher s2 = nh_.advertise<std_msgs::Float64>("stage2", 1);

    ros::Rate loop_rate(10);
    while (ros::ok()) {

        ros::spinOnce();

        if (input.empty()) {
            std_msgs::Float64 ms;
        ms.data = 1;
        s2.publish(ms);
            continue;
        }
        if (flag1 != 1.0) {
            continue;
        }
        cv::Mat mask ;
        cv::inRange(input, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), mask);

        // cv::imshow("gray", mask);

        // cv::imshow("mask", mask);

        // extract contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        double XX, YY;
        
        if (contours.size()) {
            for (int i = 0; i < contours.size(); ++i)
            {
                // fit bounding rectangle around contour
                cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

                // read points and angle
                cv::Point2f rect_points[4];
                rotatedRect.points( rect_points );

                float  angle = rotatedRect.angle; // angle

                // read center of rotated rect
                cv::Point2f center = rotatedRect.center; // center
                XX = center.x;
                YY = center.y;
                // // draw rotated rect
                // for(unsigned int j=0; j<4; ++j)
                //     cv::line(input, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0));
                //
                // // draw center and print text
                // std::stringstream ss;   ss << angle; // convert float to string
                // cv::circle(input, center, 10, cv::Scalar(255,255,255)); // draw center
                // cout<<rotatedRect.center;
                // cv::putText(input, ss.str(), center + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); // print angle
            }

            // pubb.publish();




            geometry_msgs::Pose2D centre_msg;
            // centre_msg.header.stamp = ros::Time::now();
            centre_msg.x = XX;
            centre_msg.y = YY;
            centre_pub.publish(centre_msg);

            // ros::spin();
        }
        // else
        // {
        //     geometry_msgs::Pose2D centre_msg;
        //     // centre_msg.header.stamp = ros::Time::now();
        //     centre_msg.x = 200;
        //     centre_msg.y = 0;
        //     centre_pub.publish(centre_msg);

        // }

        loop_rate.sleep();


    }
    return 0;
}