/**
* \file ros_cv.cpp
*
* \brief ROS node to track the center of mass of a particular color.
* 
* ColorTracker is a C++ implementation of tracking code presented at:
* http://www.aishack.in/2010/07/tracking-colored-objects-in-opencv/
* 
* ROS code derived from:
* http://dasl.mem.drexel.edu/wiki/index.php/How_to_make_ROS_and_Opencv_work_together
*
* \author Andrew Price
*/

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>

#include "ColorTracker.h"

class SimpleROSTracker
{
	ros::NodeHandle nh_;
	ros::NodeHandle n;
	ros::Publisher pub ;
	image_transport::ImageTransport it_;    
	image_transport::Subscriber image_sub_; //image subscriber 
	image_transport::Publisher image_pub_; //image publisher
	std_msgs::String msg;
public:

	SimpleROSTracker()
		: it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/color/image", 1, &SimpleROSTracker::imageCb, this);
		image_pub_= it_.advertise("/camera/tracked",1);
		pub = nh_.advertise<geometry_msgs::Point>("/CoM",1);
	}

	~SimpleROSTracker()
	{
		//cv::destroyWindow(WINDOW);
	}

	ColorTracker tracker;

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(msg, "bgr8");
		
		cv::Point CoM = tracker.getCoM(imgPtr->image);

		cv::circle(imgPtr->image, CoM, 10, CV_RGB(255,0,0));

		image_pub_.publish(imgPtr->toImageMsg());

		//pub.publish(geometry_msgs::Point(CoM.x, CoM.y, 0));
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_tracker");

	SimpleROSTracker st;

	ros::spin();

	return 0;
}
