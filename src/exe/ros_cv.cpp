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

#include <stdio.h>
#include <iostream>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant header files
#include <opencv2/highgui/highgui.hpp>

// PCL includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "HuboApplication/ColorTracker.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> KinectSyncPolicy;

class SimpleKinectTracker
{
public:
	SimpleKinectTracker()
		: visual_sub_ (nh_, "/camera/rgb/image_rect_color", 1),
		  depth_sub_ (nh_, "/camera/depth_registered/image_rect", 1),
		  cloud_sub_ (nh_, "/camera/depth/points", 1),
		  sync_(KinectSyncPolicy(1), visual_sub_, depth_sub_, cloud_sub_)
	{
		ROS_INFO("Initialized.");
		sync_.registerCallback(boost::bind(&SimpleKinectTracker::kinectCallback, this, _1, _2, _3));
	}

	void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth, const sensor_msgs::PointCloud2ConstPtr points)
	{
		ROS_INFO("Got sync'd frames.");
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(color, "bgr8");
		
		cv::Point CoM = tracker.getCoM(imgPtr->image);

		pcl::PointCloud<pcl::PointXYZRGB> pCloud;
		pcl::fromROSMsg(*points, pCloud);
		ROS_INFO("Cloud Size: %i x %i\n", CoM.x, CoM.y);
		pcl::PointXYZRGB target = pCloud.points[CoM.x + CoM.y * pCloud.width];

		// Get body to camera tf
		ROS_INFO("Target Point: <%f,%f,%f>\n", target.x, target.y, target.z);
	}
private:
	ros::NodeHandle nh_;
	message_filters::Subscriber<sensor_msgs::Image> visual_sub_ ;
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
	message_filters::Synchronizer<KinectSyncPolicy> sync_;

	ColorTracker tracker;
};

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
		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &SimpleROSTracker::imageCb, this);
		image_pub_= it_.advertise("/camera/tracked",1);
		pub = nh_.advertise<geometry_msgs::Point>("/CoM",1);
		ROS_INFO("Initialized.");
	}

	~SimpleROSTracker()
	{
		//cv::destroyWindow(WINDOW);
	}

	ColorTracker tracker;

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		ROS_INFO("Callback.");
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(msg, "bgr8");
		
		cv::Point CoM = tracker.getCoM(imgPtr->image);

		cv::circle(imgPtr->image, CoM, 10, CV_RGB(255,0,0));

		image_pub_.publish(imgPtr->toImageMsg());

		geometry_msgs::Point gcom;
		gcom.x = CoM.x;
		gcom.y = CoM.y;
		gcom.z = 0;
		pub.publish(gcom);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_tracker");

	SimpleROSTracker st;
	//SimpleKinectTracker skt;

	ros::spin();

	return 0;
}
