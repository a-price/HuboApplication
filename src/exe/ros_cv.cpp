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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant header files
#include <opencv2/highgui/highgui.hpp>

// PCL includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "HuboApplication/ColorTracker.h"
#include "HuboApplication/tf_eigen.h"

#include "HuboApplication/SetHuboArmPose.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> KinectSyncPolicy;
//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> KinectSyncPolicy;

void colorCallbackTest(const sensor_msgs::ImageConstPtr color)
{
	ROS_INFO("Got a Color Image.");
}
void depthCallbackTest(const sensor_msgs::ImageConstPtr depth)
{
	ROS_INFO("Got a Depth Image.");
}
void cloudCallbackTest(const sensor_msgs::PointCloud2ConstPtr points)
{
	ROS_INFO("Got a Point Cloud.");
}

class SimpleKinectTracker
{
public:
	SimpleKinectTracker()
		: visual_sub_ (nh_, "/camera/rgb/image_rect_color", 8),
		  depth_sub_ (nh_, "/camera/depth_registered/image_rect", 8),
		  cloud_sub_ (nh_, "/camera/depth/points", 8),
		  sync_(KinectSyncPolicy(8), visual_sub_, depth_sub_, cloud_sub_)
		  //sync_(KinectSyncPolicy(1), visual_sub_, depth_sub_)
	{
		ROS_INFO("Initialized.");
		visual_sub_.registerCallback(boost::bind(&colorCallbackTest, _1));
		depth_sub_.registerCallback(boost::bind(&depthCallbackTest, _1));
		cloud_sub_.registerCallback(boost::bind(&cloudCallbackTest, _1));
		sync_.registerCallback(boost::bind(&SimpleKinectTracker::kinectCallback, this, _1, _2, _3));

		pose_client_ = nh_.serviceClient<HuboApplication::SetHuboArmPose>("/hubo/set_arm");
		//sync_.registerCallback(boost::bind(&SimpleKinectTracker::kinectCallback, this, _1, _2));
	}

	void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth, const sensor_msgs::PointCloud2ConstPtr points)
	//void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth)
	{
		ROS_WARN("Got sync'd frames.\n");
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(color, "bgr8");
		
		cv::Point CoM = tracker.getCoM(imgPtr->image);
		if (CoM.x == -1) {return;} // No color found

		pcl::PointCloud<pcl::PointXYZ> pCloud; // no color in this topic?
		pcl::fromROSMsg(*points, pCloud);
		ROS_INFO("Cloud Size: %i x %i", pCloud.height, pCloud.width);
		pcl::PointXYZ target = pCloud.points[CoM.x + CoM.y * pCloud.width];

		// Get body to camera tf
		ROS_INFO("Target Point: ( %i , %i )", CoM.x, CoM.y);
		ROS_INFO("Target Point: <%f,%f,%f>\n", target.x, target.y, target.z);

		Eigen::Vector3f eTarget = target.getVector3fMap();
		ROS_INFO("Eigen Point: <%f,%f,%f>\n", eTarget.x(), eTarget.y(), eTarget.z());

		// get object pose in camera frame
		Eigen::Isometry3d eHeadObject = Eigen::Isometry3d::Identity();
		eHeadObject.translate(eTarget.cast<double>());

		tf::StampedTransform tHeadObject;
		tf::TransformEigenToTF(eHeadObject, tHeadObject);
		tf_broad_.sendTransform(tf::StampedTransform(tHeadObject, ros::Time::now(), "/camera_depth_optical_frame", "/target_object"));


		// Get pose in body frame, grab it. eventually in a different node
		tf::StampedTransform tTorsoObject, tTorsoHead;
		Eigen::Isometry3d eTorsoObject, eTorsoHead;
		HuboApplication::SetHuboArmPose srv;
		try
		{
			//listener_.lookupTransform("/Body_Torso", "/target_object", ros::Time(0), tTorsoObject);
			listener_.lookupTransform("/Body_Torso", "/camera_depth_optical_frame", ros::Time(0), tTorsoHead);
			tf::TransformTFToEigen(tTorsoHead, eTorsoHead);
			tf::TransformTFToEigen(tHeadObject, eHeadObject);

			eTorsoObject = eTorsoHead * eHeadObject;
			eTorsoObject.matrix().topLeftCorner<3,3>() = Eigen::Matrix3d::Identity();
			//eTorsoObject.translate(Eigen::Vector3d(0,0.4,0)); // for testing...
			eTorsoObject.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
			eTorsoObject.rotate(Eigen::AngleAxisd(-M_PI/6, Eigen::Vector3d::UnitY()));

			tf::TransformEigenToTF(eTorsoObject, tTorsoObject);

			tf::poseTFToMsg(tTorsoObject, srv.request.Target);
			srv.request.ArmIndex = 0;
			pose_client_.call(srv);
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}

		/*
		// convert to body frame
		tf::StampedTransform tHeadTorso, tTorsoObject;
		Eigen::Isometry3d eHeadTorso, ePose;
		listener_.lookupTransform("/camera_link", "/Body_Torso", ros::Time(0), tHeadTorso);
		tf::TransformTFToEigen(tHeadTorso, eHeadTorso);
		std::cerr << eHeadTorso.matrix() << std::endl;

		eTarget = eHeadTorso.cast<float>() * eTarget;
		ROS_INFO("Transformed Point: <%f,%f,%f>\n", eTarget.x(), eTarget.y(), eTarget.z());
		ePose = Eigen::Isometry3d::Identity();
		ePose.translate(eTarget.cast<double>());

		HuboApplication::SetHuboArmPose srv;
        tf::poseEigenToMsg(ePose, srv.request.Target);
		srv.request.ArmIndex = 0;
		pose_client_.call(srv);
		*/

	}

private:
	ros::NodeHandle nh_;
	message_filters::Subscriber<sensor_msgs::Image> visual_sub_ ;
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
	message_filters::Synchronizer<KinectSyncPolicy> sync_;
	tf::TransformListener listener_;
	tf::TransformBroadcaster tf_broad_;
	ros::ServiceClient pose_client_;

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
		ROS_ERROR("Callback.");
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
	ROS_INFO("Tracker Started.");
	ros::init(argc, argv, "simple_tracker");

	//SimpleROSTracker st;
	SimpleKinectTracker skt;

	ros::spin();

	return 0;
}
