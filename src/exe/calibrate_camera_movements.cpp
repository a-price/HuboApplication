/**
 * calibrate_camera_movements.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: arprice
 */

#include <ros/ros.h>

#include "HuboApplication/SetHuboArmPose.h"
#include "HuboApplication/tf_eigen.h"

const int NUM_CALIBRATION_POSES = 15;
const int X_MIN =  0.1;
const int X_MAX =  0.3;
const int Y_MIN =  0.25;
const int Y_MAX = -0.25;
const int Z_MIN = -0.25;
const int Z_MAX =  0.1;

double randbetween(double min, double max)
{
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}

int main(int argc, char** argv)
{
	ROS_INFO("Tracker Started.");
	ros::init(argc, argv, "simple_tracker");

	ros::NodeHandle nh;
	ros::ServiceClient poseClient = nh.serviceClient<HuboApplication::SetHuboArmPose>("/hubo/set_arm");

	ros::Rate loop_rate(0.1);

	while (ros::ok())
	{
		double x = randbetween(X_MIN, X_MAX);
		double y = randbetween(Y_MIN, Y_MAX);
		double z = randbetween(Z_MIN, Z_MAX);

		Eigen::Isometry3d ePose;
		tf::StampedTransform tPose;
		HuboApplication::SetHuboArmPose srv;

		ePose.matrix() <<   1,  0,  0,  x,
							0,  1,  0,  y,
							0,  0,  1,  z,
							0,  0,  0,  1;

		ePose.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
		ePose.rotate(Eigen::AngleAxisd(-M_PI/6, Eigen::Vector3d::UnitY()));

		tf::TransformEigenToTF(ePose, tPose);

		tf::poseTFToMsg(tPose, srv.request.Target);
		srv.request.ArmIndex = 0;
		poseClient.call(srv);

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}

