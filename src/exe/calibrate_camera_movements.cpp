/**
 * calibrate_camera_movements.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: arprice
 */

#include <ros/ros.h>

#include "hubo_vision/SetHuboArmPose.h"
#include "hubo_vision/tf_eigen.h"
#include "Collision_Checker.h"


const int NUM_CALIBRATION_POSES = 15;
const double X_MIN =  0.1;
const double X_MAX =  0.3;
const double Y_MIN =  0.25;
const double Y_MAX = -0.25;
const double Z_MIN = -0.25;
const double Z_MAX =  0.1;

double randbetween(double min, double max)
{
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}

int main(int argc, char** argv)
{
	ROS_INFO("Tracker Started.");
	ros::init(argc, argv, "simple_tracker");

	ros::NodeHandle nh;
	ros::ServiceClient poseClient = nh.serviceClient<hubo_vision::SetHuboArmPose>("/hubo/set_arm");

	ros::Rate loop_rate(0.1);

	while (ros::ok())
	{
		/*double x = randbetween(X_MIN, X_MAX);
		double y = randbetween(Y_MIN, Y_MAX);
		double z = randbetween(Z_MIN, Z_MAX);*/

		Eigen::Isometry3d ePose;
		tf::StampedTransform tPose;
		hubo_vision::SetHuboArmPose srv;

		ePose.matrix() <<   1,  0,  0, .3,
							0,  1,  0, .2,
							0,  0,  1,  0,
							0,  0,  0,  1;

		ePose.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
		ePose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
/*
		Collision_Checker cc;
		cc.initCollisionChecker();
		cc.checkSelfCollision(ePose);
		tf::TransformEigenToTF(ePose, tPose);

		tf::poseTFToMsg(tPose, srv.request.Target);
		srv.request.ArmIndex = 1;
		poseClient.call(srv);
*/
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}

