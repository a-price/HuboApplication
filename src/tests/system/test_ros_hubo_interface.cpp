/**
* \file test_ros_hubo_interface.cpp
*
* \brief ROS node to send dummy Hubo joint commands.
* Potentially more functionality to follow
*
* \author Andrew Price
*/

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "tf_eigen.h"


class ROSHuboTester
{
public:
	ROSHuboTester()
	{
		m_JointPublisher = nh_.advertise<sensor_msgs::JointState>("/hubo/target_joints", 1);
	}

	void publishSample(int seed)
	{
		//Eigen::Matrix< double, 6, 1 > cmdJoints;
		sensor_msgs::JointState joints;
		joints.position.push_back((cos(seed/50.0)+1.0)/3.0);
		m_JointPublisher.publish(joints);
	}

	void publishElbow()
	{
		//Eigen::Matrix< double, 6, 1 > cmdJoints;
		tf::StampedTransform tS,tE,tW;
		listener.lookupTransform("/openni_depth_frame","/right_shoulder_1",ros::Time(0),tS);
		listener.lookupTransform("/openni_depth_frame","/right_elbow_1",ros::Time(0),tE);
		listener.lookupTransform("/openni_depth_frame","/right_hand_1",ros::Time(0),tW);

		sensor_msgs::JointState joints;
		double angle = getAngle(tS,tE,tW);
		joints.position.push_back(angle);
		//m_JointPublisher.publish(joints);
	}

/*
	void TransformTFToEigen(const tf::Transform &t, Eigen::Isometry3d &k)
	{
		Eigen::Affine3d affine;
		tf::TransformTFToEigen(t, affine);

		k.translation() = affine.translation();
		k.linear() = affine.rotation();
	}
*/

	double getAngle(tf::Transform& tShoulder,
		tf::Transform& tElbow,
		tf::Transform& tWrist)
	{
		Eigen::Isometry3d eShoulder, eElbow, eWrist;
		tf::TransformTFToEigen(tShoulder, eShoulder);	
		tf::TransformTFToEigen(tElbow, eElbow);	
		tf::TransformTFToEigen(tWrist, eWrist);	

		Eigen::Vector3d Vse, Vew;
		Vse = eShoulder.translation() - eElbow.translation();
		Vew = eElbow.translation() - eWrist.translation();

		double angle;
		angle = acos((Vse.dot(Vew))/(Vse.norm() * Vew.norm()));
		
		ROS_INFO("Angle value: %f\n", angle);

		return angle;
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher m_JointPublisher;
	tf::TransformListener listener;
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ROSHuboTester");

	ROS_INFO("Starting Hubo Test Publisher.\n");
	ROSHuboTester hi;

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		hi.publishSample(count);
		//hi.publishElbow();

		std::cout << "Published..." << std::endl;
		ros::spinOnce();

		ROS_INFO("Published.\n");

		loop_rate.sleep();
		++count;
	}

	ros::spin();

	return 0;
}
