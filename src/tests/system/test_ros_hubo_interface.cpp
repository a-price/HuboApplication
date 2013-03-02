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

#include "HuboApplication/tf_eigen.h"
#include "HuboApplication/hubo_joint_names.h"


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

	void publishArm()
	{
		//Eigen::Matrix< double, 6, 1 > cmdJoints;
		tf::StampedTransform tLS,tLE,tLW,tRS,tRE,tRW;
		bool foundPlayer = false;
		for (int player = 1; player <= 6 && !foundPlayer; player++)
		{
			std::string strPlayer = std::to_string(player);
			try
			{
				listener.lookupTransform("/openni_depth_frame","/right_shoulder_"+strPlayer,ros::Time(0),tRS);
				listener.lookupTransform("/openni_depth_frame","/right_elbow_"+strPlayer,ros::Time(0),tRE);
				listener.lookupTransform("/openni_depth_frame","/right_hand_"+strPlayer,ros::Time(0),tRW);

				listener.lookupTransform("/openni_depth_frame","/left_shoulder_"+strPlayer,ros::Time(0),tLS);
				listener.lookupTransform("/openni_depth_frame","/left_elbow_"+strPlayer,ros::Time(0),tLE);
				listener.lookupTransform("/openni_depth_frame","/left_hand_"+strPlayer,ros::Time(0),tLW);
				foundPlayer = true;
			}
			catch (tf::TransformException ex)
			{
				//ROS_ERROR("%s", ex.what());
				//return;
			}
		}
		if (!foundPlayer)
		{
			ROS_ERROR("No player found.");
			return;
		}

		sensor_msgs::JointState joints;
		//double elbow = getElbow(tS,tE,tW);

		Eigen::Vector4d leftArmAngles, rightArmAngles;
		getArmJoints(tRS, tRE, tRW, 0, rightArmAngles);
		//getArmJoints(tLS, tLE, tLW, 1, leftArmAngles);
		
		for (int idx = 0; idx < 4; idx++)
		{
			joints.position.push_back(leftArmAngles[idx]);
			joints.name.push_back(HUBO_JOINT_NAMES[LSP + idx]);
		}

		for (int idx = 0; idx < 4; idx++)
		{
			joints.position.push_back(rightArmAngles[idx]);
			joints.name.push_back(HUBO_JOINT_NAMES[RSP + idx]);
		}

		m_JointPublisher.publish(joints);
	}


	void getArmJoints(tf::Transform& tShoulder,
		tf::Transform& tElbow,
		tf::Transform& tWrist,
		int side, Eigen::Vector4d& angles)
	{
		Eigen::Isometry3d eShoulder, eElbow, eWrist;
		tf::TransformTFToEigen(tShoulder, eShoulder);	
		tf::TransformTFToEigen(tElbow, eElbow);	
		tf::TransformTFToEigen(tWrist, eWrist);	

		Eigen::Vector3d Vse, Vew;
		Vse = eShoulder.translation() - eElbow.translation();
		Vew = eElbow.translation() - eWrist.translation();

		double elbow = acos((Vse.dot(Vew))/(Vse.norm() * Vew.norm()));

		// In Hubo's coordinate standard
		//double sRoll = atan2(Vse.y(),Vse.z());
		//double sPitch = atan2(Vse.x(),Vse.z());

		// In NITE frames
		double sPitch = atan2(Vse.x(),Vse.y());
		double sRoll = atan2(Vse.z(),Vse.y());

		Eigen::Isometry3d eShoulderElbow = eShoulder.inverse() * eElbow;

		Eigen::Quaterniond rSE(eShoulderElbow.rotation());

		double sYaw = acos(rSE.w()) * 2;
		
		//ROS_INFO("Angle value: %f\n", angle);

		if (side == 0)
		{
			//angles = Eigen::Vector4d(sPitch, 1.57079632679-sRoll, sYaw, -elbow);
			angles = Eigen::Vector4d(-sPitch, -(1.57079632679-sRoll), 0, -elbow);
		}
		else
		{
			angles = Eigen::Vector4d(sPitch, sRoll, sYaw, elbow);
		}
	
		std::stringstream info;

		info << angles.matrix();

		ROS_INFO("Angles: \n%s", info.str().c_str());

		return;
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
		//hi.publishSample(count);
		hi.publishArm();

		std::cout << "Published..." << std::endl;
		ros::spinOnce();

		ROS_INFO("Published.\n");

		loop_rate.sleep();
		++count;
	}

	ros::spin();

	return 0;
}
