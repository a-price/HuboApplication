/**
* \file ros_hubo_interface.cpp
*
* \brief ROS node to send Hubo joint commands.
* Potentially more functionality to follow
*
* \author Andrew Price
*/

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Core>

#include "HuboApplication/tf_eigen.h"
#include "HuboApplication/HuboManipulator.h"
#include "HuboApplication/HuboStateROS.h"
#include "HuboApplication/SetHuboJointPositions.h"
#include "HuboApplication/SetHuboArmPose.h"

class ROSHubo
{
public:
	ROSHubo()
	{
		// Set up hubo
		/*
		m_Manip.homeJoints();
		m_Manip.sendCommand();
		usleep(2000000);
		bool fullHome = true;
		hubo_state s = m_HuboState.getState(true);
		for (int i = 0; i < NUM_UPPER_BODY_JOINTS; i++)
		{
			bool homeSuccess = (s.status[i].homeFlag == HUBO_HOME_OK);
			fullHome = fullHome && homeSuccess;
			if (!homeSuccess)
			{
				m_Manip.homeJoint(i, false);
				ROS_ERROR("Failed to home %i.", i);
			}
		}
		if (!fullHome)
		{
			m_Manip.sendCommand();
			usleep(2000000);
		}
		*/

		//m_JointSubscriber = nh_.subscribe("/hubo/target_joints", 1, &ROSHubo::jointCmdCallback, this);
		m_JointPublisher = nh_.advertise<sensor_msgs::JointState>("/hubo/joint_states", 1);

		m_JointService = nh_.advertiseService("/hubo/set_joints", &ROSHubo::srvSetHuboJointPositions, this);
		m_PoseService = nh_.advertiseService("/hubo/set_arm", &ROSHubo::srvSetHuboArmPose, this);
	}

	bool srvSetHuboJointPositions(HuboApplication::SetHuboJointPositions::Request &req,
								  HuboApplication::SetHuboJointPositions::Response &res)
	{
		bool response = true;
		// Look up the index of each joint by its name and add it to the control signal
		sensor_msgs::JointState joints = req.Targets;
		std::map<std::string, int>::const_iterator it;
		for (int i = 0; i < joints.position.size(); i++)
		{
			it = HUBO_JOINT_NAME_TO_INDEX.find(joints.name[i]);
			if (it == HUBO_JOINT_NAME_TO_INDEX.end())
			{
				ROS_ERROR("Joint name '%s' is unknown.", joints.name[i].c_str());
						response = false;
			}
					else
					{
						m_Manip.setJoint(it->second,joints.position[i]);
					}
			}

			m_Manip.sendCommand();
			res.Success = response;
		return response;
	}
	
	bool srvSetHuboArmPose(HuboApplication::SetHuboArmPose::Request &req,
						   HuboApplication::SetHuboArmPose::Response &res)
	{
		bool response = true;
		Eigen::Affine3d tempPose;
		Eigen::Isometry3d armPose;

		tf::poseMsgToEigen(req.Target, tempPose);
		if (tempPose(0,3) != tempPose(0,3)) // null check
		{
			res.Success = false;
			return false;
		}

		//std::cerr << tempPose.matrix() << std::endl;

		armPose = tempPose.matrix();
		//armPose.translation() = tempPose.translation();
		//armPose.linear() = tempPose.rotation();
		//std::cerr << armPose.matrix() << std::endl;

		m_Manip.setControlMode(END_EFFECTOR);
		m_Manip.setAngleMode(QUATERNION);
		m_Manip.setPose(armPose, req.ArmIndex);
		m_Manip.sendCommand();
		//std::cerr << armPose.matrix() << std::endl;
			res.Success = response;
		return response;
	}

	void jointCmdCallback(const sensor_msgs::JointStateConstPtr& joints)
	{
		// Look up the index of each joint by its name and add it to the control signal
		std::map<std::string, int>::const_iterator it;
		for (int i = 0; i < joints->position.size(); i++)
		{
			it = HUBO_JOINT_NAME_TO_INDEX.find(joints->name[i]);
			if (it == HUBO_JOINT_NAME_TO_INDEX.end())
			{
				ROS_ERROR("Joint name '%s' is unknown.", joints->name[i].c_str());
				continue;
			}
			m_Manip.setJoint(it->second,joints->position[i]);
		}

		m_Manip.sendCommand();
		
		//ROS_INFO("Remote value: %f\n", m_HuboState.getState().joint[RKN].pos);
	}

	void publishState()
	{
		//ROS_INFO("Publishing State...");
		hubo_state temp = m_HuboState.getState(true);
		for (int i = 0; i < HUBO_JOINT_COUNT; i++)
		{
			std::cout << "class get " << i << ": " << temp.joint[i].pos << std::endl;
		}

		m_JointPublisher.publish(m_HuboState.getJointState(true));

		// Have to publish head separately, since we don't have an accurate URDF
		// TODO: Replace with values derived from known angles + calibration
		Eigen::Isometry3d psA, psB;
		psA = Eigen::Isometry3d::Identity();
		psB = Eigen::Isometry3d::Identity();

		//const double alpha = 0.523598776, beta = 1.04719755;
		const double alpha = 0.86602540378, beta = 1.04719755;
		psA.translate(Eigen::Vector3d(0.09, 0.0, 0.28));
		psA.rotate(Eigen::Quaterniond(cos(alpha/2), 0, sin(alpha/2), 0).normalized());
		//psA.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
		//psA.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
		psB.translate(Eigen::Vector3d(0.10, 0.0, 0.23));
		psB.rotate(Eigen::Quaterniond(cos(beta/2), 0, sin(beta/2), 0).normalized());
		//psB.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
		//psB.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

		tf::Transform tfA,tfB;
		tf::TransformEigenToTF(psA, tfA);
		tf::TransformEigenToTF(psB, tfB);

		m_TFBroad.sendTransform(tf::StampedTransform(tfA, ros::Time::now(), "/Body_Torso", "/camera_link"));
		m_TFBroad.sendTransform(tf::StampedTransform(tfB, ros::Time::now(), "/Body_Torso", "/camera_link1"));
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber m_JointSubscriber;
	ros::Publisher m_JointPublisher;
	tf::TransformBroadcaster m_TFBroad;

	ros::ServiceServer m_JointService;
	ros::ServiceServer m_PoseService;
	
	HuboManipulator m_Manip;
	HuboStateROS m_HuboState;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ROSHubo");
	ROS_ERROR("Setting up Hubo Relay with userID: %i.", geteuid());

	// Initialize Ach channels
	if (geteuid() == 0 && system(NULL))
	{
		int i = 0;
		i = i | system("ach -1 -C \"hubo-state\" -m 10 -n 3000 -o 666");
		i = i | system("ach -1 -C \"hubo-manip\" -m 10 -n 3000 -o 666");

		if (i != 0) 
		{
			ROS_ERROR("Failed to create ach channel.");
			return -1;
		}

		i = i | system("achd pull -d 192.168.1.245 \"hubo-state\"");
		i = i | system("achd push -d 192.168.1.245 \"hubo-manip\"");

		if (i != 0) 
		{
			ROS_ERROR("Failed to push ach channel.");
			return -1;
		}
	}

	ROS_INFO("Started Hubo Relay.");

	ROSHubo hi;

	ros::Rate r(5); // 5 hz
	while (ros::ok())
	{
		hi.publishState();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
