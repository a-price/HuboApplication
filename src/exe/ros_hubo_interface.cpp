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
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "Hubo_Tech.h"

class ROSHubo
{
public:
	ROSHubo()
	{
		m_JointSubscriber = nh_.subscribe("/hubo/joints", 1, &ROSHubo::jointCallback, this);
	}

	void jointCallback(const sensor_msgs::JointStateConstPtr& joints)
	{
		Eigen::Matrix< double, 6, 1 > cmdJoints;
		cmdJoints[3] = -joints->position[0];
		ROS_INFO("Got a joint: %f\n", cmdJoints[3]);
		//hubo.setArmAngles(0, cmdJoints);
		//hubo.setJointAngle(-joints->position[0]);
		//hubo.setJointAngle(REB, 0.5);
		//hubo.sendControls();
		hubo.update();
		double remote = hubo.getJointAngle(REB);
		ROS_INFO("Remote value: %f\n", remote);
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber m_JointSubscriber;
	
	Hubo_Tech hubo;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ROSHubo");
	ROS_INFO("Started Hubo Relay.");

	ROSHubo hi;

	ros::spin();

	return 0;
}
