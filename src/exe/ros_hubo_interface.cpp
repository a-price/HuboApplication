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

//#include "Hubo_Tech.h"
#include "HuboManipulator.h"
#include "HuboStateROS.h"

class ROSHubo
{
public:
	ROSHubo()
	{
		m_JointSubscriber = nh_.subscribe("/hubo/target_joints", 1, &ROSHubo::jointCmdCallback, this);
		m_JointPublisher = nh_.advertise("/hubo/joints", 1);
	}

	void jointCmdCallback(const sensor_msgs::JointStateConstPtr& joints)
	{
		Eigen::Matrix< double, 6, 1 > cmdJoints;
		cmdJoints[3] = -joints->position[0];
		ROS_INFO("Got a joint: %f\n", cmdJoints[3]);
		m_Manip.setJoint(REB, -joints->position[0]);
		
		ROS_INFO("Remote value: %f\n", m_HuboState.getState().joint[REB].pos);
	}

	void publishState()
	{
		m_JointPublisher.publish(m_HuboState.getJointState());
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber m_JointSubscriber;
	ros::Publisher m_JointPublisher;
	
	HuboManipulator m_Manip;
	HuboStateROS m_HuboState;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ROSHubo");
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
