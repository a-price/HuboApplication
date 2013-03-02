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
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

//#include "Hubo_Tech.h"
#include "HuboManipulator.h"
#include "HuboStateROS.h"
#include "HuboApplication/SetHuboJointPositions.h"
#include "HuboApplication/SetHuboArmPose.h"

class ROSHubo
{
public:
	ROSHubo()
	{
		m_JointSubscriber = nh_.subscribe("/hubo/target_joints", 1, &ROSHubo::jointCmdCallback, this);
		m_JointPublisher = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
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
