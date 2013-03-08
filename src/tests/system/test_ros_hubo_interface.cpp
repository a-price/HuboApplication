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

#include "HuboApplication/SetHuboJointPositions.h"
#include "HuboApplication/SetHuboArmPose.h"


/**
* \file test_ros_hubo_interface.cpp
*
* \brief Generates and sends dummy Hubo joint commands.
*/
class ROSHuboTester
{
public:
	ROSHuboTester()
	{
		m_JointPublisher = nh_.advertise<sensor_msgs::JointState>("/hubo/target_joints", 1);
		m_Client = nh_.serviceClient<HuboApplication::SetHuboJointPositions>("set_hubo_joints");
	}

	void publishSample(int seed)
	{
		//Eigen::Matrix< double, 6, 1 > cmdJoints;
		sensor_msgs::JointState joints;
		joints.position.push_back(-(cos(seed/50.0)+1.0)/3.0);
		joints.name.push_back(HUBO_JOINT_NAMES[REB]);
		m_JointPublisher.publish(joints);
	}
	
	void callService(int seed)
	{
		sensor_msgs::JointState joints;
		joints.position.push_back(-(cos(seed/50.0)+1.0)/3.0);
		joints.name.push_back(HUBO_JOINT_NAMES[REB]);
		
		HuboApplication::SetHuboJointPositions srv;
		srv.request.Targets = joints;
		if (m_Client.call(srv))
		{
			ROS_INFO("Service returned %i.", srv.response.Success);
		}
		else
		{
			ROS_ERROR("Service call did not return.");
		}
	}
	

private:
	ros::NodeHandle nh_;
	ros::Publisher m_JointPublisher;
	ros::ServiceClient m_Client;
	
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
//		hi.publishSample(count);
		hi.callService(count);
		
		ros::spinOnce();

		ROS_INFO("Published.\n");

		loop_rate.sleep();
		++count;
	}

	ros::spin();

	return 0;
}
