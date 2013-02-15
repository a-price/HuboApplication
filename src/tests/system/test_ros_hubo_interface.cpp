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

#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

class ROSHuboTester
{
public:
	ROSHuboTester()
	{
		m_JointPublisher = nh_.advertise<sensor_msgs::JointState>("/hubo/joints", 1);
	}

	void publishSample(int seed)
	{
		//Eigen::Matrix< double, 6, 1 > cmdJoints;
		sensor_msgs::JointState joints;
		joints.position.push_back((cos(seed/100)+1)/3);
		m_JointPublisher.publish(joints);
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher m_JointPublisher;
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_tracker");

	ROSHuboTester hi;

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		hi.publishSample(count);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	ros::spin();

	return 0;
}