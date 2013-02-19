/**
 * \file ros_fastrak.cpp
 * \brief Publishes Fastrak readings from ACH as TFs
 * 
 * \author Andrew Price
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>
#include "Fastrak.h"
#include "tf_eigen.h"

Fastrak fastrak;

void publishFastrack()
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	
	Eigen::Isometry3d pose;

	for (int i = 0; i < 4; i++)
	{
		fastrak.getPose(pose, i);
		tf::TransformEigenToTF(pose,transform);
		char chanChar[20];
		sprintf(chanChar, "ft_channel_%i", i);
		std::string chanStr = std::string(chanChar);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", chanStr));
		std::cout << pose.matrix() << std::endl;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "fastrak_tf_broadcaster");

	ros::NodeHandle node;

	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
		publishFastrack();
		ros::spinOnce();
		r.sleep();
	}
  return 0;
};
