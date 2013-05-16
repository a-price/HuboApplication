/**
 * \file ros_activity_coordinator.cpp
 * \brief 
 *
 *  \date April 15, 2013
 *  \author Andrew Price
 */

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "hubo_vision/SetHuboObjectPose.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_activity_coordinator");
	ros::NodeHandle nh;
	ROS_INFO("Started ros_activity_coordinator.");

	tf::TransformListener listener;
	ros::ServiceClient pose_client;

	pose_client = nh.serviceClient<hubo_vision::SetHuboObjectPose>("/hubo/set_object");

	std::string text;

	while (ros::ok())
	{
		std::getline(std::cin, text);

		// Get pose in body frame, grab it. eventually in a different node
		tf::StampedTransform tTorsoObject;
		hubo_vision::SetHuboObjectPose srv;
		try
		{
			listener.lookupTransform("/Body_Torso", "/cylinder", ros::Time(0), tTorsoObject);
			ROS_INFO("Got transform!");

			tf::poseTFToMsg(tTorsoObject, srv.request.Target);
			srv.request.ObjectIndex = 0;
			pose_client.call(srv);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	}

	return 0;
}

