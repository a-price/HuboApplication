/**
 * \file HuboStateROS.cpp
 * \brief Provides ROS representation of Hubo's state
 * 
 * \author Andrew Price
 */

#include "hubo_vision/HuboStateROS.h"

HuboStateROS::HuboStateROS(std::string chanName)
{
	mAchChanName = chanName;
	memset(&mHuboState, 0, sizeof(mHuboState));
	ach_status r = ach_open( &mAchChan, mAchChanName.c_str(), NULL );

	if( ACH_OK != r )
	{
		fprintf(stderr, "\nUnable to open state channel '%s', error: (%d) %s\n",
			mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}
}

HuboStateROS::~HuboStateROS()
{
	ach_close(&mAchChan);
}

ach_status HuboStateROS::updateState()
{
	ach_status r = ACH_OK;
	size_t fs;
	r = ach_get( &mAchChan, &mHuboState, sizeof(mHuboState), &fs, NULL, ACH_O_LAST );
	// Need some error checking here...
	
	return r;
}

hubo_state HuboStateROS::getState(bool update)
{
	if (update)
	{
		updateState();
	}
	return mHuboState;
}

sensor_msgs::JointState HuboStateROS::getJointState(bool update)
{
	if (update)
	{
		updateState();
	}

	sensor_msgs::JointState js;

	for (int i = 0; i < HUBO_JOINT_COUNT; i++)
	{
		js.position.push_back(mHuboState.joint[i].pos);
		js.velocity.push_back(mHuboState.joint[i].vel);
		js.name.push_back(HUBO_URDF_JOINT_NAMES[i]);
	}

	// How is hubo time specified?
	js.header.stamp = ros::Time::now(); // (mHuboState.time);

	return js;
}
