/**
 * \file HuboStateROS.h
 * \brief Provides ROS representation of Hubo's state
 * 
 * \author Andrew Price
 */

#ifndef HUBO_STATE_ROS_H
#define HUBO_STATE_ROS_H

#include <ach.h>
#include <hubo.h>
#include <hubo_joint_names.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <sensor_msgs/JointState.h>

/**
 * \class HuboManipulator
 * \brief Provides ROS representation of Hubo's state
 */
class HuboStateROS
{
public:
	/// Constructor
	HuboStateROS(std::string chanName = "hubo-state");

	/// Destructor
	~HuboStateROS();
	ach_status updateState();
	hubo_state getState(bool update = true);

	sensor_msgs::JointState getJointState(bool update = true);

private:
	ach_channel_t mAchChan;
	std::string mAchChanName;
	hubo_state mHuboState;
};


#endif // HUBO_STATE_ROS_H