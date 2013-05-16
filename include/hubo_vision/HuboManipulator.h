/**
 * \file HuboManipulator.h
 * \brief Abstracts access to manipulator daemon's ach channels
 * 
 * \author Andrew Price
 */

#ifndef HUBO_MANIPULATOR_H
#define HUBO_MANIPULATOR_H

#include "hubo.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <stdio.h>

#include "hubo_vision/manipulation_instruction_t.h"

/**
 * \class HuboManipulator
 * \brief Abstracts access to manipulator daemon's ach channels
 */
class HuboManipulator
{
public:
	/// Constructor
	HuboManipulator(std::string chanName = "hubo-manip");

	/// Destructor
	~HuboManipulator();

	/**
	 * \fn setInstruction
	 * \brief Sets the desired manipulation instruction to send to Hubo
	 */
	void setInstruction(manipulation_instruction_t inst);

	/**
	 * \fn setControlMode
	 * \brief Sets the desired instruction control_mode to send to Hubo
	 */
	void setControlMode(control_mode mode);

	/**
	 * \fn setAngleMode
	 * \brief Sets the desired instruction pose_angle_mode to send to Hubo
	 */
	void setAngleMode(pose_angle_mode mode);

	/**
	 * \fn setPose
	 * \brief Sets the desired instruction IK-based pose for a given side to send to Hubo
	 */
	void setPose(Eigen::Isometry3d pose, int side);

	/**
	 * \fn setJoint
	 * \brief Sets the reference value for a given joint index to send to Hubo
	 */
	void setJoint(int jointIndex, double val);

	/**
	 * \fn setJoints
	 * \brief Sets all reference values for joint positions to send to Hubo
	 */
	void setJoints(manip_q_vector_t q);

	/**
	 * \fn homeJoints
	 * \brief Puts the packet in "Home" mode and sets all commanded joints to home.
	 */
	void homeJoints(bool immediate = false);
	void homeJoints(manip_q_vector_t q, bool immediate = false);
	void homeJoint(int jointIndex, bool immediate = false);

	void sendCommand();
	void sendCommand(manipulation_instruction_t inst);

	void clearInstructionData();

private:
	ach_channel_t mAchChan;
	manipulation_instruction_t mInstruction;
	std::string mAchChanName;
};


#endif // HUBO_MANIPULATOR_H
