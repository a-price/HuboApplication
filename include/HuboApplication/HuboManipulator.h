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

#include "HuboApplication/manipulation_instruction_t.h"

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

	void setInstruction(manipulation_instruction_t inst);
	void setControlMode(control_mode mode);
	void setAngleMode(pose_angle_mode mode);
	void setPose(Eigen::Isometry3d pose, int side);

	void setJoint(int jointIndex, double val);
	void setJoints(manip_q_vector_t q);

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
