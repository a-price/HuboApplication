/**
 * \file HuboManipulator.cpp
 * \brief Abstracts access to manipulator daemon's ach channels
 * 
 * \author Andrew Price
 */

#include "hubo_vision/HuboManipulator.h"
#include <iostream>


HuboManipulator::HuboManipulator(std::string chanName)
{
	mAchChanName = chanName;
	memset(&mInstruction, 0, sizeof(mInstruction));
	ach_status r = ach_open( &mAchChan, mAchChanName.c_str(), NULL );

	if( ACH_OK != r )
	{
		fprintf(stderr, "\nUnable to open manipulation channel '%s', error: (%d) %s\n",
			mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}
}

HuboManipulator::~HuboManipulator()
{
	ach_close(&mAchChan);
}

void HuboManipulator::setInstruction(manipulation_instruction_t inst)
{
	mInstruction = inst;
}

void HuboManipulator::setControlMode(control_mode mode)
{
	mInstruction.controlMode = mode;
}

void HuboManipulator::setAngleMode(pose_angle_mode mode)
{
	mInstruction.poseMode = mode;
}

void HuboManipulator::setPose(Eigen::Isometry3d pose, int side)
{
	ee_pose_t newPose;
	newPose.x = pose.translation().x();
	newPose.y = pose.translation().y();
	newPose.z = pose.translation().z();

	if (mInstruction.poseMode == QUATERNION)
	{
		Eigen::Quaterniond quat(pose.rotation());
		newPose.i = quat.x();
		newPose.j = quat.y();
		newPose.k = quat.z();
		newPose.w = quat.w();
	}
	else if (mInstruction.poseMode == EULER_ANGLE)
	{
		// TODO: What is this?
	}
	if (side == RIGHT)
	{
		mInstruction.targetPoseRight = newPose;
	}
	else if (side == LEFT)
	{
		mInstruction.targetPoseLeft = newPose;
	}
}

void HuboManipulator::setJoint(int jointIndex, double val)
{
	mInstruction.controlMode = JOINT_VECTOR;
	if (jointIndex < NUM_UPPER_BODY_JOINTS && jointIndex >= 0)
	{
		mInstruction.targetJoints.data[jointIndex] = val;
	}
}

void HuboManipulator::setJoints(manip_q_vector_t q)
{
	mInstruction.controlMode = JOINT_VECTOR;
	mInstruction.targetJoints = q;
}

void HuboManipulator::homeJoints(bool immediate)
{
	mInstruction.controlMode = HOME_JOINTS;
	//memset(&mInstruction.targetJoints, 1, sizeof(mInstruction.targetJoints));
	for (int i = 0; i < NUM_UPPER_BODY_JOINTS; i++)
	{
		mInstruction.targetJoints.data[i] = 1;
	}
	if (immediate) sendCommand();
}

void HuboManipulator::homeJoints(manip_q_vector_t q, bool immediate)
{
	mInstruction.controlMode = HOME_JOINTS;
	mInstruction.targetJoints = q;
	if (immediate) sendCommand();
}

void HuboManipulator::homeJoint(int jointIndex, bool immediate)
{
	mInstruction.controlMode = HOME_JOINTS;
	if (jointIndex < NUM_UPPER_BODY_JOINTS && jointIndex >= 0)
	{
		mInstruction.targetJoints.data[jointIndex] = 1;
	}
	if (immediate) sendCommand();
}

void HuboManipulator::sendCommand()
{
	ach_status_t s = ach_put(&mAchChan, &mInstruction, sizeof(mInstruction));
	//std::cerr << mInstruction.targetPoseRight.x << "," << mInstruction.targetPoseRight.y << "," << mInstruction.targetPoseRight.z << std::endl;
	clearInstructionData();
}

void HuboManipulator::sendCommand(manipulation_instruction_t inst)
{
	mInstruction = inst;
	sendCommand();
}

void HuboManipulator::clearInstructionData()
{
	// Clear the joint and pose information after sending or changing message types
	memset(&mInstruction.targetJoints, 0, sizeof(mInstruction.targetJoints));
	memset(&mInstruction.targetPoseLeft, 0, sizeof(mInstruction.targetPoseLeft));
	memset(&mInstruction.targetPoseRight, 0, sizeof(mInstruction.targetPoseRight));
	//std::cerr << "Cleared.\n";
}
