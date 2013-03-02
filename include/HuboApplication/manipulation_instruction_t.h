/**
 * \file manipulation_instruction_t.h
 * \brief Defines control data structures passed to Hubo's manipulation daemon.
 *
 * \author Andrew Price
 */

#ifndef MANIPULATION_INSTRUCTION_T_H
#define MANIPULATION_INSTRUCTION_T_H

/**
 * \enum control_mode
 * \brief Tells the manipulation daemon whether to use workspace or joint-space control
 */
typedef enum
{
	END_EFFECTOR,   ///< Position the end-effector at the target pose using inverse kinematics
	JOINT_VECTOR    ///< Generate a trajectory in joint space only
} control_mode;

/**
 * \enum pose_angle_mode
 * \brief Tells the manipulation daemon whether to expect quaternion or euler angle orientation parameters
 */
typedef enum
{
	QUATERNION,   ///< Pose is specified in quaternions
	EULER_ANGLE   ///< Pose is specified in Euler angles
} pose_angle_mode;

/**
 * \union ee_pose_t
 * \brief Contains all pose parameters to pass to manipulation daemon for ik-control
 * This structure is defined as a union, so one can access the stored values like
 * myPose.data[1] or myPose.y, and they will refer to the same chunk of memory.
 * Names are provided for both quaternion and Euler angles.
 */
typedef union 
{
	double data[7];  ///< Array of raw data containing pose info.
	struct
	{
		double x,y,z;
		union
		{
			struct
			{
				double i,j,k,w;
			};

			struct
			{
				double alpha,beta,gamma,empty;
			};
		};
	};
}ee_pose_t;

/**
 * \struct manip_q_vector_t
 * \brief Contains all joint parameters to pass to manipulation daemon for joint control
 * Note that the joints are indexed according to the #defines provide by hubo.h.
 * This means data[WST] will get/set the trunk yaw joint.
 */
typedef struct 
{
	double data[1+3+7+7]; ///< Data for two arms, head P/T2, and torso. Sequence is identical to hubo.h #defines (WST, RPY, etc.)
} manip_q_vector_t;

/**
 * \struct manipulation_instruction_t
 * \brief Contains all parameters to pass to manipulation daemon for motion control
 */
typedef struct
{
	control_mode controlMode;      ///< Tells the manipulation daemon whether to use workspace or joint-space control
	pose_angle_mode poseMode;      ///< Tells the manipulation daemon whether to expect quaternion or euler angle orientation parameters
	bool incrementalMode;          ///< Tells teh manipulation daemon whether to use incremental or absolute position specifications
	
	ee_pose_t targetPoseLeft;      ///< Contains all pose parameters to pass to manipulation daemon for Left Handik-control
	ee_pose_t targetPoseRight;     ///< Contains all pose parameters to pass to manipulation daemon for Right Hand ik-control
	manip_q_vector_t targetJoints; ///< Contains all joint parameters to pass to manipulation daemon for joint control
} manipulation_instruction_t;

#endif //MANIPULATION_INSTRUCTION_T_H
