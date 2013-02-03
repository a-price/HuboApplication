typedef enum
{
	WST		=0,		///<	Trunk Yaw

	NKY		=1,		///<	Neck Yaw
	NK1		=2,		///<	Neck 1
	NK2		=3,		///<	Neck 2

	LSP		=4,		///<	Left Shoulder Pitch
	LSR		=5,		///<	Left Shoulder Yaw
	LSY		=6,		///<	Left Shoulder Roll
	LEB		=7,		///<	Left Elbow Pitch
	LWY		=8,		///<	Left Wrist yaw
	LWR		=9,		///<	Left Wrist roll
	LWP		=10,	///<	Left Wrist pitch

	RSP		=11,	///<	Right Shoulder Pitch
	RSR		=12,	///<	Right Shoulder Roll
	RSY		=13,	///<	Right Shoulder Yaw
	REB		=14,	///<	Right Elbow Pitch
	RWY		=15,	///<	Right Wrist yaw
	RWR		=16,	///<	Right Wrist roll
	RWP		=17,	///<	Right Wrist Pitch

	LHY		=19,	///<	Left Hip Yaw
	LHR		=20,	///<	Left Hip Roll
	LHP		=21,	///<	Left Hip Pitch
	LKN		=22,	///<	Left Knee Pitch
	LAP		=23,	///<	Left Ankle Pitch
	LAR		=24,	///<	Left Ankle Roll

	RHY		=26,	///<	Right Hip Yaw
	RHR		=27,	///<	Right Hip Roll
	RHP		=28,	///<	Right Hip Pitch
	RKN		=29,	///<	Right Knee Pitch
	RAP		=30,	///<	Right Ankle Pitch
	RAR		=31,	///<	Right Ankle Roll

	RF1		=32,	///<	Right Finger
	RF2		=33,	///<	Right Finger
	RF3		=34,	///<	Right Finger
	RF4		=35,	///<	Right Finger
	RF5		=36,	///<	Right Finger
	LF1		=37,	///<	Left Finger
	LF2		=38,	///<	Left Finger
	LF3		=39,	///<	Left Finger
	LF4		=40,	///<	Left Finger
	LF5		=41		///<	Left Finger
} hubo_joint_t;

typedef enum {
	SUCCESS = 0,	///< The command returned successfully
	JOINT_OOB,      ///< The joint you tried to specify is out of bounds
	SENSOR_OOB,     ///< You requested data from a sensor which doesn't exist
	VALUE_OOB,      ///< Some generic value was out of acceptable bounds
	WRONG_MODE,     ///< You are not in the correct control mode to do what you asked
	BAD_SIDE,       ///< You did not use LEFT or RIGHT correctly
	SHORT_VECTOR,   ///< The VectorXd you tried to use has too few entries
	LONG_VECTOR,    ///< The VectorXd you tried to use has too many entries
	REF_STALE,      ///< The reference values were not able to update for some reason
	STATE_STALE,    ///< The state values were not able to update for some reason
	FASTRAK_STALE,  ///< The Fastrak values were not able to update for some reason
	ALL_STALE,      ///< Nothing was able to update for some reason
	CHAN_OPEN_FAIL, ///< A channel failed to open

} hp_flag_t;