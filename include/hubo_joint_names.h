/**
 * \file hubo_joint_names.h
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 *
 * \author Andrew Price
 */
#ifndef HUBO_JOINT_NAMES_H
#define HUBO_JOINT_NAMES_H

#include <string>

/**
 * \var HUBO_JOINT_NAMES
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 * NB: The size of the array must be equal to HUBO_JOINT_COUNT as defined in <hubo.h>
 */
const std::string HUBO_JOINT_NAMES[] = 
{
	"WST","NKY","NK1","NK2",
	"LSP","LSR","LSY","LEB","LWY","LWR","LWP",
	"RSP","RSR","RSY","REB","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKN","LAP","LAR","",
	"RHY","RHR","RHP","RKN","RAP","RAR",
	"RF1","RF2","RF3","RF4","RF5",
	"LF1","LF2","LF3","LF4","LF5",
};

/**
 * \var HUBO_URDF_JOINT_NAMES
 * \brief Contains std::strings for each of Hubo's joints in the URDF model, indexed according to <hubo.h>
 * NB: The size of the array must be equal to HUBO_JOINT_COUNT as defined in <hubo.h>
 */
const std::string HUBO_URDF_JOINT_NAMES[] = 
{
	"HPY","NKY","NK1","NK2", //HNR, HNP ?
	"LSP","LSR","LSY","LEP","LWY","LWR","LWP",
	"RSP","RSR","RSY","REP","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKP","LAP","LAR","",
	"RHY","RHR","RHP","RKP","RAP","RAR",
	"rightThumbKnuckle1","rightIndexKnuckle1","rightMiddleKnuckle1","rightRingKnuckle1","rightPinkyKnuckle1",
	"leftThumbKnuckle1","leftIndexKnuckle1","leftMiddleKnuckle1","leftRingKnuckle1","leftPinkyKnuckle1",
};

#endif //HUBO_JOINT_NAMES_H
