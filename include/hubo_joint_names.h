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
 * \var hubo_joint_names
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 */
static const std::string HUBO_JOINT_NAMES[] = 
{
	"WST","NKY","NK1","NK2",
	"LSP","LSR","LSY","LEB","LWY","LWR","LWP",
	"RSP","RSR","RSY","REB","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKN","LAP","LAR","",
	"RHY","RHR","RHP","RKN","RAP","RAR",
	"RF1","RF2","RF3","RF4","RF5",
	"LF1","LF2","LF3","LF4","LF5",
};

#endif //HUBO_JOINT_NAMES_H
