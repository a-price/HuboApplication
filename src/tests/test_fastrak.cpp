/**
 * \file test_fastrak.cpp
 * \brief Attempts to read Fastrak data published over ach and print it to the terminal
 *
 * \Author Andrew Price
 */

#include "Fastrak.h"
#include <Eigen/Geometry>
#include <iostream>

int main()
{
	Fastrak fastrak;
	Eigen::Isometry3d pose;
	while (true)
	{
		fastrak.achUpdate();
		for (int i = 0; i < fastrak.getNumChannels(); i++)
		{
			fastrak.getPose(pose, i, false);
			std::cout << "Sensor " << i << ": " << std::endl;
			std::cout << pose.matrix() << std::endl;
		}
	}
	return 0;
}