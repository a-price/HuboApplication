#include <boost/thread/thread.hpp>

#include <stdint.h>
#include <stdlib.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ach.h"


void serialize(int numChannels, Eigen::Isometry3f* poses, float* data)
{
	for (int i = 0; i < numChannels; i++)
	{
		data[i*7 + 0] = poses[i].translation().x();
		data[i*7 + 1] = poses[i].translation().y();
		data[i*7 + 2] = poses[i].translation().z();
		
		data[i*7 + 3] = poses[i].rotation().coeff(3);
		data[i*7 + 4] = poses[i].rotation().coeff(0);
		data[i*7 + 5] = poses[i].rotation().coeff(1);
		data[i*7 + 6] = poses[i].rotation().coeff(2);
	}
}

int main(int argc, char* argv[])
{
	int numChannels = 4;
	if (argc == 2)
	{
		numChannels = atoi(argv[1]);
	}

	// Create Data Storage
	Eigen::Isometry3f* poses;
	float* data;
	poses = new Eigen::Isometry3f[numChannels];
	data = new float[numChannels*7];

	// Initialize Data
	for (int i = 0; i < numChannels; i++)
	{
		poses[i].setIdentity();
	}

	// Create ach channels
	ach_channel_t chan;
	ach_status r = ach_open(&chan, "fastrak", NULL);

	while (true)
	{
		// Write data to ach channel
		serialize(numChannels, poses, data);
		for (int i = 0; i < numChannels; i++)
		{
			for (int j = 0; j < 7; j++)
			{
				std::cout << "\t" << data[i*7+j];
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;

		ach_put( &chan, data, sizeof(float)*numChannels*7);

		for (int i = 0; i < numChannels; i++)
		{
			std::cout << poses[i].matrix() << std::endl;
			poses[i].rotate(Eigen::Quaternionf(1,1,1,1).normalized());
			poses[i].translation() = Eigen::Vector3f::Random();
		}
		boost::this_thread::sleep( boost::posix_time::milliseconds(750) );
	}

	delete[] poses;
	delete[] data;
}
