/**
 * \file Fastrak.h
 * \brief Encapsulates Fastrack access for ACH IPC channels.
 *
 * \Author Andrew Price
 */
#pragma once

#ifndef FASTRAK_H
#define FASTRAK_H

// ACH includes
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <ach.h>

// Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Message Results
#include "ft_flag_t.h"
#include "daemonizer.h"

#include <stdio.h>

//#define FASTRAK_CHAN_NAME "fastrak"

typedef struct
{
	float data[4][7];
} fastrak_data_c_t;

class Fastrak
{
public:
	double fastrakScale;

	Fastrak(std::string channel="fastrak", bool assert=false);
	~Fastrak(void);

	ft_flag_t initFastrak(bool assert=false);

	ach_status achUpdate();

	void setFastrakScale( double scale );
	double getFastrakScale();

	int getNumChannels();

	ft_flag_t getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor=1, bool update=true );
	ft_flag_t getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor=1, bool update=true );
	ft_flag_t getPose( Eigen::Isometry3d &tf, int sensor=1, bool update=true );

private:
	ach_channel_t chan_fastrak;
	fastrak_data_c_t fastrak;
	std::string fastrak_chan_name;
	
};

#endif // FASTRAK_H

