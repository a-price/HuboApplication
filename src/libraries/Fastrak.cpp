#include "Fastrak.h"
#include <iostream>

Fastrak::Fastrak(bool assert)
{
	memset( &fastrak, 0, sizeof(fastrak) );
	initFastrak(assert);
	fastrakScale = 1;
}


Fastrak::~Fastrak(void)
{
	//TODO: Do we need to clean up ACH memory?
}


ft_flag_t Fastrak::initFastrak(bool assert)
{
	int r = ach_open( &chan_fastrak, FASTRAK_CHAN_NAME, NULL );
	
	if( ACH_OK != r )
	{
		fprintf(stderr, "Unable to open fastrak channel: (%d) %s\n",
			r, ach_result_to_string((ach_status_t)r));
		//if(assert)
		//	daemon_assert( ACH_OK == r, __LINE__ );
		return CHAN_OPEN_FAIL;
	}

	return SUCCESS;
}

ach_status Fastrak::achUpdate()
{
	int r = ACH_OK;
	size_t fs;
	r = ach_get( &chan_fastrak, &fastrak, sizeof(fastrak), &fs, NULL, ACH_O_LAST );

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			std::cout << "\t" << fastrak.data[i][j];
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
	//if( r == ACH_OK )
	//	daemon_assert( sizeof(fastrak) == fs, __LINE__ );
}

void Fastrak::setFastrakScale( double scale ) { fastrakScale = scale; }
double Fastrak::getFastrakScale() { return fastrakScale; };
int Fastrak::getNumChannels(){ return 4; }

ft_flag_t Fastrak::getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor, bool update )
{
	int r = ACH_OK;
	if(update)
	{
		r = achUpdate();
	}

	if( sensor < getNumChannels() )
	{
		position[0] = fastrak.data[sensor][0]/fastrakScale;
		position[1] = fastrak.data[sensor][1]/fastrakScale;
		position[2] = fastrak.data[sensor][2]/fastrakScale;

		quat.w() = (double)fastrak.data[sensor][3];
		quat.x() = (double)fastrak.data[sensor][4];
		quat.y() = (double)fastrak.data[sensor][5];
		quat.z() = (double)fastrak.data[sensor][6];
	}
	else
		return SENSOR_OOB;

	if( ACH_OK != r )
		return FASTRAK_STALE;

	return SUCCESS;
}


ft_flag_t Fastrak::getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor, bool update )
{
	Eigen::Quaterniond quat;
	ft_flag_t flag = getPose( position, quat, sensor, update );

	if( flag==SENSOR_OOB )
		return flag;

	rotation = quat.matrix();

	return flag;
}


ft_flag_t Fastrak::getPose( Eigen::Isometry3d &tf, int sensor, bool update )
{
	Eigen::Vector3d position;
	Eigen::Quaterniond quat;

	ft_flag_t flag = getPose( position, quat, sensor, update );

	if( flag==SENSOR_OOB )
		return flag;

	tf = Eigen::Matrix4d::Identity();
	tf.translate( position );
	tf.rotate( quat );

	return flag;
	
}
