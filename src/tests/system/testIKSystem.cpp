/**
 * \file TestIK.cpp
 * \brief Unit tests for forward and Inverse Kinematics
 *
 * \author Maxwell McRae
 */

#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <hubo.h>
#include "HuboKin.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef struct
{
	double x, y, z, error;
} reachability_t;

double compareT(Eigen::Isometry3d a, Eigen::Isometry3d b,
		Eigen::VectorXd weight)
{
	Eigen::Quaterniond qa(a.rotation());
	Eigen::Quaterniond qb(b.rotation());
	Eigen::Vector3d pa = a.translation();
	Eigen::Vector3d pb = b.translation();
	Eigen::VectorXd va(7), vb(7), verr(7), vScaled(7);
	va << pa, qa.x(), qa.y(), qa.z(), qa.w();
	vb << pb, qb.x(), qb.y(), qb.z(), qb.w();
	verr = vb - va;
	vScaled = weight.cwiseProduct(verr);
	return vScaled.squaredNorm();
}

// TODO: consider orientations other than identity
std::vector<reachability_t> TestReachability(double rotation)
{
	const double XMIN = -0.1, XMAX = 0.7, YMIN = -0.8, YMAX = .2, ZMIN = -1, ZMAX =
			0.3, STEPSIZE = 0.05;
	HK::HuboKin hubo;
	std::vector<reachability_t> results;

	Eigen::VectorXd weight(7);
	weight << 1, 1, 1, 1, 1, 1, 1; //change this for weights!
	Vector6d q;
	int count = 0, valid = 0;
	for (double x = XMIN; x <= XMAX; x += STEPSIZE)
	{
		for (double y = YMIN; y <= YMAX; y += STEPSIZE)
		{
			for (double z = ZMIN; z <= ZMAX; z += STEPSIZE)
			{
				++count;
				Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
				Eigen::Isometry3d result;
				target.translation().x() = x;
				target.translation().y() = y;
				target.translation().z() = z;
				target.rotate(Eigen::AngleAxisd(-(M_PI/6) * rotation, Eigen::Vector3d::UnitX()));
				hubo.armIK(q, target, Vector6d::Zero(), RIGHT);
				hubo.armFK(result, q, RIGHT);
				double ret = compareT(target, result, weight);
				//std::cout << ret << "\t";
				if (ret > 0.15)
				{
					continue;
				}
				else
				{
					valid++;
					reachability_t pointScore;
					pointScore.x = x;
					pointScore.y = y;
					pointScore.z = z;
					pointScore.error = ret;
					results.push_back(pointScore);
				}

			}

		}

	}

	return results;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ROSHuboTester");

	ROS_INFO("Starting Hubo Test Publisher.\n");

	ros::NodeHandle nh;
	ros::Publisher resultPublisher;

	resultPublisher = nh.advertise<visualization_msgs::MarkerArray>( "grasp_points", 0 );

	ros::Rate rate(1);
	double count;
	while (ros::ok())
	{
		std::vector<reachability_t> results = TestReachability(count);
		visualization_msgs::MarkerArray mArray;

		double maxError=0;

		for (int j = 0; j < results.size(); j++)
		{
			reachability_t point = results[j];
			if (point.error>maxError)maxError=point.error;
		}

		for (int i = 0; i < results.size(); i++)
		{
			reachability_t point = results[i];

			visualization_msgs::Marker marker;
			marker.header.frame_id = "Body_Torso";
			//marker.header.stamp = headerTime;
			marker.ns = "HuboApplication";
			marker.id = i;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = point.x;
			marker.pose.position.y = point.y;
			marker.pose.position.z = point.z;

			marker.scale.x = 0.1*point.error/maxError;
			marker.scale.y = 0.1*point.error/maxError;
			marker.scale.z = 0.1*point.error/maxError;
			marker.color.a = (1-point.error/maxError)*.70; // based on # & weight of connections

			marker.color.r = point.error/maxError; // based on parent
			marker.color.g = 1-point.error/maxError; //
			marker.color.b = 0.0;
			mArray.markers.push_back(marker);
		}

		resultPublisher.publish(mArray);

		ros::spinOnce();

		rate.sleep();
		count++;
	}


}
