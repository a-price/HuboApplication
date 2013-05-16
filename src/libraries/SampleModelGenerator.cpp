/*
 * SampleModelGenerator.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: arprice
 */

#include "hubo_vision/SampleModelGenerator.h"

SampleModelGenerator::SampleModelGenerator()
{
	// TODO Auto-generated constructor stub

}

SampleModelGenerator::~SampleModelGenerator()
{
	// TODO Auto-generated destructor stub
}


pcl::PointCloud<pcl::PointXYZ>::Ptr SampleModelGenerator::GenerateSampleSphere(const Eigen::Isometry3f center, const float r, const float thetaRes, const float phiRes)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (float theta = 0; theta < 2*M_PI; theta += thetaRes)
	{
		for (float phi = 0; phi < M_PI; phi += phiRes)
		{
			pcl::PointXYZ point;
			point.x = r * sin(phi) * cos(theta);
			point.y = r * sin(phi) * sin(theta);
			point.z = r * cos(phi);

			cloud->points.push_back(point);
		}
	}

	pcl::transformPointCloud(*cloud, *cloud, center);

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleModelGenerator::GenerateSamplePlane(const Eigen::Isometry3f center, const float r, const float rRes)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (float x = -r; x <= r; x+=rRes)
	{
		for (float y = -r; y <= r; y+=rRes)
		{
			pcl::PointXYZ point;
			point.x = x;
			point.y = y;
			point.z = 0;
			cloud->points.push_back(point);
		}
	}

	pcl::transformPointCloud(*cloud, *cloud, center);

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleModelGenerator::GenerateSampleCube(const Eigen::Isometry3f center, const float r, const float rRes)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < 6; i++)
	{
		Eigen::Vector3f planeCenter = Eigen::Vector3f::Zero();
		planeCenter(i%3) = (i < 3) ? r : -r;
		Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
		pose.translate(planeCenter);
		(*cloud) += *GenerateSamplePlane(pose, r, rRes);
	}

	pcl::transformPointCloud(*cloud, *cloud, center);

	return cloud;
}
