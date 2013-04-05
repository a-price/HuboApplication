/*
 * ModelMatcher.h
 *
 *  Created on: Apr 4, 2013
 *      Author: arprice
 */

#ifndef MODELMATCHER_H_
#define MODELMATCHER_H_

#include <string>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>

class ModelMatcher
{
public:
	ModelMatcher();
	virtual ~ModelMatcher();

	Eigen::VectorXf ComputeModelParameters(pcl::PointCloud<pcl::PointXYZ>& model);
	Eigen::VectorXi Histogram(Eigen::VectorXf input, int numBins);
	std::vector<int> GetClosestModels(Eigen::VectorXf search, int numModels);

	void LoadModelFiles(std::string directory, std::vector<std::string> files);
	void GetDatabaseCloud(int index, pcl::PointCloud<pcl::PointXYZ>& cloud);
private:
	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> modelClouds;
	std::string modelDirectory;
	std::vector<std::string> modelFiles;
	std::vector<Eigen::VectorXf> modelDescriptors;
};

#endif /* MODELMATCHER_H_ */
