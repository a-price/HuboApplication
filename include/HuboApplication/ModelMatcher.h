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
	Eigen::VectorXi VectorHistogram(Eigen::VectorXf input, int numBins);
	std::vector<int> GetClosestModels(Eigen::VectorXf search, size_t numModels);

	void LoadModelFiles(std::string directory, std::vector<std::string> files);
	void GetDatabaseCloud(int index, pcl::PointCloud<pcl::PointXYZ>& cloud);

	static double descriptorSimilarity(const Eigen::VectorXf a, const Eigen::VectorXf b);
	static bool scoreCompare(const std::pair<int, double> a, const std::pair<int, double> b);
private:
	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> modelClouds;
	std::string modelDirectory;
	std::vector<std::string> modelFiles;
	std::vector<Eigen::VectorXf> modelDescriptors;
};

#endif /* MODELMATCHER_H_ */
