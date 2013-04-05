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
#include <pcl/registration/icp.h>

class ModelMatcher
{
public:
	ModelMatcher();
	virtual ~ModelMatcher();

	Eigen::VectorXf ComputeModelParameters(pcl::PointCloud<pcl::PointXYZ>& model);
	Eigen::VectorXi VectorHistogram(Eigen::VectorXf input, int numBins);
	std::vector<std::pair<int, double> > GetClosestModelsCoarse(Eigen::VectorXf search);
	std::vector<std::pair<int, double> > GetClosestModelsFine(pcl::PointCloud<pcl::PointXYZ>& search, std::vector<int> modelIndices);

	void LoadModelFiles(std::string directory, std::vector<std::string> files);
	void GetDatabaseCloud(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

	static double descriptorSimilarity(const Eigen::VectorXf a, const Eigen::VectorXf b);
	static bool scoreCompare(const std::pair<int, double> a, const std::pair<int, double> b);
private:
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> modelClouds;
	std::string modelDirectory;
	std::vector<std::string> modelFiles;
	std::vector<Eigen::VectorXf> modelDescriptors;
};

#endif /* MODELMATCHER_H_ */
