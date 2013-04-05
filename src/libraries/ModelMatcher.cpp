/*
 *
 * ModelMatcher.cpp
 *
 *  Created on: Apr 4, 2013
 *      Author: arprice
 */

#include "HuboApplication/ModelMatcher.h"

ModelMatcher::ModelMatcher()
{
	// TODO Auto-generated constructor stub

}

ModelMatcher::~ModelMatcher()
{
	// TODO Auto-generated destructor stub
}

Eigen::VectorXf ModelMatcher::ComputeModelParameters(pcl::PointCloud<pcl::PointXYZ>& model)
{
	const int numBins = 10;
	Eigen::MatrixXf cloudMat = model.getMatrixXfMap(3,4,0);

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(cloudMat, Eigen::ComputeFullU);
	//std::cout << svd.matrixU() << std::endl;

	cloudMat.transposeInPlace();

	//std::cout << cloudMat.rows() << "x" << cloudMat.cols()
	//		  << " * " << svd.matrixU().rows() << "x" << svd.matrixU().cols() << std::endl;
	// centroid, xyz bounding box, eccentricity, xyz moments
	cloudMat *= svd.matrixU();
	Eigen::VectorXf centroid(cloudMat.cols()), dimensions(cloudMat.cols());
	Eigen::VectorXf hist(numBins * cloudMat.cols());

	for (int i = 0; i < cloudMat.cols(); i++)
	{
		centroid(i) = cloudMat.col(i).mean();
		dimensions(i) = cloudMat.col(i).maxCoeff() - cloudMat.col(i).minCoeff();
		hist.block<numBins, 1>(i*numBins, 0) = VectorHistogram(cloudMat.col(i), numBins).cast<float>().normalized();
	}
	//std::cout << centroid << std::endl;
	//std::cout << dimensions << std::endl;

	Eigen::VectorXf output(cloudMat.cols()*(2 + numBins));
	output << centroid, dimensions, hist;

	return output;
	// for scanned clouds, consider mirroring about occlusion boundary
}

Eigen::VectorXi ModelMatcher::VectorHistogram(const Eigen::VectorXf input, const int numBins)
{
	float min = input.minCoeff()-0.0001, max = input.maxCoeff()+0.0001;
	float stepSize = (max - min) / numBins;
	Eigen::VectorXi histogram = Eigen::VectorXi::Zero(numBins);
	//std::cout << "Bins: " << numBins << "\t Rows: " << input.rows() << std::endl;

	for (int i = 0; i < input.rows(); i++)
	{
		int binNum = (int)((input(i)-min) / stepSize);
		//std::cout << binNum << std::endl;
		histogram(binNum) = histogram(binNum) + 1;
		//std::cout << histogram.transpose() << std::endl;
	}
	return histogram;

}

std::vector<int> ModelMatcher::GetClosestModels(Eigen::VectorXf search, size_t numModels)
{
	std::vector<int> closest;
	std::vector<std::pair<int, double> > scores;
	closest.push_back(1);
	for (int i = 0; i < modelDescriptors.size(); i++)
	{
		std::pair<int, double> score(i, descriptorSimilarity(search, modelDescriptors[i]));
		scores.push_back(score);
	}
	std::sort(scores.begin(), scores.end(), this->scoreCompare);

	for (int i = 0; i < std::min(numModels, modelDescriptors.size()); i++)
	{

	}
	return closest;
}

void ModelMatcher::LoadModelFiles(std::string directory, std::vector<std::string> files)
{
	modelDirectory = directory;
	modelFiles = files;
	for (int i = 0; i < files.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::io::loadPCDFile <pcl::PointXYZ>(modelDirectory + modelFiles[i], cloud);
		//modelClouds.push_back(cloud);

		Eigen::VectorXf descriptor = ComputeModelParameters(cloud);
		modelDescriptors.push_back(descriptor);
	}
}

void ModelMatcher::GetDatabaseCloud(int index, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	pcl::io::loadPCDFile <pcl::PointXYZ>(modelDirectory + modelFiles[index], cloud);
}


double ModelMatcher::descriptorSimilarity(const Eigen::VectorXf a, const Eigen::VectorXf b)
{
	return a.dot(b)/(a.norm()*b.norm());
}

bool ModelMatcher::scoreCompare(const std::pair<int, double> a, const std::pair<int, double> b)
{
	return (a.second > b.second);
}
