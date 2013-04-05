/**
 * \file TestModelMatcher.cpp
 * \brief Unit tests for model matching using sample objects
 *
 * \author Andrew Price
 */

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <iostream>
#include <sstream>
#include "HuboApplication/SampleModelGenerator.h"
#include "HuboApplication/ModelMatcher.h"

/**
 * \class TestModelMatcher
 * \brief Unit tests for HuboKin FK/IK.
 */
class TestModelMatcher : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestModelMatcher );
	CPPUNIT_TEST(TestVectorHistogram);
	CPPUNIT_TEST(TestModelMatcherMain);
	CPPUNIT_TEST(TestModelMatcherPCD);
	CPPUNIT_TEST_SUITE_END();

public:

	std::string directory;
	ModelMatcher mm;
	/**
	 * \fn setUp
	 * \brief Initializes test context for following unit tests
	 */
	void setUp()
	{
		// Load .pcd files
		directory = "/home/arprice/fuerte_workspace/sandbox/HuboApplication/src/tests/unit/resources/";
		std::vector<std::string> files;

		files.push_back("ism_train_cat.pcd");
		files.push_back("ism_train_wolf.pcd");
		files.push_back("ism_train_horse.pcd");
		files.push_back("ism_train_lioness.pcd");
		files.push_back("ism_train_michael.pcd");

		mm.LoadModelFiles(directory, files);
		std::cout << "Loaded Models.\n" << std::endl;
	}

	/**
	 * \fn tearDown
	 * \brief Cleans up test context for unit tests
	 */
	void tearDown()
	{

	}

	void TestModelMatcherMain()
	{
		SampleModelGenerator smg;
		pcl::PointCloud<pcl::PointXYZ>::Ptr sample = smg.GenerateSamplePlane(Eigen::Isometry3f::Identity(), 1, 0.1);
		mm.ComputeModelParameters(*sample);
		std::cout << "Exiting Main." << std::endl;
	}

	void TestModelMatcherPCD()
	{
		std::cerr << "Starting file test.\n";

		pcl::PointCloud<pcl::PointXYZ>::Ptr test(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile <pcl::PointXYZ>(directory + "ism_train_cat.pcd", *test);
		std::cerr << "Getting model parameters for test file.\n";
		Eigen::VectorXf descriptor = mm.ComputeModelParameters(*test);

		std::cerr << "Preparing to compute neighbors.\n";
		mm.GetClosestModels(descriptor, 2);
		std::cerr << "Exiting file test.\n";
	}

	void TestVectorHistogram()
	{
		Eigen::VectorXf temp(10);
		for (int i = 0; i < temp.rows(); i++)
		{
			temp(i) = i;
		}

		Eigen::VectorXi hist = mm.VectorHistogram(temp, 10);
		std::stringstream ss;
		ss << hist.transpose() << std::endl;
		ss << "Original: " << temp.transpose();
		CPPUNIT_ASSERT_MESSAGE("Histogram: " + ss.str(),hist == Eigen::VectorXi::Ones(hist.rows()));
	}

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestModelMatcher);
