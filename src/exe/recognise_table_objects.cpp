/**
 * \file recognise_table_objects.cpp
 * \brief 
 *
 *  \date Apr 6, 2013
 *  \author Andrew Price
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <tabletop_object_detector/TabletopSegmentation.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>

#include "HuboApplication/tf_eigen.h"
#include "HuboApplication/ModelMatcher.h"

ros::Subscriber cloudSub;
ros::Publisher cloudPub;
tf::TransformListener* tfListener;
tf::TransformBroadcaster* tfBroadcaster;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr points)
{
	std::cerr << "Got a Point Cloud.\n";

	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered2(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered3(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudColored(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*points, *pCloud);

	// call segmenter, get pointclouds
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	Eigen::VectorXf fitPlane;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pCloud));

	// Get rid of the floor
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.12);
	ransac.computeModel();
	ransac.getModelCoefficients(fitPlane); // TODO: verify that this is the floor
	ransac.getInliers(inliers->indices);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (pCloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*pCloudFiltered);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (pCloudFiltered);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.2);
	pass.filter (*pCloudFiltered2);

	// Get rid of the table
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p2 (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pCloudFiltered2));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2 (model_p2);
	ransac2.setDistanceThreshold (.03);
	ransac2.computeModel();
	ransac2.getModelCoefficients(fitPlane); // TODO: verify that this is the table
	ransac2.getInliers(inliers->indices);

	extract.setInputCloud (pCloudFiltered2);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*pCloudFiltered2);

	// Filter bottom of table...
	if (fitPlane[3] < 0) { fitPlane *= -1;}
	inliers->indices.clear();
	for (int i = 0; i < pCloudFiltered2->points.size(); i++)
	{
		float len = fitPlane.topRows(3).dot(pCloudFiltered2->points[i].getVector3fMap());
		if (fabs(len)+0.003 > fitPlane[3])
		{
			inliers->indices.push_back(i);
		}
	}
	// assuming plane points directly up
	extract.setInputCloud (pCloudFiltered2);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*pCloudFiltered2);


	// Remove outlier points as noise
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (pCloudFiltered2);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*pCloudFiltered2);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (pCloudFiltered2);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pCloudFiltered2);
	ec.extract (cluster_indices);

	std::cout << "Found " << cluster_indices.size() << " clusters.\n";
	if (cluster_indices.size() < 1) {return;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pSubCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int cluster = 0; cluster < cluster_indices.size(); cluster ++)
	{
		for (int i = 0; i < cluster_indices[cluster].indices.size(); i++)
		{
			pcl::PointXYZRGB cPoint;
			pcl::PointXYZ point = pCloudFiltered2->points[cluster_indices[cluster].indices[i]];
			cPoint.x = point.x;
			cPoint.y = point.y;
			cPoint.z = point.z;
			cPoint.r = (cluster==0) ? 255 : 0;
			cPoint.g = (cluster==1) ? 255 : 0;
			cPoint.b = (cluster==2) ? 255 : 0;
			pCloudColored->points.push_back(cPoint);
		}
	}


	pCloudColored->header = pCloudFiltered2->header;
	sensor_msgs::PointCloud2 filteredPoints;
	pcl::toROSMsg(*pCloudColored, filteredPoints);

	cloudPub.publish(filteredPoints);

	if (cluster_indices[0].indices.size() < 50) {return;}


	boost::shared_ptr<pcl::PointIndices> *ptrCluster = new boost::shared_ptr<pcl::PointIndices>(&cluster_indices[0]);
	extract.setInputCloud (pCloudFiltered2);
	extract.setIndices (*ptrCluster);
	extract.setNegative (false);
	extract.filter (*pSubCloud);
	ModelMatcher mm;
	Eigen::Isometry3f bottlePose = mm.MatchCylinderModel(pSubCloud);//.cast<double>();

	tf::Transform tCyl;
	std::cerr << "Got pose.";
	tf::TransformEigenToTF(bottlePose.cast<double>(), tCyl);

	std::cerr << "Got msg.";
	//std::cerr << tCyl;
	tfBroadcaster->sendTransform(tf::StampedTransform(tCyl, ros::Time::now(), "/camera_depth_optical_frame", "/cylinder"));

	// switch to pc2

	// recognize with database

	// publish model types
}


int main(int argc, char** argv)
{
	ROS_INFO("Started recognise_table_objects.");
	ros::init(argc, argv, "recognise_table_objects");
	ros::NodeHandle nh;

	// subscribe to camera point cloud
	cloudSub = nh.subscribe("/camera/depth/points", 2, cloudCallback);
	cloudPub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
	tfListener = new tf::TransformListener;
	tfBroadcaster = new tf::TransformBroadcaster;
	//ros::ServiceClient tsrClient = nh.serviceClient<tabletop_object_detector::TabletopSegmentationRequest>("/hubo/set_arm");

	ros::spin();

	return 0;
}

