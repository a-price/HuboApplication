/**
 * \file recognise_table_objects.cpp
 * \brief 
 *
 *  \date Apr 6, 2013
 *  \author Andrew Price
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <tabletop_object_detector/TabletopSegmentation.h>

#include <pcl/ros/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

ros::Subscriber cloudSub;
ros::Publisher cloudPub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr points)
{
	ROS_INFO("Got a Point Cloud.");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered3(new pcl::PointCloud<pcl::PointXYZ>);
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

	// Get rid of the table
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p2 (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pCloudFiltered));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2 (model_p2);
	ransac2.setDistanceThreshold (.04);
	ransac2.computeModel();
	ransac2.getModelCoefficients(fitPlane); // TODO: verify that this is the table
	ransac2.getInliers(inliers->indices);

	extract.setInputCloud (pCloudFiltered);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*pCloudFiltered2);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (pCloudFiltered2);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.2);
	pass.filter (*pCloudFiltered3);

	sensor_msgs::PointCloud2 filteredPoints;
	pcl::toROSMsg(*pCloudFiltered3, filteredPoints);

	cloudPub.publish(filteredPoints);



	//tabletop_object_detector::TabletopSegmentationRequest tsr;
	//tsr.table


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
	//ros::ServiceClient tsrClient = nh.serviceClient<tabletop_object_detector::TabletopSegmentationRequest>("/hubo/set_arm");

	ros::spin();

	return 0;
}

