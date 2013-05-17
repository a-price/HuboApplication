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
#include <visualization_msgs/Marker.h>
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
#include <pcl/io/pcd_io.h>

#include <boost/timer.hpp>

#include "HuboApplication/tf_eigen.h"
#include "HuboApplication/ModelMatcher.h"

ros::Subscriber cloudSub;
ros::Publisher cloudPub;
tf::TransformListener* tfListener;
tf::TransformBroadcaster* tfBroadcaster;
ros::Publisher cylPub;
boost::timer timer;
const size_t MIN_CLOUD_SIZE = 50;

void tic()
{
	timer.restart();
}

void toc(std::string msg, bool restart=true)
{
	double te = timer.elapsed();
	std::cerr << "\t" << te << " -- " << msg << std::endl;
	if (restart)
		timer.restart();
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr points)
{
	std::cerr << "Got a Point Cloud.\n";

	tic();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudColored(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*points, *pCloud);
	toc("ROS->PCL");

	// call segmenter, get pointclouds
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	Eigen::VectorXf fitPlane;
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Filter to our region of interest (limit Z dimension)
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (pCloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.5);
	pass.filter (*pCloudFiltered);
	toc("ROI");

	if (pCloudFiltered->points.size() < MIN_CLOUD_SIZE) {return;}

	// Get rid of the table
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p2 (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pCloudFiltered));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2 (model_p2);
	ransac2.setDistanceThreshold (.03);
	ransac2.computeModel();
	ransac2.getModelCoefficients(fitPlane); // TODO: verify that this is the table
	ransac2.getInliers(inliers->indices);

	extract.setInputCloud (pCloudFiltered);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*pCloudFiltered);
	toc("RANSAC");

	if (pCloudFiltered->points.size() < MIN_CLOUD_SIZE) {return;}

	// Filter bottom of table...
	if (fitPlane[3] < 0) { fitPlane *= -1;}
	inliers->indices.clear();
	for (int i = 0; i < pCloudFiltered->points.size(); i++)
	{
		float len = fitPlane.topRows(3).dot(pCloudFiltered->points[i].getVector3fMap());
		if (fabs(len)+0.003 > fitPlane[3])
		{
			inliers->indices.push_back(i);
		}
	}
	// assuming plane points directly up
	extract.setInputCloud (pCloudFiltered);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*pCloudFiltered);
	toc("Table Projection");

	if (pCloudFiltered->points.size() < MIN_CLOUD_SIZE) {return;}

	// Remove outlier points as noise
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (pCloudFiltered);
	sor.setMeanK (MIN_CLOUD_SIZE);
	sor.setStddevMulThresh (1.0);
	sor.filter (*pCloudFiltered);
	toc("Outliers"); if (!ros::ok()) {return;}

	// FIXME: This is apparently a bug in PCL 1.5 where
	// if the # of points is too low it segfaults.
	if (pCloudFiltered->points.size() < MIN_CLOUD_SIZE*10) {return;}

	// Cluster nearby points
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (pCloudFiltered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.035); // 3.5cm
	ec.setMinClusterSize (MIN_CLOUD_SIZE);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pCloudFiltered);
	ec.extract (cluster_indices);
	toc("Clusters"); if (!ros::ok()) {return;}

	std::cerr << "Found " << cluster_indices.size() << " clusters.\n";
	if (cluster_indices.size() < 1) {return;}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pSubCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int cluster = 0; cluster < cluster_indices.size(); cluster ++)
	{
		for (int i = 0; i < cluster_indices[cluster].indices.size(); i++)
		{
			pcl::PointXYZRGB cPoint;
			pcl::PointXYZ point = pCloudFiltered->points[cluster_indices[cluster].indices[i]];
			cPoint.x = point.x;
			cPoint.y = point.y;
			cPoint.z = point.z;
			cPoint.r = (cluster==0) ? 255 : 0;
			cPoint.g = (cluster==1) ? 255 : 0;
			cPoint.b = (cluster==2) ? 255 : 0;
			pCloudColored->points.push_back(cPoint);
		}
	}
	toc("Colors");

	// Publish colored points for visualization.
	pCloudColored->header = pCloudFiltered->header;
	sensor_msgs::PointCloud2 filteredPoints;
	pcl::toROSMsg(*pCloudColored, filteredPoints);
	cloudPub.publish(filteredPoints);
	toc("Published");

	if (cluster_indices[0].indices.size() < MIN_CLOUD_SIZE) {return;}

	boost::shared_ptr<pcl::PointIndices> *ptrCluster = new boost::shared_ptr<pcl::PointIndices>(&cluster_indices[0]);
	extract.setInputCloud (pCloudFiltered);
	extract.setIndices (*ptrCluster);
	extract.setNegative (false);
	extract.filter (*pSubCloud);
	std::cerr << "Points before Cylinder: " << pSubCloud->points.size() << std::endl;

	ModelMatcher mm;
	mm.LoadModelFiles("./models/");
	std::vector<int> modelIndices; modelIndices.push_back(0);
	//mm.GetClosestModelsFine(*pSubCloud, modelIndices);
	Eigen::Isometry3f bottlePose = mm.MatchCylinderModel(pSubCloud);//.cast<double>();

	// Project the bottom of the cylinder to the table frame
	float pointPlaneDist = bottlePose.translation().dot(fitPlane.topRows(3));
	Eigen::Vector3f normTrans = fitPlane.topRows(3)*pointPlaneDist;
	//bottlePose.translate(normTrans);

	tf::Transform tCyl;
	tf::TransformEigenToTF(bottlePose.cast<double>(), tCyl);
	tfBroadcaster->sendTransform(tf::StampedTransform(tCyl, ros::Time::now(), "/camera_depth_optical_frame", "/cylinder"));
	toc("Cylinder");
	// recognize with database

	// publish model types
	// Create a marker to show parameters
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/camera_depth_optical_frame";
	//marker.header.stamp = headerTime;
	marker.ns = "hubo";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = tCyl.getOrigin().x();
	marker.pose.position.y = tCyl.getOrigin().y();
	marker.pose.position.z = tCyl.getOrigin().z();
	marker.pose.orientation.x = tCyl.getRotation().x();
	marker.pose.orientation.y = tCyl.getRotation().y();
	marker.pose.orientation.z = tCyl.getRotation().z();
	marker.pose.orientation.w = tCyl.getRotation().w();
	marker.scale.x = 0.06;
	marker.scale.y = 0.06;
	marker.scale.z = 0.10;
	marker.color.a = 1.0; // based on # & weight of connections
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	cylPub.publish(marker);
	//pcl::io::savePCDFileASCII("partial_cylinder.pcd", *pSubCloud);
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
	cylPub = nh.advertise<visualization_msgs::Marker>( "model_marker", 1 );
	//ros::ServiceClient tsrClient = nh.serviceClient<tabletop_object_detector::TabletopSegmentationRequest>("/hubo/set_arm");

	ros::spin();

	return 0;
}

