/**
 * \file remove_body_from_image.cpp
 * \brief 
 *
 *  \date May 9, 2013
 *  \author arprice
 */

#include <ros/ros.h>

#include <rgbd_graph_segmentation/rgbd_graph_segmentation.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDSyncPolicy;

void colorCallbackTest(const sensor_msgs::ImageConstPtr color)
{
	ROS_INFO("Got a Color Image.");
}
void depthCallbackTest(const sensor_msgs::ImageConstPtr depth)
{
	ROS_INFO("Got a Depth Image.");
}

/**
 * \class SegmentedRobotRemover
 * \description
 * Segments the RGB-D image using modified F-H,
 * projects the arm and leg joints into the camera frame,
 * and deletes the segments containing the joints.
 */
class SegmentedRobotRemover
{
public:
	SegmentedRobotRemover()
		: visual_sub_ (nh_, "/camera/rgb/image_rect_color", 8),
		  depth_sub_ (nh_, "/camera/depth_registered/image_rect", 8),
		  visual_calib_sub_(nh_, "/camera/rgb/camera_info", 8),
		  sync_(RGBDSyncPolicy(8), visual_sub_, depth_sub_)
	{
		camera_info_acquired_ = false;
		visual_sub_.registerCallback(boost::bind(&colorCallbackTest, _1));
		depth_sub_.registerCallback(boost::bind(&depthCallbackTest, _1));
		visual_calib_sub_.registerCallback(boost::bind(&SegmentedRobotRemover::colorCalibCallback, this, _1));
		sync_.registerCallback(boost::bind(&SegmentedRobotRemover::kinectCallback, this, _1, _2));
	}

	void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth)
	{
		if (!camera_info_acquired_) {return;}
		ROS_INFO("Got Callback\n");

		// Get F-H Segmentation
		cv_bridge::CvImagePtr colorImgPtr = cv_bridge::toCvCopy(color, "bgr8");
		cv_bridge::CvImagePtr depthImgPtr = cv_bridge::toCvCopy(depth, "mono8");
		rgbd_graph_segmentation::Segmentation segmentation = rgbd_graph_segmentation::segment(colorImgPtr->image, depthImgPtr->image);

		// Project coordinates of all TF's into current camera frame
		std::vector<std::string> frameNames;
		std::vector<std::string> chainNames;

		listener_.chainAsVector("/Body_RWP", ros::Time(0),"/Body_Torso", ros::Time(0), "/Body_Torso", chainNames);
		frameNames.insert( frameNames.end(), chainNames.begin(), chainNames.end() );

		tf::StampedTransform tHeadJoint;
		Eigen::Vector3d point;
		std::vector<Eigen::Vector2i> points;

		std::cout << "Calibration: " << calibration_ << "\n";

		for (std::vector<std::string>::iterator iter = frameNames.begin();
			 iter != frameNames.end(); ++iter)
		{
			if (*iter == "NO_PARENT") {continue;}

			try
			{
				listener_.lookupTransform("/camera_depth_optical_frame", (*iter), ros::Time(0), tHeadJoint);
				Eigen::Vector3d point3d = Eigen::Vector3d( tHeadJoint.getOrigin().x(),
						 tHeadJoint.getOrigin().y(),
						 tHeadJoint.getOrigin().z());
				point3d /= point3d[2];
				point = calibration_ * point3d;
				if (point[0] > 0 && point[0] < colorImgPtr->image.cols &&
					point[1] > 0 && point[1] < colorImgPtr->image.rows)
				{
					points.push_back(Eigen::Vector2i(round((double)(point[0])), round((double)(point[1]))));
					std::cerr << *iter << ": " << points[points.size()-1].transpose() << "\n";
				}
			}
			catch(tf::TransformException& ex)
			{
				ROS_ERROR("%s", ex.what());
			}
		}

		// Get all segments containing a joint frame
		std::set<uint32_t> segments;
		for (std::vector<Eigen::Vector2i>::iterator iter = points.begin();
			 iter != points.end(); ++iter)
		{
			std::cout << (*iter).transpose() << "\n";
			uint32_t segment = segmentation.segmentContaining(std::pair<uint16_t, uint16_t>((uint16_t)((*iter)[1]), (uint16_t)((*iter)[0])));
			segments.insert(segment);
		}

		// Delete segments and republish
		for (std::set<uint32_t>::iterator iter = segments.begin();
			 iter != segments.end(); ++iter)
		{
			std::vector<rgbd_graph_segmentation::Pixel> pixels = segmentation.pixels(*iter);
			// set these pixels to bad...
		}

	}

	void colorCalibCallback(const sensor_msgs::CameraInfoConstPtr ciPtr)
	{
		ROS_INFO("Got a Calibration Matrix.");
		sensor_msgs::CameraInfo camera_info_ = (*ciPtr);
		calibration_ = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(camera_info_.K.elems);
		camera_info_acquired_ = true;
		visual_calib_sub_.unsubscribe();
	}

private:
	ros::NodeHandle nh_;
	message_filters::Subscriber<sensor_msgs::Image> visual_sub_ ;
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> visual_calib_sub_;
	message_filters::Synchronizer<RGBDSyncPolicy> sync_;
	tf::TransformListener listener_;
	Eigen::Matrix3d calibration_;
	bool camera_info_acquired_;
};

class URDFRobotRemover
{

};

int main(int argc, char** argv)
{
	ROS_INFO("Started remove_body_from_image.");
	ros::init(argc, argv, "remove_body_from_image");

	SegmentedRobotRemover srr;

	ros::spin();

	return 0;
}

