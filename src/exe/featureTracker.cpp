/**
 * \file featureTracker.cpp
 *
 * \brief Tracks the center of several detected keypoints.
 * Derived from http://stackoverflow.com/questions/5461148/sift-implementation-with-opencv-2-2
 *
 * \author Andrew Price
*/

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <vector>

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

bool matchSortStoL(const cv::DMatch& d1, const cv::DMatch& d2)
{
  return d1.distance < d2.distance;
}

bool matchSortLtoS(const cv::DMatch& d1, const cv::DMatch& d2)
{
  return d1.distance > d2.distance;
}

int main( int argc, const char** argv )
{
	std::string filenameBase = "images/Ryobi ";
	std::string filename;
	char c = ' ';

	// Very important!!
	cv::initModule_nonfree();
	/*
	//for (int fileIdx = 1; fileIdx <= 2; fileIdx++)
	{
		int fileIdx = 1;
		std::stringstream ss;	ss << fileIdx;
		filename = filenameBase + ss.str() + ".jpg";
		std::cout << filename << std::endl;
		cv::Mat img = cv::imread(filename);

		std::vector<cv::KeyPoint> keypoints;
		cv::Mat featureDescriptors;
		
		cv::Ptr<cv::FeatureDetector> featureDetector = cv::FeatureDetector::create("SURF");
		cv::Ptr<cv::DescriptorExtractor> descriptorExctractor = cv::DescriptorExtractor::create("SURF");

		featureDetector->detect(img, keypoints);
		descriptorExctractor->compute(img, keypoints, featureDescriptors);
		
		cv::Mat outputImg;
		cv::Scalar keypointColor = cv::Scalar(0, 255, 0); // Green Keypoints
		cv::drawKeypoints(img, keypoints, outputImg, keypointColor, cv::DrawMatchesFlags::DEFAULT);

		cv::namedWindow("Output");
		cv::imshow("Output", outputImg);


		while ((c = cv::waitKey(0)) != 'q'); // Hold window open
		
	}
	*/
	cv::Mat baseImg = cv::imread(filenameBase + "1.jpg");

	std::vector<cv::KeyPoint> baseKeypoints;
	cv::Mat baseFeatureDescriptors;

	std::vector<cv::KeyPoint> newKeypoints;
	cv::Mat newFeatureDescriptors;

	cv::Ptr<cv::FeatureDetector> featureDetector = cv::FeatureDetector::create("SURF");
	cv::Ptr<cv::DescriptorExtractor> descriptorExctractor = cv::DescriptorExtractor::create("SURF");
	
	// Compute search features
	featureDetector->detect(baseImg, baseKeypoints);
	descriptorExctractor->compute(baseImg, baseKeypoints, baseFeatureDescriptors);
	
	cv::Mat baseOutputImg;
	cv::Scalar keypointColor = cv::Scalar(0, 255, 0); // Green Keypoints
	cv::drawKeypoints(baseImg, baseKeypoints, baseOutputImg, keypointColor, cv::DrawMatchesFlags::DEFAULT);


	cv::namedWindow("Baseline");
	cv::imshow("Baseline", baseOutputImg);

	cv::FlannBasedMatcher matcher;
	
	// Initialize capturing live feed from the camera
	cv::VideoCapture capture;
	capture.open(CV_CAP_ANY);
	
	// Couldn't get a device? Throw an error and quit
	if(!capture.isOpened())
	{
		std::cout << "Could not initialize capturing...\n";
		return -1;
	}
	
	// An infinite loop
	while(true)
	{
		// Will hold a frame captured from the camera
		cv::Mat frame;

		// Grab an image from the camera
		if(!capture.read(frame))
		{
			// If we couldn't grab a frame... quit
			break;
		}

		// Detect new features
		featureDetector->detect(frame, newKeypoints);
		descriptorExctractor->compute(frame, newKeypoints, newFeatureDescriptors);

		// Compute new putative matches
		std::vector<cv::DMatch> matches;
		matcher.match(newFeatureDescriptors, baseFeatureDescriptors, matches);

		// Sort and trim to get best matches
		std::sort(matches.begin(), matches.end(), matchSortStoL);
		matches.resize(min(matches.size(),100));
		
		// Draw matches
		cv::Mat imgMatches;
		cv::drawMatches(baseImg, baseKeypoints, frame, newKeypoints, matches, imgMatches,
			cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
			cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		cv::imshow( "Matches", imgMatches );
	}


}

