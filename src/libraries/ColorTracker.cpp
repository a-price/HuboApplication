/**
 * \file ColorTracker.cpp
 * \brief Thresholds an image for a given color and returns the center of mass.
 * 
 * \author Andrew Price
 */

#include "ColorTracker.h"

ColorTracker::ColorTracker()
{
	ColorTrackerParams params;
	params.lowerColorBound = cv::Scalar(160,65,60);
	params.upperColorBound = cv::Scalar(190,256,256);
	params.erosionRadius = 3;
	params.dilationRadius = 3;
	params.blurRadius = 5;
}

ColorTracker::ColorTracker(ColorTrackerParams params)
{
	mParameters = params;
}

ColorTracker::~ColorTracker() {}

cv::Mat ColorTracker::thresholdImage(cv::Mat target)
{
	// Convert the image into an HSV image
	cv::Mat imgHSV;
	cv::cvtColor(target, imgHSV, CV_BGR2HSV);
	
	// Stores the matched color pixels
	cv::Mat imgThreshed;

	// Filter the image
	cv::GaussianBlur(imgHSV,imgHSV, cv::Size(5,5), 5);

	// Threshold the HSV image
	cv::inRange(imgHSV, mParameters.lowerColorBound, mParameters.upperColorBound, imgThreshed);

	imgHSV.release();

	return imgThreshed;
}

cv::Point ColorTracker::getCoM(cv::Mat target)
{
	// Get the thresholded image
	cv::Mat imgThresh = thresholdImage(target);

	// Perform a morphological opening
// TODO: incorporate parameters for erode and dilate
	cv::erode(imgThresh, imgThresh, cv::Mat());
	cv::dilate(imgThresh, imgThresh, cv::Mat());

	// Calculate the moments to estimate the position of the ball
	cv::Moments moments = cv::moments(imgThresh, 1);
	
	// The actual moment values
	double moment10 = moments.m10;
	double moment01 = moments.m01;
	double area = moments.m00;
	if (area == 0)
	{
		area = 1;
	}

	// Holding the last and current ball positions
	int posX = 0;
	int posY = 0;
	
	posX = (int)(moment10/area);
	posY = (int)(moment01/area);

	imgThresh.release();

	return cv::Point(posX, posY);
}


void ColorTracker::setTrackingParameters(ColorTrackerParams params)
{
	mParameters = params;
}

ColorTrackerParams ColorTracker::getTrackingParameters()
{
	return mParameters;
}

