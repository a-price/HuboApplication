/**
 * \file ColorTrackerParams.h
 * \brief Contains struct for color tracker settings
 * 
 * \author Andrew Price
 */

#include <opencv2/core/core.hpp>

struct ColorTrackerParams
{
	cv::Scalar lowerColorBound;
	cv::Scalar upperColorBound;

	int erosionRadius;
	int dilationRadius;

	int blurRadius;
};
