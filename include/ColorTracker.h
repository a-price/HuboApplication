/**
 * \file ColorTracker.h
 * \brief Thresholds an image for a given color and returns the center of mass.
 * 
 * \author Andrew Price
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ColorTrackerParams.h>

class ColorTracker
{
public:
	ColorTracker();
	ColorTracker(ColorTrackerParams params);
	~ColorTracker();

	cv::Mat thresholdImage(cv::Mat target);
	cv::Point getCoM(cv::Mat target);

	void setTrackingParameters(ColorTrackerParams params);
	ColorTrackerParams getTrackingParameters();
	

private:
	ColorTrackerParams mParameters;

};
