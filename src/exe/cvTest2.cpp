/**
 * \file cvTest.cpp
 *
 * \brief Tracks the center of mass of a particular color.
 * 
 * C++ implementation of tracking code presented at:
 * http://www.aishack.in/2010/07/tracking-colored-objects-in-opencv/
 *
 * NB: You may want to disable auto white balance and auto gain on your camera
 *
 * \author Andrew Price
*/

// OpenCV libraries for tracking
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <cstdio>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "HuboApplication/ColorTracker.h"


/**
 * \fn main
 * \brief Grabs frames from the default camera device, matches them against a color, and sends the x,y position of the CoM.
 * Also demonstrates boost code for reading in a settings file and sending data over sockets.
 */
int main( int argc, const char** argv )
{
	/* Tracking Code */
	ColorTracker tracker;

	// Initialize capturing live feed from the camera
	cv::VideoCapture capture;
	capture.open(CV_CAP_ANY);
	
	// Couldn't get a device? Throw an error and quit
	if(!capture.isOpened())
	{
		printf("Could not initialize capturing...\n");
		return -1;
	}
	
	// The two windows we'll be using
	cv::namedWindow("video");
	cv::namedWindow("thresh");

	// This image holds the "scribble" data...
	// the tracked positions of the target
	cv::Mat imgScribble;

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
		
		
		// If this is the first frame, we need to initialize it
		if(!imgScribble.data)
		{
			imgScribble = cv::Mat::zeros(frame.size(), CV_8UC3);
		}
		
		// Holds the red thresholded image (yellow = white, rest = black)
		cv::Point CoM = tracker.getCoM(frame);

		static int posX, posY;
		int lastX = posX;
		int lastY = posY;
		posX = CoM.x;
		posY = CoM.y;		

		// Print it out for debugging purposes
		printf("position (%d,%d)\n", posX, posY);

		// We want to draw a line only if its a valid position
		if(lastX>0 && lastY>0 && posX>0 && posY>0)
		{
			//posX = (int)SimpleFilter(posX, lastX, 1/10);
			//posY = (int)SimpleFilter(posY, lastY, 1/10);

			// Draw a yellow line from the previous point to the current point
			cv::line(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,255), 5);
		}

		// Add the scribbling image and the frame
		cv::add(frame, imgScribble, frame);
		//cv::imshow("thresh", imgRedThresh);
		cv::imshow("video", frame);

		// Wait for a keypress
		int c = cv::waitKey(10);
		if(c!=-1)
		{
			// If pressed, break out of the loop
			break;
		}

		// Release the thresholded image... we need no memory leaks.. please
		//imgRedThresh.release();
	}

	// We're done using the camera. Other applications can now use it
	capture.release();
	return 0;
}
