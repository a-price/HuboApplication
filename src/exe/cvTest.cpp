/**
 * \file cvTest.cpp
 *
 * \brief Tracks the center of mass of a particular color and
 * sends its movements over sockets to Hubo.
 * 
 * C++ implementation of tracking code presented at:
 * http://www.aishack.in/2010/07/tracking-colored-objects-in-opencv/
 *
 * NB: You may want to disable auto white balance and auto gain on your camera
 *
 * \author Andrew Price
*/

// Boost libraries for sockets and config
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

// OpenCV libraries for tracking
//#include "opencv/cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

using boost::asio::ip::tcp;

//#define USE_SOCKETS

/// Thresholding Function
cv::Mat GetThresholdedImage(cv::Mat img)
{
	// Convert the image into an HSV image
	cv::Mat imgHSV;
	cv::cvtColor(img, imgHSV, CV_BGR2HSV);
	
	// Stores the matched color pixels
	cv::Mat imgThreshed;

	// Filter the image
	cv::GaussianBlur(imgHSV,imgHSV, cv::Size(5,5), 5);
	//cv::bilateralFilter(img,imgBilat, 10, 5, 3);

	// Threshold the HSV image
	cv::inRange(imgHSV, cv::Scalar(160,65,60), cv::Scalar(190,256,256), imgThreshed);

	imgHSV.release();

	return imgThreshed;
}


void ParseSocketCommand(char* cmd, int length, double* x, double* y)
{
	int startX, endX, startY, endY;
	for(int i = 0; i < length; i++)
	{
		switch (cmd[i])
		{
		case '<':
			startX = i + 1;
			break;
		case ',':
			endX = i - 1;
			startY = i + 1;
			break;
		case '>':
			endY = i - 1;
			break;
		default:
			break;
		}
	}
	*x = atof(cmd + startX);
	*y = atof(cmd + startY);
}

double SimpleFilter(double xCurrent, double xPrevious, double timeStep, double pctConfidence = 0.65)
{
	return pctConfidence * xCurrent + (1 - pctConfidence) * (xPrevious + (xCurrent - xPrevious) * timeStep);
}

/**
 * \fn main
 * \brief Grabs frames from the default camera device, matches them against a color, and sends the x,y position of the CoM.
 * Also demonstrates boost code for reading in a settings file and sending data over sockets.
 */
int main( int argc, const char** argv )
{
	// Random Tests
	//double x,y;
	//char* test = "<12.34,45.67>";
	//ParseSocketCommand(test, std::strlen(test), &x, &y);
	//std::cout << "Result: " << x << "," << y << "\n";
	std::cout << Eigen::Isometry3d::Identity().matrix() << std::endl;

	boost::property_tree::ptree properties;
	boost::property_tree::read_xml("Settings.xml", properties);

	/* Sockets code */
	bool endpointConnected = false;
	size_t req_len;
	std::string host = properties.get<std::string>("NetworkSettings.IPAddress");
	std::string port = properties.get<std::string>("NetworkSettings.Port");

	boost::asio::io_service io_svc;
	
	tcp::resolver resolver(io_svc);
	tcp::resolver::query query(host, port); //("192.168.1.245", "12345");
	tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
	tcp::resolver::iterator end;

	tcp::socket socket(io_svc);
	boost::system::error_code error = boost::asio::error::host_not_found;
	while (error && endpoint_iterator != end)
	{
		std::cout << "Trying Endpoint...\n";
		socket.close();
		socket.connect(*endpoint_iterator++, error);
	}
	if (error)
	{
		//throw boost::system::system_error(error);
	}
	else
	{
		endpointConnected = true;
		std::cout << "Connected to endpoint.";
	}
	

	/* Tracking Code */

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
		cv::Mat imgRedThresh = GetThresholdedImage(frame);

		// Calculate the moments to estimate the position of the ball
		cv::Moments moments = cv::moments(imgRedThresh, 1);
		
		// The actual moment values
		double moment10 = moments.m10; //cvGetSpatialMoment(moments, 1, 0);
		double moment01 = moments.m01; //cvGetSpatialMoment(moments, 0, 1);
		double area = moments.m00;     //cvGetCentralMoment(moments, 0, 0);
		if (area == 0)
		{
			area = 1;
		}

		// Holding the last and current ball positions
		static int posX = 0;
		static int posY = 0;

		int lastX = posX;
		int lastY = posY;
		
		posX = (int)(moment10/area);
		posY = (int)(moment01/area);

		// Print it out for debugging purposes
		printf("position (%d,%d)\n", posX, posY);

		// We want to draw a line only if its a valid position
		if(lastX>0 && lastY>0 && posX>0 && posY>0)
		{
			posX = (int)SimpleFilter(posX, lastX, 1/10);
			posY = (int)SimpleFilter(posY, lastY, 1/10);

			// Draw a yellow line from the previous point to the current point
			cv::line(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,255), 5);
		}

		if (endpointConnected)
		{
			char request[1024];
			char response[1024];
			float diffX = (float)(lastX - posX)*0.002;
			float diffY = (float)(lastY - posY)*0.002;

			//if (abs(diffX) > 0.05)
			//	diffX = 0;
			//if (abs(diffY) > 0.05)
			//	diffY = 0;

			sprintf(request, "<%f,%f>", diffX, diffY);
			req_len = std::strlen(request);
		
			try
			{
				std::cout << "request: " << request << std::endl;
				socket.write_some(boost::asio::buffer(request, req_len));
				socket.read_some(boost::asio::buffer(response, socket.available()));
				std::cout << "response: " << response << std::endl;
			}
			catch (std::exception& e)
			{
				std::cerr << e.what() << std::endl;
			}
		}

		// Add the scribbling image and the frame
		cv::add(frame, imgScribble, frame);
		cv::imshow("thresh", imgRedThresh);
		cv::imshow("video", frame);

		// Wait for a keypress
		int c = cv::waitKey(10);
		if(c!=-1)
		{
			// If pressed, break out of the loop
			break;
		}

		// Release the thresholded image... we need no memory leaks.. please
		imgRedThresh.release();

		//delete moments;

	}

	// We're done using the camera. Other applications can now use it
	capture.release();
	return 0;
}
