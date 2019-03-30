#pragma once
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

// Class to store the data of rotated radar frame
class RadarFrameRotation
{
public:
	int angle;
	float intensity;
	Point carLocation;
	Mat rotatedFrame;
	Mat matchedFrame;

	bool operator<(const RadarFrameRotation& obj)
	{
		return this->intensity > obj.intensity;
	}
};

// Class to store specifications of one radar  frame
class RadarFrame
{
public:
	static int rotationFrequency;
	static int rotationDegree;
	static int maxBestFramesMatch;
	static Mat groundTruthMap;
	static string bestRotatedFramesFile;

	Mat frame;
	int frameNumber;
	Point baseCarLocation;
	Point resizeCarLocation;
	vector<RadarFrameRotation> bestRotatedFrames;
};

// Class containing methods to process Radar frame and match with ground truth map
class Radar
{
public:

	// object of RadarFrame class
	RadarFrame radarFrame;

	// Constructor
	Radar(Mat sourceFrame, int frameNumber);

	// Function to set data
	static void setStaticMembers(Mat groTruthMap, int rotFrequency, int rotDegree, int maxBestFrames, string rotatedFramesFile);

	// Starting point of radar fram processing
	void handleFrame();

	// Function to filter radar data
	void filterRedColor();

	// Function to contour filtered frame
	void contourFrame();

	// Function to resize the radar frame
	void resizeFrame();

	// Function to rotate frame n times, store rotated frames and calculate car location on each rotated frame
	RadarFrameRotation rotateFrame(int RotationAngle);

	// Function to match rotated radar frame with groundtruth map
	RadarFrameRotation matchTemplateFrame(RadarFrameRotation rotatedFrame);

	// Function to store best matched rotated frames 
	void populateBestRotatedFrames(RadarFrameRotation rotatedFrame);

	// Function to return the car's location on groundtruth map
	Point getCarPosition();

	// Function to return the angle of car
	float getOrientation();

	// Test Function to plot point on image to see the car location
	static Mat plotPoint(Mat sourceFrame, Point point);

	// Test function to create file for storing data of best selected rotated frames
	static string createBestRotatedframesFile(string fileLocation);

	// Test function to see data after matching best rotated frames
	void writeBestRotatedframesFile();

	// Test function to store final images after matching. 
	void saveBestRotatedFrames();
	};

