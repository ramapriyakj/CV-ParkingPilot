#pragma once

#include "cil/cImg.h"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp" 
#include "../common/Helper.h"

class TemplateMatch {

public:
	//Constructor
	TemplateMatch(cv::Mat image_source, cv::Mat image_template, cv::Mat image_matched, float scale, float* angleRange);

	// perform Match, result can be accessed using other public methods, e.g. getX());
	void Match();

	//get positionX
	int getX();
	//get position Y
	int getY();

	// return a vector of max and min score
	cVector<double> getMaxMinscore();

	// return a vector of max scale and max angle
	cVector<float> getMaxScaleAngle();


	QVector<c3DPoint<double>> getMaxPose();
	// Intended to get img_matched for testing (currently wird image) * TO DO: test if the img_matched is correct
	cv::Mat getMatchedImage();

	cv::Mat getTemplateImage();
private:
	// used by Match()
	cv::Mat rotateAndScale(cv::Mat& image, double angle, double scale);

	// input data for template matching
	cv::Mat img_source;
	cv::Mat img_template;
	cv::Mat img_matched;
	cv::Mat templateImage;

	// Scale Range : TO DO: only used one scale
	double scale;

	// Angle Range: TO DO: constrain angle by ego motion
	double angleRange[2];

	//Resulted position
	int positionX;
	int positionY;

	QVector<c3DPoint<double>> maxPose;

	//Resulted maxScore, minScore
	double maxScore, minScore;

	//Resulted maxScale, maxAngle
	float maxScale, maxAngle;

};