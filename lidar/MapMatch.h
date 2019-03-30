#pragma once

#include "opencv2/core.hpp"
//#include <opencv2/opencv.hpp> 
#include "../common/Helper.h"
#include "cil/cImg.h"
#include "cbl/c2DPoint.h"
#include "cbl/cRect.h"
#include"veFramework/veBaseTypes.h"
#include "QtCore/qvector.h"
#include "opencv2/core.hpp"
#include "TemplateMatch.h"


class MapMatch {

public:
	//Constructor
		MapMatch(cImg *TruthMap, cImg *LidarData, float *angleRange);
		//Pointer to ground truth map
		cImg *TruthMap;
		//Pointer to LidarVideo Frame
		cImg *LidarData;
		//Angle range to restrain searching angle
		float *angleRange;
		// Mapmatching method 1: template Matching 
		void Template_match();

		// Get result Pose
		vePose getPose();
		QVector<c3DPoint<double>> getMaxPose();
		//Get image that need to test (can be visulized in Cassandra output port "Test", Remember to set according test image in other files)
		cImg getTestImage(); 



private:
	// Test image that need to be seen in Cassandra (returned by public: getTestImage())
	cImg testImg;

	// Mat format for input data (Transformed by private: toCV())
	cv::Mat cv_map;
	cv::Mat cv_lidardata;

	// Mat format for output testImg (need to be transformed to cImg by private: tocImg())
	cv::Mat cv_testImg;

	//Resulted vePose (accessed by getPose())
	vePose result{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	QVector<c3DPoint<double>> maxPose;
	// Transform Mat in opencv to cImg in Cassandra
	cImg tocImg(cv::Mat *ipMat);

	cImg tocImgFloat(cv::Mat *ipMat);
	// Transform cImg in Cassandra to Mat in Opencv
	cv::Mat toCv(cImg *ipImg);
};