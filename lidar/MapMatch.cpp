#include "lidarPCH.h"
#include "MapMatch.h"

using namespace cv;

MapMatch::MapMatch(cImg *TruthMap, cImg *LidarData, float *angleRange) {
	this->LidarData = LidarData;
	this->TruthMap = TruthMap;
	this->angleRange = angleRange;
};

void MapMatch::Template_match() {

	float scale = 0.5;
	//float angleRange[2];
	//angleRange[0] = 0;
	//angleRange[1] = CV_PI * 2;


	// Transform data in cImg format to cv::Mat
	cv_lidardata = toCv(this->LidarData);
    cv_map = toCv(this->TruthMap);
	cv_testImg = Mat::zeros(cv_map.rows, cv_map.cols, cv_map.type());

	// Match
	TemplateMatch match(this->cv_map, this->cv_lidardata, cv_testImg, scale, angleRange);
	match.Match();
	
	
	cv_testImg = match.getTemplateImage().clone();
	testImg = tocImg(&cv_testImg);
	
	maxPose = match.getMaxPose();
	// Get result
	result.setX(match.getX());
	result.setY(match.getY());
	result.setYawAngle(( CV_PI *1.5) - match.getMaxScaleAngle().getY());
	/*cTracer::cout << "YAW" <<(1.5*CV_PI) - match.getMaxScaleAngle().getY()<< "3/2Pi" <<1.5 * CV_PI << "raw"<< match.getMaxScaleAngle()<< " Angle Range: "<< angleRange[0] << " to "<<angleRange[1]<< TREND;*/
}


vePose MapMatch::getPose() {
	return result;
}

QVector<c3DPoint<double>> MapMatch::getMaxPose() {
	return maxPose;
}

cImg MapMatch::getTestImage() {
	return testImg;
}


cv::Mat MapMatch::toCv(cImg *ipImg)
{
	int rowCount = (*ipImg).getHeight();
	int columnCount = (*ipImg).getWidth();

	Mat mat = Mat::zeros( rowCount, columnCount, CV_8UC1);

	// Copy the data
	for (int i = 0; i < rowCount; i++)
	{
		for (int j = 0; j < columnCount; j++)
		{
			mat.at<uint8>(i, j) = *(*ipImg).getPixel<uint8>(i, j);
		}
	}
	
	return mat;
}

cImg MapMatch::tocImg(Mat *ipMat)
{
	int rowCount = (*ipMat).rows;
	int columnCount = (*ipMat).cols;

	cImg cimg = cImg(cImgType::ImageTypes::UInt8, columnCount, rowCount);

	// Copy the data
	for (int i = 0; i < rowCount; i++)
	{
		for (int j = 0; j < columnCount; j++)
		{
			*cimg.getPixel<uint8>(i, j) = (*ipMat).at<uint8>(i, j);
		}
	}

	return cimg;
}


cImg MapMatch::tocImgFloat(Mat *ipMat)
{
	int rowCount = (*ipMat).rows;
	int columnCount = (*ipMat).cols;

	cImg cimg = cImg(cImgType::ImageTypes::Float32, columnCount, rowCount);

	// Copy the data
	for (int i = 0; i < rowCount; i++)
	{
		for (int j = 0; j < columnCount; j++)
		{
			*cimg.getPixel<float32>(i, j) = (*ipMat).at<float32>(i, j);
		}
	}

	return cimg;
}
