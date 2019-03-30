#include "lidarPCH.h"
#include "TemplateMatch.h"
#include "../common/Helper.h"


using namespace cv;

TemplateMatch::TemplateMatch(cv::Mat image_source, cv::Mat image_template, cv::Mat image_matched, float scale, float* angleRange) {
	this->img_source = image_source.clone();
	this->img_template = image_template.clone();
	this->img_matched = image_matched.clone();

	this->scale = scale;
	this->angleRange[0] = angleRange[0];
	this->angleRange[1] = angleRange[1];
}

void TemplateMatch::Match() {
	//init

	int AngleStep = 73;
	int ScaleStep = 1;
	float resolutionScale = scale;
	float resolutionAngle = (angleRange[1] - angleRange[0]) / (AngleStep - 1);
	//template match
	Mat scores = Mat::zeros(ScaleStep, AngleStep, CV_32FC1);
	Mat locX = Mat::zeros(ScaleStep, AngleStep, CV_32FC1);
	Mat locY = Mat::zeros(ScaleStep, AngleStep, CV_32FC1);
	int initX = 395;
	int initY = 436;


	for (int i = 0; i < ScaleStep; i++) {
		for (int j = 0; j < AngleStep; j++) {

			float angle = angleRange[0] + resolutionAngle*j;
			Mat templateImage = rotateAndScale(img_template, angle, scale);
			//showImage( templateImage,"templateimage",0);
			matchTemplate(img_source, templateImage, img_matched, cv::TM_CCORR_NORMED);
			double minVal, maxVal;
			cv::Point minLoc, maxLoc;
			//Ñ°ÕÒ×î¼ÑÆ¥ÅäÎ»ÖÃ  

			//image_matched = image_matched(Rect(initY-20, initX - 20, 40, 40));
			cv::minMaxLoc(img_matched, &minVal, &maxVal, &minLoc, &maxLoc);
			normalize(img_matched, img_matched, 0, 255, NORM_MINMAX, -1, Mat());
			//img_matched = img_matched / maxVal * 225;
			
			scores.at<float>(i, j) = maxVal;
			locX.at<float>(i, j) = maxLoc.y;
			locY.at<float>(i, j) = maxLoc.x;
			//cTracer::cout << "X: " << maxLoc.x << " Y: " << maxLoc.y << " Score: " << maxVal <<" Angle " << angle;

		}
		//cTracer::cout << TREND;
	}

	for (int i = 0; i < 5; i++) {


		Point maxScoreLoc, minScoreLoc;
		double maxScore, minScore;


		maxScoreLoc.x = 0;
		maxScoreLoc.y = 0;
		minMaxLoc(scores, &minScore, &maxScore, &minScoreLoc, &maxScoreLoc);

		maxAngle = maxScoreLoc.x*resolutionAngle + angleRange[0];
		Mat templateImage = rotateAndScale(img_template, maxAngle, scale);

		//resize(templateImage, img_matched, img_matched.size());  //Check the orientation of the temp
		positionX = locX.at<float>(maxScoreLoc.y, maxScoreLoc.x) + templateImage.rows / 2;
		positionY = locY.at<float>(maxScoreLoc.y, maxScoreLoc.x) + templateImage.cols / 2;

		c3DPoint<double> newPose;
		newPose.setX(positionX);
		newPose.setY(positionY);
		newPose.setZ(CV_PI*1.5-maxAngle);
		//newPose.setYawAngle(maxAngle);
	//	cTracer::cout << i << ":X: " << positionX << " Y: " << positionY << " MaxScore: " << maxScore << " Angle: " << maxAngle / CV_PI * 180 << " Scale" << scale << TREND;
		maxPose.append(newPose);
	//	cTracer::cout << scores.at<float>(maxScoreLoc.y, maxScoreLoc.x) << TREND;
		scores.at<float>(maxScoreLoc.y, maxScoreLoc.x) = -1.0;

		/*	for (int p = 0; p < scores.cols; p++) {
		cTracer::cout << scores.at<float>(0, p) << TREND;
		}*/
	//	cTracer::cout << scores.at<float>(maxScoreLoc.y, maxScoreLoc.x) << TREND;
	}

	Point maxScoreLoc, minScoreLoc;
	minMaxLoc(scores, &minScore, &maxScore, &minScoreLoc, &maxScoreLoc);
	maxAngle = maxScoreLoc.x*resolutionAngle + angleRange[0];
	 templateImage = rotateAndScale(img_template, maxAngle, scale);

	//resize(templateImage, img_matched, img_matched.size());  //Check the orientation of the temp
	positionX = locX.at<float>(maxScoreLoc.y, maxScoreLoc.x) + templateImage.rows / 2;
	positionY = locY.at<float>(maxScoreLoc.y, maxScoreLoc.x) + templateImage.cols / 2;

	cTracer::cout << "X: " << positionX << " Y: " << positionY << " MaxScore: " << maxScore << " Angle: " << maxAngle / CV_PI * 180 << " Scale" << scale << TREND;
}

Mat TemplateMatch::rotateAndScale(Mat& image, double angle, double scale) {

	// create transformation matrices
	// translation to origin
	Mat T = Mat::eye(3, 3, CV_32FC1);
	T.at<float>(0, 2) = -image.cols / 2.0;
	T.at<float>(1, 2) = -image.rows / 2.0;
	// rotation
	Mat R = Mat::eye(3, 3, CV_32FC1);
	R.at<float>(0, 0) = cos(angle);
	R.at<float>(0, 1) = -sin(angle);
	R.at<float>(1, 0) = sin(angle);
	R.at<float>(1, 1) = cos(angle);
	//cout << R << endl;
	// scale
	Mat S = Mat::eye(3, 3, CV_32FC1);
	S.at<float>(0, 0) = scale;
	S.at<float>(1, 1) = scale;
	// combine
	Mat H = R*S*T;

	// compute corners of warped image
	Mat corners(1, 4, CV_32FC2);
	corners.at<Vec2f>(0, 0) = Vec2f(0, 0);
	corners.at<Vec2f>(0, 1) = Vec2f(0, image.rows);
	corners.at<Vec2f>(0, 2) = Vec2f(image.cols, 0);
	corners.at<Vec2f>(0, 3) = Vec2f(image.cols, image.rows);
	perspectiveTransform(corners, corners, H);

	// compute size of resulting image and allocate memory
	volatile float x_start = min(min(corners.at<Vec2f>(0, 0)[0], corners.at<Vec2f>(0, 1)[0]), min(corners.at<Vec2f>(0, 2)[0], corners.at<Vec2f>(0, 3)[0]));
	volatile float x_end = max(max(corners.at<Vec2f>(0, 0)[0], corners.at<Vec2f>(0, 1)[0]), max(corners.at<Vec2f>(0, 2)[0], corners.at<Vec2f>(0, 3)[0]));
	volatile float y_start = min(min(corners.at<Vec2f>(0, 0)[1], corners.at<Vec2f>(0, 1)[1]), min(corners.at<Vec2f>(0, 2)[1], corners.at<Vec2f>(0, 3)[1]));
	volatile float y_end = max(max(corners.at<Vec2f>(0, 0)[1], corners.at<Vec2f>(0, 1)[1]), max(corners.at<Vec2f>(0, 2)[1], corners.at<Vec2f>(0, 3)[1]));

	// create translation matrix in order to copy new object to image center
	T.at<float>(0, 0) = 1;
	T.at<float>(1, 1) = 1;
	T.at<float>(2, 2) = 1;
	T.at<float>(0, 2) = (x_end - x_start + 1) / 2.0;
	T.at<float>(1, 2) = (y_end - y_start + 1) / 2.0;

	// change homography to take necessary translation into account
	H = T * H;
	// warp image and copy it to output image
	Mat output;
	warpPerspective(image, output, H, Size(x_end - x_start + 1, y_end - y_start + 1), CV_INTER_LINEAR);

	std::vector<Mat> output1;
	split(output, output1);
	//write("output.jpg", output1.at(0));

	return output;

}


QVector<c3DPoint<double>> TemplateMatch::getMaxPose() {
	return maxPose;
}
int TemplateMatch::getX() {
	return this->positionX;
}

int TemplateMatch::getY() {
	return this->positionY;
}

cVector<double> TemplateMatch::getMaxMinscore() {
	cVector<double> MaxMinscore = cVector<double>(2);
	MaxMinscore.setX(maxScore);
	MaxMinscore.setY(minScore);
	return MaxMinscore;
}
cVector<float> TemplateMatch::getMaxScaleAngle() {
	cVector<float> MaxScaleAngle = cVector<float>(2);
	MaxScaleAngle.setX(maxScale);
	MaxScaleAngle.setY(maxAngle);
	return MaxScaleAngle;
}

Mat TemplateMatch::getMatchedImage() {
	return img_matched;
}

Mat TemplateMatch::getTemplateImage() {
	return templateImage;
}