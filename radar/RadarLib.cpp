#include "RadarPCH.h"
#include "RadarLib.h"

#include <fstream>

int RadarFrame::rotationFrequency;
int RadarFrame::rotationDegree;
int RadarFrame::maxBestFramesMatch;
Mat RadarFrame::groundTruthMap;
string RadarFrame::bestRotatedFramesFile;


Radar::Radar(Mat sourceFrame, int frameNumber)
{
	this->radarFrame.frame = sourceFrame;
	this->radarFrame.frameNumber = frameNumber;
	this->radarFrame.baseCarLocation.x = sourceFrame.cols / 2;
	this->radarFrame.baseCarLocation.y = sourceFrame.rows / 2;
	handleFrame();
	// Call this test function to store data of best rotated frames in file
	// writeBestRotatedframesFile();
	// Call this test function to store matched image
	//saveBestRotatedFrames();

}

void Radar::setStaticMembers(Mat groTruthMap, int rotFrequency, int rotDegree, int maxBestFrames, string rotatedFramesFile = "")
{
	RadarFrame::groundTruthMap = groTruthMap;
	RadarFrame::bestRotatedFramesFile = rotatedFramesFile;
	RadarFrame::rotationFrequency = rotFrequency;
	RadarFrame::rotationDegree = rotDegree;
	RadarFrame::maxBestFramesMatch = maxBestFrames;
}

void Radar::handleFrame()
{
	filterRedColor();
	contourFrame();
	resizeFrame();
	for (int i = 0; i < RadarFrame::rotationFrequency; i++)
	{
		RadarFrameRotation radarFrameRotation = rotateFrame(i*RadarFrame::rotationDegree);
		radarFrameRotation = matchTemplateFrame(radarFrameRotation);
		populateBestRotatedFrames(radarFrameRotation);
	}
}

void Radar::filterRedColor()
{
	if (!radarFrame.frame.empty())
	{
		Mat mask, mask_a, mask_b, hsv, result;
		cvtColor(radarFrame.frame, hsv, CV_BGR2HSV);
		inRange(hsv, Scalar(0, 50, 50), Scalar(10, 255, 255), mask_a);
		inRange(hsv, Scalar(170, 50, 50), Scalar(100, 255, 255), mask_b);
		mask = mask_a | mask_b;
		bitwise_and(radarFrame.frame, radarFrame.frame, result, mask = mask);
		radarFrame.frame = result;
	}
}

void Radar::contourFrame()
{
	if (!radarFrame.frame.empty())
	{
		cvtColor(radarFrame.frame, radarFrame.frame, CV_RGB2GRAY);
		Canny(radarFrame.frame, radarFrame.frame, 100, 200);
		cvtColor(radarFrame.frame, radarFrame.frame, CV_GRAY2RGB);
	}
}

void Radar::resizeFrame()
{
	int width = radarFrame.frame.cols;
	int height = radarFrame.frame.rows;
	int groundTruthMapCols = RadarFrame::groundTruthMap.cols;
	int groundTruthMapRows = RadarFrame::groundTruthMap.rows;
	int minX = width;
	int minY = height;
	int maxX = 0;
	int maxY = 0;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	Mat src = radarFrame.frame;
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			r = src.at<cv::Vec3b>(y, x)[0];
			g = src.at<cv::Vec3b>(y, x)[1];
			b = src.at<cv::Vec3b>(y, x)[2];

			if (r > 0 || g > 0 || b > 0)
			{
				minX = min(minX, x);
				minY = min(minY, y);
				maxX = max(maxX, x);
				maxY = max(maxY, y);
			}
		}
	}
	radarFrame.resizeCarLocation = Point(radarFrame.baseCarLocation.x - minX, radarFrame.baseCarLocation.y - minY);
	if (radarFrame.resizeCarLocation.x >= 0 && radarFrame.resizeCarLocation.y >= 0)
	{
		Rect r = Rect(minX, minY, maxX - minX, maxY - minY);
		radarFrame.frame = radarFrame.frame(r);
		if (radarFrame.frame.cols > groundTruthMapCols || radarFrame.frame.rows > groundTruthMapRows)
		{
			double factorX = (double)groundTruthMapCols / radarFrame.frame.cols;
			double factorY = (double)groundTruthMapRows / radarFrame.frame.rows;
			radarFrame.resizeCarLocation = Point(round(factorX*radarFrame.resizeCarLocation.x), round(factorY*radarFrame.resizeCarLocation.y));
			resize(radarFrame.frame, radarFrame.frame, Size(groundTruthMapCols, groundTruthMapRows), 0, 0, INTER_AREA);
		}
	}
	else
	{
		radarFrame.resizeCarLocation = Point(groundTruthMapCols / 2, groundTruthMapRows / 2);
		Rect r = Rect(0, 0, groundTruthMapCols, groundTruthMapRows);
		radarFrame.frame = radarFrame.frame(r);
	}
}


RadarFrameRotation Radar::rotateFrame(int RotationAngle)
{
	Mat M, trans;
	int cols = radarFrame.frame.cols;
	int rows = radarFrame.frame.rows;

	M = getRotationMatrix2D(Point(cols / 2, rows / 2), RotationAngle, 1);
	warpAffine(radarFrame.frame, trans, M, Size(cols, rows));

	vector<Point> orgv, newv;
	orgv.push_back(radarFrame.resizeCarLocation);
	cv::transform(orgv, newv, M);

	trans = plotPoint(trans, newv[0]);

	RadarFrameRotation radarFrameRotation;
	radarFrameRotation.angle = RotationAngle;
	radarFrameRotation.carLocation = newv[0];
	radarFrameRotation.rotatedFrame = trans;

	return radarFrameRotation;
}


RadarFrameRotation Radar::matchTemplateFrame(RadarFrameRotation rotatedFrame)
{
	Mat result, sourceImage, templateImage;
	RadarFrame::groundTruthMap.copyTo(sourceImage);
	rotatedFrame.rotatedFrame.copyTo(templateImage);

	//0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED
	int match_method = 4;

	if (sourceImage.empty() || templateImage.empty())
	{
		//cout << "Cant read images!!";
		return rotatedFrame;
	}

	/// Create the result matrix
	int result_cols = sourceImage.cols - templateImage.cols + 1;
	int result_rows = sourceImage.rows - templateImage.rows + 1;

	result.create(result_rows, result_cols, CV_32FC1);

	/// Do the Matching and Normalize
	matchTemplate(sourceImage, templateImage, result, match_method);

	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

	/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
	if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
	{
		matchLoc = minLoc;
	}
	else
	{
		matchLoc = maxLoc;
	}

	Point point = Point(matchLoc.x + rotatedFrame.carLocation.x, matchLoc.y + rotatedFrame.carLocation.y);
	sourceImage = plotPoint(sourceImage, point);

	rotatedFrame.carLocation = point;
	rotatedFrame.intensity = round(maxVal * 1000.0) / 1000.0;
	rotatedFrame.matchedFrame = sourceImage;

	return  rotatedFrame;
}

void Radar::populateBestRotatedFrames(RadarFrameRotation rotatedFrame)
{
	radarFrame.bestRotatedFrames.push_back(rotatedFrame);
	std::sort(radarFrame.bestRotatedFrames.begin(), radarFrame.bestRotatedFrames.end());
	if (radarFrame.bestRotatedFrames.size() > RadarFrame::maxBestFramesMatch)
	{
		radarFrame.bestRotatedFrames.erase(radarFrame.bestRotatedFrames.end() - 1);
	}
}

Point Radar::getCarPosition()
{
	// send one best position, if array is needed, send a vector<point> with n bestRotatedFrames positions
	return radarFrame.bestRotatedFrames[0].carLocation;
}

float Radar::getOrientation()
{
	// send one best angle, if array is needed, send a vector<point> with n bestRotatedFrames angles
	return radarFrame.bestRotatedFrames[0].angle *(CV_PI / 180);
}

Mat Radar::plotPoint(Mat sourceFrame, Point point)
{
	if (!sourceFrame.empty())
	{
		line(sourceFrame, point, point, Scalar(0, 0, 255), 10, 8, 0);
	}
	return sourceFrame;
}
string Radar::createBestRotatedframesFile(string fileLocation)
{
	char filename[5000];
	if (strcmp(fileLocation.c_str(), "") != 0)
	{
		strcpy_s(filename, fileLocation.c_str());
		strcat_s(filename, "BestFrameRotations.txt");
		ofstream outfile(filename);
		outfile << "FrameNumber,CarLocationX,carLocationY,FrameRotationAngle,MatchIntensity" << endl;
		outfile.close();
	}
	return filename;
}

void Radar::writeBestRotatedframesFile()
{
	if (strcmp(RadarFrame::bestRotatedFramesFile.c_str(), "") != 0)
	{
		ofstream outfile;
		outfile.open(RadarFrame::bestRotatedFramesFile, ios_base::app);
		for (int i = 0; i < radarFrame.bestRotatedFrames.size(); i++)
		{
			outfile << radarFrame.frameNumber << "," << radarFrame.bestRotatedFrames[i].carLocation.x << "," << radarFrame.bestRotatedFrames[i].carLocation.y << "," << radarFrame.bestRotatedFrames[i].angle << "," << radarFrame.bestRotatedFrames[i].intensity << endl;
		}
		outfile.close();
	}
}

void Radar::saveBestRotatedFrames()
{
	string resultFrame = "F:/TU_Berlin/study/AutonomousDriving/Data/output/RadarFrames/radar.jpg";
	string matchedFrame = "F:/TU_Berlin/study/AutonomousDriving/Data/output/Results/gt.jpg";
	if (strcmp(resultFrame.c_str(), "") != 0)
	{
		//imwrite(resultFrame, radarFrame.bestRotatedFrames[0].rotatedFrame);
	}
	if (strcmp(matchedFrame.c_str(), "") != 0)
	{
		//imwrite(matchedFrame, radarFrame.bestRotatedFrames[0].matchedFrame);
	}
}


