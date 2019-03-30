#include "RadarPCH.h"
#include "ConvertImgFormat.h"

Mat ConvertImgFormat::convertToCv(cImg *sourceImage)
{
	uint32 rowCount = (*sourceImage).getHeight();
	uint32 columnCount = (*sourceImage).getWidth();
	auto mat = cv::Mat(rowCount, columnCount, CV_8UC3);
	uchar* ptr;

	uint32 type = sourceImage->getImgType().getTypeId();
	// we get the type as RGB32
	// Copy the data
	for (uint32 i = 0; i < rowCount; i++)
	{
		for (uint32 j = 0; j < columnCount; j++)
		{
			ptr = (*sourceImage).getPixel<uchar>(j, i);
			mat.at<cv::Vec3b>(i, j) = cv::Vec3b(*(ptr + 2), *(ptr + 1), *ptr);
			/*mat.at<cv::Vec3f>(i, j)[0] = *ptr;
			mat.at<cv::Vec3f>(i, j)[1] = *ptr+1;
			mat.at<cv::Vec3f>(i, j)[2] = *ptr + 2;*/
		}
	}
	//imwrite("F:/TU_Berlin/study/AutonomousDriving/Data/result.jpg", mat);
	return mat;
}

cImg ConvertImgFormat::convertToCimg(Mat *sourceImage)
{
	int rowCount = (*sourceImage).rows;
	int columnCount = (*sourceImage).cols;

	cImg cimg = cImg(cImgType::ImageTypes::Float32, columnCount, rowCount);

	// Copy the data
	for (int i = 0; i < rowCount; i++)
	{
		for (int j = 0; j < columnCount; j++)
		{
			*cimg.getPixel<float32>(i, j) = (*sourceImage).at<float32>(i, j);
		}
	}
	return cimg;
}

