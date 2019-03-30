//#include "string"
//#include <iostream>
//#include "cil/cImgConvertFunctor.h"
//#include <cil/cImgType.h>
#include <opencv2/opencv.hpp>
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <stdio.h>
#include "cil/cImg.h"

using namespace std;
using namespace cv;

class ConvertImgFormat
{
	public:
		Mat convertToCv( cImg *sourceImage);
		cImg convertToCimg(Mat *sourceImage);
};