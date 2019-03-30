#pragma once

#include "opencv2/core.hpp"
#include "veFramework/veBaseTypes.h"

class Noise
{
public:
	virtual ~Noise() = default;

	virtual vePose sample(float64 t, const vePose& pose) = 0;

protected:
	cv::Mat makeNoisy(cv::Mat mean, cv::Mat cov);
};
