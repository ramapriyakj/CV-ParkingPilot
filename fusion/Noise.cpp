#include "fusionPCH.h"
#include "Noise.h"

#include "../common/Helper.h"
#include "../common/MultiVariateNormalDistribution.h"

cv::Mat Noise::makeNoisy(cv::Mat mean, cv::Mat cov)
{
	std::mt19937 engine;
	engine.seed(std::random_device{}());
	multivariate_normal_distribution<float64> distribution(cov, mean);

	return distribution(engine);
}
