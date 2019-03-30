#pragma once

#include "Decomposition.h"
#include "Helper.h"

#include "opencv2/core.hpp"

#include <random>

template<class Type = double>
class multivariate_normal_distribution
{
public:
	multivariate_normal_distribution(const cv::InputArray& _cov, const cv::InputArray& _mean)
	{
		mean = _mean.getMat();

		// Perform cholesky decomposition of the covariance matrix
		cv::Mat cov = _cov.getMat();
		dimensions = cov.rows;
		L = cholesky<Type>(cov);
	}

	template<class Engine>
	cv::Mat operator() (Engine& engine)
	{
		cv::Mat z{ dimensions, 1, cv_mat_type_trait<Type>::CV_TYPE };

		for (int i = 0; i < dimensions; ++i)
		{
			z.at<Type>(i, 0) = distribution(engine);
		}

		auto z_prime = L * z;

		return z_prime + mean;
	}

private:
	int dimensions;
	cv::Mat L;
	cv::Mat mean;
	std::normal_distribution<Type> distribution;
};
