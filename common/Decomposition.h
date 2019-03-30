#pragma once

#include "opencv2/core.hpp"

/*
 * Perform cholesky decomposition so that A = L * L^T where L is the return value
 */
template<class Type>
cv::Mat cholesky(cv::InputArray A)
{
	assert(!A.empty());
	cv::Mat _A = A.getMat();

	cv::Mat L = cv::Mat::zeros(_A.size(), _A.type());

	for (int i = 0; i < L.rows; i++)
	{
		for (int j = 0; j < (i + 1); j++)
		{
			Type s = 0;
			for (int k = 0; k < j; k++)
			{
				s += L.at<Type>(i, k) * L.at<Type>(j, k);
			}
			L.at<Type>(i, j) = (i == j) ?
				sqrt(_A.at<Type>(i, i) - s) :
				(1.0 / L.at<Type>(j, j) * (_A.at<Type>(i, j) - s));
		}
	}

	return L;
}
