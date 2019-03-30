#pragma once

#include "veFramework/veBaseTypes.h"

#include "opencv2/core.hpp"

constexpr const double METER_PER_PIXEL = 0.2;

constexpr const double PIXEL_PER_METER = 5.0;

constexpr const double PI = 3.14159265358979323846;

template<class Type>
struct cv_mat_type_trait {};

template<>
struct cv_mat_type_trait<double>
{
	static const int CV_TYPE = CV_64FC1;
};

template<>
struct cv_mat_type_trait<float>
{
	static const int CV_TYPE = CV_32FC1;
};

template<class Type>
cv::Mat toCv(const cMatrix<Type>& matrix)
{
	int rowCount = static_cast<int>(matrix.getRowCount());
	int columnCount = static_cast<int>(matrix.getColumnCount());

	cv::Mat mat{ rowCount, columnCount, cv_mat_type_trait<Type>::CV_TYPE };

	// Copy the data
	for (int i = 0; i < rowCount; i++)
	{
		for (int j = 0; j < columnCount; j++)
		{
			mat.at<Type>(i, j) = matrix(i, j);
		}
	}
	
	return mat;
}

template<class Type>
cv::Mat toCv(const cVector<Type>& vector)
{
	cv::Mat vec{ static_cast<int>(vector.getSize()), 1, cv_mat_type_trait<Type>::CV_TYPE };

	// Copy the data
	for (int i = 0; i < vec.rows; i++)
	{
		vec.at<Type>(i) = vector[i];
	}

	return vec;
}

