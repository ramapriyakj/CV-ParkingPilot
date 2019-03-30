#pragma once

#include "../common/Helper.h"
#include "../common/MultiVariateNormalDistribution.h"

#include "opencv2/core.hpp"

#include <random>

template<typename Type, int HiddenDims, int OberservedDims>
class KalmanFilter
{
public:
	KalmanFilter(const cv::Mat& x,
				 const cv::Mat& P,
				 const cv::Mat& Hk)
		: m_xInitial(x.clone())
		, m_PInitial(P.clone())
		, m_HkInitial(Hk.clone())
		, m_x(x.clone())
		, m_P(P.clone())
		, m_Hk(Hk.clone())
	{
		assert(x.rows == HiddenDims && x.cols == 1);
		assert(P.rows == HiddenDims && P.cols == HiddenDims);
		assert(Hk.rows == OberservedDims && Hk.cols == HiddenDims);
	}

	void reset()
	{
		m_x = m_xInitial.clone();
		m_P = m_PInitial.clone();
		m_Hk = m_HkInitial.clone();
	}

	void predict(const cv::Mat& Fk,
				 const cv::Mat& Qk)
	{
		assert(Fk.rows == HiddenDims && Fk.cols == HiddenDims);
		assert(Qk.rows == HiddenDims && Qk.cols == HiddenDims);


		// Draw wk from a gaussian distribution
		std::default_random_engine engine(std::random_device{}());
		multivariate_normal_distribution<double> distrib(Qk, cv::Mat::zeros(HiddenDims, 1, cv_mat_type_trait<Type>::CV_TYPE));
		cv::Mat wk = distrib(engine);

		m_x = Fk * m_x /*+ wk*/;
		m_P = Fk * m_P * Fk.t() + Qk;
	}

	void update(const cv::Mat& zk,
				const cv::Mat& Rk)
	{
		assert(zk.rows == OberservedDims && zk.cols == 1);
		assert(Rk.rows == OberservedDims && Rk.cols == OberservedDims);

		cv::Mat yk = zk - m_Hk * m_x;
		cv::Mat Sk = Rk + m_Hk * m_P * m_Hk.t();
		cv::Mat Kk = m_P * m_Hk.t() * Sk.inv();
		m_x = m_x + Kk * yk;
		m_P = m_P - Kk * m_Hk * m_P;
	}

	inline cv::Mat getX() const
	{
		return m_x;
	}

	inline cv::Mat getP() const
	{
		return m_P;
	}

private:
	cv::Mat m_xInitial;
	cv::Mat m_PInitial;
	cv::Mat m_HkInitial;
	cv::Mat m_x;
	cv::Mat m_P;
	cv::Mat m_Hk;
};
