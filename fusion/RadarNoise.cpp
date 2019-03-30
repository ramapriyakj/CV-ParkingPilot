#include "fusionPCH.h"
#include "RadarNoise.h"
#include "Interpolators.h"

#include "../common/Helper.h"
#include "../common/MultiVariateNormalDistribution.h"

#include <random>

RadarNoise::RadarNoise()
	: m_covPosition(3, 3, { 20, 0, 0, 0, 40, 0, 0, 0, 0 })
	, m_covOrientation(3, 3, { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1 })
{
}

vePose RadarNoise::sample(float64 t, const vePose& pose)
{
	cDblMatrix initialCovPosition(3, 3, { 400, 0, 0, 0, 400, 0, 0, 0, 0 });

	cDblMatrix covPosition = m_covPosition;
	if (t < 100.0)
	{
		auto interpolator = createLinear<cDblMatrix>();
		covPosition = (*interpolator)(initialCovPosition, m_covPosition, t, 100.0);
	}

	cv::Mat pos = makeNoisy(toCv(pose.getPosition())(cv::Rect{ 0, 0, 1, 2 }), toCv(covPosition)(cv::Rect{ 0, 0, 2, 2 }));
	cv::Mat orientation = makeNoisy(toCv(pose.getOrientation())(cv::Rect{ 0, 2, 1, 1 }), toCv(m_covOrientation)(cv::Rect{ 2, 2, 1, 1 }));

	return vePose{ cVector<float64>(pos.at<float64>(0), pos.at<float64>(1), 0.0), covPosition, cDblVector(0.0, 0.0, orientation.at<float64>(0)), m_covOrientation, cBufferStamp() };
}