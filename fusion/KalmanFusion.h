#pragma once

#include "FusionStrategy.h"
#include "KalmanFilter.h"

#include "boost/optional.hpp"

class KalmanFusion : public FusionStrategy
{
public:
	KalmanFusion();

	void reset() override;

	vePose fuse(const vePose & cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion) override;

private:
	void fuseOrientation(const vePose& cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion, vePose& pose);
	void updateOrientation(const vePose& measuredPose);

	void fusePosition(const vePose& cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion, vePose& pose);
	void updatePosition(const vePose& measuredPose);

	static float64				m_pixelPerMeter;

	KalmanFilter<float64, 2, 1>  m_orientationFilter;
	KalmanFilter<float64, 3, 2>  m_positionFilter;
};