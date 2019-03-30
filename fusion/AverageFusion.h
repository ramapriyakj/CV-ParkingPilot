#pragma once

#include "FusionStrategy.h"

class AverageFusion : public FusionStrategy
{
public:
	AverageFusion() = default;

	inline void reset() override {};

	vePose fuse(const vePose & cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion) override;
};