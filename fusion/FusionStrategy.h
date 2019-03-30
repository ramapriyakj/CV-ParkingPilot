#pragma once

#include "veFramework/veBaseTypes.h"

class FusionStrategy
{
public:
	virtual ~FusionStrategy() = default;

	virtual void reset() = 0;

	virtual vePose fuse(const vePose& cameraPose, 
						const vePose& lidarPose, 
						const vePose& radarPose, 
						const veEgomotion& egomotion) = 0;

	inline bool hasInvalidPosition(const vePose& pose)
	{
		return pose.getPosition()[2] < 0.0;
	}

	inline bool hasInvalidOrientation(const vePose& pose)
	{
		return pose.getOrientation()[0] < 0.0;
	}
};