#include "fusionPCH.h"
#include "AverageFusion.h"

vePose AverageFusion::fuse(const vePose & cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion)
{
	vePose pose;

	// Fuse position
	std::vector<vePose> validPositionPoses;
	if (!hasInvalidPosition(cameraPose)) { validPositionPoses.push_back(cameraPose); }
	if (!hasInvalidPosition(lidarPose)) { validPositionPoses.push_back(lidarPose); }
	if (!hasInvalidPosition(radarPose)) { validPositionPoses.push_back(radarPose); }
	
	for (const vePose& p : validPositionPoses)
	{
		pose.setPosition(pose.getPosition() + p.getPosition() / static_cast<float64>(validPositionPoses.size()));
		pose.setCovPosition(pose.getCovPosition() + p.getCovPosition() / static_cast<float64>(validPositionPoses.size()));
	}

	// Fuse orientation
	std::vector<vePose> validOrientationPoses;
	if (!hasInvalidOrientation(cameraPose)) { validOrientationPoses.push_back(cameraPose); }
	if (!hasInvalidOrientation(lidarPose)) { validOrientationPoses.push_back(lidarPose); }
	if (!hasInvalidOrientation(radarPose)) { validOrientationPoses.push_back(radarPose); }

	for (const vePose& p : validOrientationPoses)
	{
		pose.setOrientation(pose.getOrientation() + p.getOrientation() / static_cast<float64>(validOrientationPoses.size()));
		pose.setCovOrientation(pose.getCovOrientation() + p.getCovOrientation() / static_cast<float64>(validOrientationPoses.size()));
	}

	return pose;
}
