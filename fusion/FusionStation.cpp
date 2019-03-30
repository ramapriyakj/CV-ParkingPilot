#include "fusionPCH.h"
#include "FusionStation.h"
#include "AverageFusion.h"
#include "KalmanFusion.h"
//-------------------------------------------------------------------------------------------------
FusionStation::FusionStation()
	: cStation("")
	, m_ipMap(this, &FusionStation::mapCallback, "map")
	, m_ipData(this, &FusionStation::dataCallback, "egomotion", "cam_pose", "lidar_pose", "radar_pose")
	, m_opMap(this, "map")
	, m_opPose(this, "pose")
	, m_fusionStrategy(std::make_unique<KalmanFusion>())
	, m_lidarPositionErrorScale(1.0)
	, m_radarPositionErrorScale(1.0)
	, m_cameraPositionErrorScale(1.0)
	, m_lidarOrientationErrorScale(1.0)
	, m_radarOrientationErrorScale(1.0)
	, m_cameraOrientationErrorScale(1.0)
{
	m_ipData.getParams<0>().required = true;
	m_ipData.getParams<1>().required = false;
	m_ipData.getParams<2>().required = false;
	m_ipData.getParams<3>().required = false;

	m_ipMap.registerOutPorts(QList<cOPortBase*>());
	m_ipData.registerOutPorts(QList<cOPortBase*>({ &m_opMap, &m_opPose }));

	m_param["lidar_position_error_scale"].setRefData(m_lidarPositionErrorScale);
	m_param["radar_position_error_scale"].setRefData(m_radarPositionErrorScale);
	m_param["camera_position_error_scale"].setRefData(m_cameraPositionErrorScale);
	m_param["lidar_orientation_error_scale"].setRefData(m_lidarOrientationErrorScale);
	m_param["radar_orientation_error_scale"].setRefData(m_radarOrientationErrorScale);
	m_param["camera_orientation_error_scale"].setRefData(m_cameraOrientationErrorScale);
}
//-------------------------------------------------------------------------------------------------
FusionStation::~FusionStation()
{
}
//-------------------------------------------------------------------------------------------------
void FusionStation::mapCallback(cBufferT<cImg>* i_buffer)
{
	m_map = i_buffer->getData();
}
void FusionStation::dataCallback()
{
	if (m_map.isValid())
	{
		// Get all available buffers
		auto egomoBuffers = m_ipData.getBuffers<0>();
		auto camerBuffers = m_ipData.getBuffers<1>();
		auto lidarBuffers = m_ipData.getBuffers<2>();
		auto radarBuffers = m_ipData.getBuffers<3>();

		sint64 stamp;
		vePose pose;
		while (!(egomoBuffers.empty() && camerBuffers.empty() && lidarBuffers.empty() && radarBuffers.empty()))
		{
			stamp = -1;

			// Find oldest timestamp
			if (!egomoBuffers.empty()) { stamp = stamp < 0 ? egomoBuffers[0]->getStamp().getStamp() : std::min(stamp, egomoBuffers[0]->getStamp().getStamp()); }
			if (!camerBuffers.empty()) { stamp = stamp < 0 ? camerBuffers[0]->getStamp().getStamp() : std::min(stamp, camerBuffers[0]->getStamp().getStamp()); }
			if (!lidarBuffers.empty()) { stamp = stamp < 0 ? lidarBuffers[0]->getStamp().getStamp() : std::min(stamp, lidarBuffers[0]->getStamp().getStamp()); }
			if (!radarBuffers.empty()) { stamp = stamp < 0 ? radarBuffers[0]->getStamp().getStamp() : std::min(stamp, radarBuffers[0]->getStamp().getStamp()); }

			// Create the poses for fusion
			veEgomotion egomotion;
			egomotion.setDeltaTime(-1.0);
			vePose cameraPose{ -1,-1,-1, -10,-10,-10 };
			vePose lidarPose{ -1,-1,-1, -10,-10,-10 };
			vePose radarPose{ -1,-1,-1, -10,-10,-10 };

			if (!egomoBuffers.empty() && egomoBuffers[0]->getStamp().getStamp() == stamp) { egomotion = egomoBuffers[0]->getData(); egomoBuffers.removeFirst(); }
			if (!camerBuffers.empty() && camerBuffers[0]->getStamp().getStamp() == stamp) { cameraPose = camerBuffers[0]->getData(); camerBuffers.removeFirst(); }
			if (!lidarBuffers.empty() && lidarBuffers[0]->getStamp().getStamp() == stamp) { lidarPose = lidarBuffers[0]->getData(); lidarBuffers.removeFirst(); }
			if (!radarBuffers.empty() && radarBuffers[0]->getStamp().getStamp() == stamp) { radarPose = radarBuffers[0]->getData(); radarBuffers.removeFirst(); }
		
			// Scale the errors (since we don't fully trust the self evaluation of the sensors)
			cameraPose.setCovPosition(m_cameraPositionErrorScale * cameraPose.getCovPosition());
			cameraPose.setCovOrientation(m_cameraOrientationErrorScale * cameraPose.getCovOrientation());
			lidarPose.setCovPosition(m_lidarPositionErrorScale * lidarPose.getCovPosition());
			lidarPose.setCovOrientation(m_lidarOrientationErrorScale * lidarPose.getCovOrientation());
			radarPose.setCovPosition(m_radarPositionErrorScale * radarPose.getCovPosition());
			radarPose.setCovOrientation(m_radarOrientationErrorScale * radarPose.getCovOrientation());

			pose = m_fusionStrategy->fuse(cameraPose, lidarPose, radarPose, egomotion);
		}

		pose.setStamp(m_ipData.getStamp());
		m_opPose.send(pose, m_ipData.getStamp());
		m_opMap.send(m_map, m_ipData.getStamp());
	}
}
void FusionStation::stop(StopReasons reasons)
{
	if (!(reasons & StopReason::Pause))
	{
		m_fusionStrategy->reset();
	}
}
//-------------------------------------------------------------------------------------------------

STATION(FusionStation, "semesterprojekt/fusion/Fusion");