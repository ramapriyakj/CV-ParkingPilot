#pragma once

#include "FusionStrategy.h"

class FusionStation : public cStation
{
	STATIONDECL(FusionStation);
	FusionStation();
	virtual ~FusionStation();

	void mapCallback(cBufferT<cImg>* i_buffer);
	void dataCallback();

	void stop(StopReasons reasons) override;

	PORT(cImg)									   m_ipMap;
	PORTGROUP(veEgomotion, vePose, vePose, vePose) m_ipData;
	cOPort<cImg>								   m_opMap;
	cOPort<vePose>								   m_opPose;

	cImg							m_map;
	std::unique_ptr<FusionStrategy> m_fusionStrategy;
	float64                         m_lidarPositionErrorScale;
	float64                         m_radarPositionErrorScale;
	float64                         m_cameraPositionErrorScale;
	float64                         m_lidarOrientationErrorScale;
	float64                         m_radarOrientationErrorScale;
	float64                         m_cameraOrientationErrorScale;
};