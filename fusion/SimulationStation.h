#pragma once

#include "Path.h"
#include "Noise.h"

#include "cdl/cDataSourceStation.h"

class SimulationStation : public cDataSourceStation
{
	STATIONDECL(SimulationStation);
	SimulationStation();
	virtual ~SimulationStation();

	virtual bool supportsPause() const override;
	virtual bool createData() override;
	virtual void start(StartReasons reasons) override;

	cOPort<veEgomotion>							   m_opEgomotion;
	cOPort<vePose>								   m_opCameraPose;
	cOPort<vePose>								   m_opLidarPose;
	cOPort<vePose>								   m_opRadarPose;
	cOPort<vePose>								   m_opGroundTruthPose;

	std::unique_ptr<Path<cVector<float64>>> m_path;
	std::unique_ptr<Noise>                  m_cameraNoise;
	std::unique_ptr<Noise>                  m_lidarNoise;
	std::unique_ptr<Noise>                  m_radarNoise;
	uint32	                                m_firstCycle;
};