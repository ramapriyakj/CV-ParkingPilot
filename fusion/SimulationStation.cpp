#include "fusionPCH.h"
#include "SimulationStation.h"
#include "CameraNoise.h"
#include "EllipticPath.h"
#include "Interpolators.h"
#include "LidarNoise.h"
#include "LinearPath.h"
#include "RadarNoise.h"
#include "../common/Helper.h"
//-------------------------------------------------------------------------------------------------
SimulationStation::SimulationStation()
	: m_opEgomotion(this, "egomotion")
	, m_opCameraPose(this, "pose_camera")
	, m_opLidarPose(this, "pose_lidar")
	, m_opRadarPose(this, "pose_radar")
	, m_opGroundTruthPose(this, "pose_groundtruth")
	, m_firstCycle(0)
{
	m_dataRate = 60;

	// Create the path
	auto path = std::make_unique<LinearPath<cDblVector>>(createQuadraticInOut<cDblVector>());
	path->addKnot(cVector<float64>(326.0, 572.0, 0.0), 0.0);
	path->addKnot(cVector<float64>(270.0, 695.0, 0.0), 100.0);
	path->addKnot(cVector<float64>(365.0, 732.0, 0.0), 200.0);
	path->addKnot(cVector<float64>(450.0, 555.0, 0.0), 350.0);
	path->addKnot(cVector<float64>(366.0, 521.0, 0.0), 450.0);
	path->addKnot(cVector<float64>(326.0, 572.0, 0.0), 550.0);
	m_path = std::move(path);
	//m_path = std::make_unique<EllipticPath<cDblVector>>(cDblVector{ 350.0, 620.0 }, 80.0, 50.0, 120.0, createQuadraticInOut<float64>());

	m_cameraNoise = std::make_unique<CameraNoise>();
    m_lidarNoise = std::make_unique<LidarNoise>();
	m_radarNoise = std::make_unique<RadarNoise>();
}
//-------------------------------------------------------------------------------------------------
SimulationStation::~SimulationStation()
{

}
//-------------------------------------------------------------------------------------------------
bool SimulationStation::supportsPause() const
{
	return true;
}
bool SimulationStation::createData()
{
	float64 t = static_cast<float64>(m_cycle - m_firstCycle);

	if (t >= m_path->duration())
	{
		return false;
	}

	// Sample the data from the path
	cDblVector p = (*m_path)(t);
	cDblVector v = (*m_path).dt(t);
	cDblVector a = (*m_path).ddt(t);

	// Construct ground truth pose
	float64 orientation = atan2(v[1], v[0]);
	vePose pose(p, cDblMatrix::createIdentity(3, 0.0),
				cDblVector(0.0, 0.0, orientation), cDblMatrix::createIdentity(3, 0.0),
		        cBufferStamp(m_cycle));

	// Construct ground truth egomotion
	cDblVector xBase = v.normalized();
	cDblVector yBase = cDblVector(xBase[1], -xBase[0], 0);
	cDblVector zBase = cDblVector(0.0, 0.0, 1.0);
	cDblMatrix R{ 3, 3, 
	{ xBase[0], xBase[1], xBase[2],
	  yBase[0], yBase[1], yBase[2], 
	  zBase[0], zBase[1], zBase[2] } };
	// Rotation of velocity in egosystem and conversion from px/cycle to m/s
	cDblVector metricVelocity = R * v * m_dataRate * METER_PER_PIXEL;
	// Rotation of acceleration in egosystem and conversion from px/cycle^2 to m/s^2
	cDblVector metricAcceleration = R * a * METER_PER_PIXEL * m_dataRate * m_dataRate;
	// Calculation of angular velocity
	float64 dyaw = acos(metricVelocity.normalized().dotProduct((metricVelocity + metricAcceleration).normalized()));
	dyaw = metricAcceleration.getY() >= 0 ? dyaw : -dyaw;
	veEgomotion egomotion{ 
		metricVelocity, cDblVector{ 0.0, 0.0, dyaw },
		metricAcceleration,
		1.0 / m_dataRate, 
		"", 
		cBufferStamp(m_cycle) };

	// Send the ground truth data
	m_opGroundTruthPose.send(pose, cBufferStamp(m_cycle));
	m_opGroundTruthPose.sendFinish(cBufferStamp(m_cycle));

	// Send the egomotion
	m_opEgomotion.send(egomotion, cBufferStamp(m_cycle));
	m_opEgomotion.sendFinish(cBufferStamp(m_cycle));

	// Add noise to the pose and send it as camera pose
	m_opCameraPose.send(m_cameraNoise->sample(t, pose), cBufferStamp(m_cycle));
	m_opCameraPose.sendFinish(cBufferStamp(m_cycle));

	// Add noise to the pose and send it as lidar pose
	m_opLidarPose.send(m_lidarNoise->sample(t, pose), cBufferStamp(m_cycle));
	m_opLidarPose.sendFinish(cBufferStamp(m_cycle));
	
	// Add noise to the pose and send it as radar pose
	m_opRadarPose.send(m_radarNoise->sample(t, pose), cBufferStamp(m_cycle));
	m_opRadarPose.sendFinish(cBufferStamp(m_cycle));

	return true;
}

void SimulationStation::start(StartReasons reasons)
{
	cDataSourceStation::start(reasons);

	if (!(reasons & StartReason::Resume))
	{
		m_firstCycle = m_cycle;
	}
}

STATION(SimulationStation, "semesterprojekt/fusion/Simulation");
