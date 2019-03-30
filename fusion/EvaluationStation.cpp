#include "fusionPCH.h"
#include "EvaluationStation.h"
//-------------------------------------------------------------------------------------------------
EvaluationStation::EvaluationStation()
	: cStation("")
	, m_ipData(this, &EvaluationStation::dataCallback, "estimate_pose", "groundtruth_pose")
	, m_opRmsePosition(this, "rmse_position")
	, m_opRmseOrientation(this, "rmse_orientation")
{
}
EvaluationStation::~EvaluationStation()
{
}
//-------------------------------------------------------------------------------------------------
void EvaluationStation::dataCallback()
{
	// Store the ground truth and estimation
	vePose estimatedPose = m_ipData.getValue<0>();
	vePose groundTruthPose = m_ipData.getValue<1>();
	m_estimationGroundTruthPairs.emplace_back(std::make_tuple(estimatedPose.get2DPose(), groundTruthPose.get2DPose()));
	
	// Calculate the RMSE including all measurements
	cDblVector sqrErrorSum{ 2, 0.0 };
	for (const std::tuple<cDblVector, cDblVector>& pair : m_estimationGroundTruthPairs)
	{
		cDblVector estimation = std::get<0>(pair);
		cDblVector groundTruth = std::get<1>(pair);

		cDblVector temp = estimation - groundTruth;
		
		sqrErrorSum[0] += temp[0] * temp[0] + temp[1] * temp[1];
		sqrErrorSum[1] += temp[2] * temp[2];
	}

	sqrErrorSum /= m_estimationGroundTruthPairs.size();

	m_opRmsePosition.send(sqrt(sqrErrorSum[0]), m_ipData.getStamp());
	m_opRmseOrientation.send(sqrt(sqrErrorSum[1]), m_ipData.getStamp());
}
//-------------------------------------------------------------------------------------------------
void EvaluationStation::stop(StopReasons reasons)
{
	if (!(reasons & StopReason::Pause))
	{
		m_estimationGroundTruthPairs.clear();
	}
}
//-------------------------------------------------------------------------------------------------
STATION(EvaluationStation, "semesterprojekt/fusion/Evaluation");