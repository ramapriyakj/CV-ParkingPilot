#pragma once

#include <tuple>
#include <vector>

class EvaluationStation : public cStation
{
	STATIONDECL(EvaluationStation);
	EvaluationStation();
	virtual ~EvaluationStation();

	void dataCallback();

	void stop(StopReasons reasons) override;

	PORTGROUP(vePose, vePose) m_ipData;
	cOPort<float64>           m_opRmsePosition;
	cOPort<float64>           m_opRmseOrientation;

	std::vector<std::tuple<cDblVector, cDblVector>> m_estimationGroundTruthPairs;
};