#pragma once

#include "Noise.h"

class CameraNoise : public Noise
{
public:
	CameraNoise();

	virtual vePose sample(float64 t, const vePose& pose) override;

private:
	cMatrix<float64> m_covPosition;
	cMatrix<float64> m_covOrientation;
};