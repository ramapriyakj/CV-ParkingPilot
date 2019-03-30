#pragma once

#include "Noise.h"

class LidarNoise : public Noise
{
public:
	LidarNoise();

	virtual vePose sample(float64 t, const vePose& pose) override;

private:
	cMatrix<float64> m_covPosition;
	cMatrix<float64> m_covOrientation;
};