#pragma once

#include "Interpolator.h"
#include "Path.h"

#include "../common/Helper.h"

#include "veFramework/veBaseTypes.h"

#include <cmath>
#include <memory>

template<typename Type>
class EllipticPath : public Path<Type>
{
public:
	EllipticPath(const Type& center, float64 a, float64 b,
				 float64 duration,
		         std::unique_ptr<Interpolator<float64>>&& interpolator)
		: m_center(center)
		, m_a(a)
		, m_b(b)
		, m_duration(duration)
		, m_interpolator(std::move(interpolator))
	{
	}

	EllipticPath(const Type& center, float64 radius,
		         float64 duration,
		         std::unique_ptr<Interpolator<float64>>&& interpolator)
		: elliptic_path(center, radius, radius, duration, std::move(interpolator))
	{
	}

	inline float64 duration() const override
	{
		return m_duration;
	}

	Type operator() (float64 t) const override
	{
		float64 angle = (*m_interpolator)(0, -2 * PI, t, m_duration);

		float64 x = m_a * std::cos(angle);
		float64 y = m_b * std::sin(angle);

		return Type{ x, y } + m_center;
	}

	Type dt(float64 t) const override
	{
		float64 angle = (*m_interpolator)(0, -2 * PI, t, m_duration);
		float64 dangledt = (*m_interpolator).dt(0, -2 * PI, t, m_duration);

		float64 dx = -m_a * std::sin(angle) * dangledt;
		float64 dy = m_b * std::cos(angle) * dangledt;

		return Type{ dx, dy };
	}

	Type ddt(float64 t) const override
	{
		float64 angle = (*m_interpolator)(0, -2 * PI, t, m_duration);
		float64 dangledt = (*m_interpolator).dt(0, -2 * PI, t, m_duration);
		float64 dangleddt = (*m_interpolator).ddt(0, -2 * PI, t, m_duration);

		float64 ddx = -m_a * (std::cos(angle) * dangledt * dangledt + std::sin(angle) * dangleddt);
		float64 ddy = m_b * (-std::sin(angle) * dangledt * dangledt + std::cos(angle) * dangleddt);

		return Type{ ddx, ddy };
	}

private:
	Type                                   m_center;
	float64                                m_a;
	float64                                m_b;
	float64                                m_duration;
	std::unique_ptr<Interpolator<float64>> m_interpolator;
};