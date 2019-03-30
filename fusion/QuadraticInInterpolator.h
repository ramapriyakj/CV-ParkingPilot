#pragma once

#include "Interpolator.h"
#include "ScalingFunction.h"

template<typename Type>
class QuadraticInInterpolator : public Interpolator<Type>
{
public:
	virtual Type operator() (const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		auto st = s(duration)(t);
		return (to - from) * st * st + from;
	}

	virtual Type dt(const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		auto st = s(duration)(t);
		auto dsdt = s(duration).dt(t);

		return float64(2) * (to - from) * st * dsdt;
	}

	virtual Type ddt(const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		auto st = s(duration)(t);
		auto dsdt = s(duration).dt(t);
		auto dsddt = s(duration).ddt(t);

		return float64(2) * (to - from) * (dsdt * dsdt + st * dsddt);
	}

private:
	constexpr ScalingFunction<float64> s(float64 duration) const
	{
		return ScalingFunction<float64>(float64(1) / duration);
	}
};