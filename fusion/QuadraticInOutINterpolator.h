#pragma once

#include "Interpolator.h"
#include "ScalingFunction.h"

template<typename Type>
class QuadraticInOutInterpolator : public Interpolator<Type>
{
public:
	virtual Type operator() (const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		auto st = s(duration)(t);
		return st < float64(0.5) ?
			float64(2) * (to - from) * st * st + from :
			-float64(0.5) * (to - from) * ((float64(2) * st - float64(1))*(float64(2) * st - float64(3.0)) - float64(1.0)) + from;
	}

	virtual Type dt(const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		auto st = s(duration)(t);
		auto dsdt = s(duration).dt(t);

		return st < float64(0.5) ?
			float64(4) * (to - from) * st * dsdt :
			-float64(4)* (to - from) * (st - float64(1)) * dsdt;
	}

	virtual Type ddt(const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		auto st = s(duration)(t);
		auto dsdt = s(duration).dt(t);
		auto dsddt = s(duration).ddt(t);

		return st < float64(0.5) ?
			(float64(4) * (to - from) * (dsdt * dsdt + st * dsddt)) :
			(-float64(4) * (to - from) * (dsdt * dsdt + (st - float64(1)) * dsddt));
	}

private:
	constexpr ScalingFunction<float64> s(float64 duration) const
	{
		return ScalingFunction<float64>(float64(1) / duration);
	}
};
