#pragma once

#include "Interpolator.h"
#include "ScalingFunction.h"

template<typename Type>
class LinearInterpolator : public Interpolator<Type>
{
public:
	virtual Type operator() (const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		return (to - from) * s(duration)(t) + from;
	}

	virtual Type dt(const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		return (to - from) * s(duration).dt(t);
	}

	virtual Type ddt(const Type& from, const Type& to, float64 t, float64 duration) const override
	{
		return (to - from) * s(duration).ddt(t);
	}

private:
	constexpr ScalingFunction<float64> s(float64 duration) const
	{
		return ScalingFunction<float64>(1.0 / duration);
	}
};
