#pragma once

#include "Interpolator.h"

template<typename Type>
class SquareInInterpolator : public Interpolator<Type>
{
public:
	virtual Type operator() (const Type& from, const Type& to, float64 t, float64 tScale = 1.0) const
	{
		t = t * tScale;
		return (1 - t*t) * from + t*t * to;
	};

	virtual Type dt(const Type& from, const Type& to, float64 t, float64 tScale = 1.0) const
	{
		// - 2t*s^2 * from + 2t*s^2 * to = 2t*s^2 (to - from)
		return 2.0 * t * tScale * tScale * (to - from);
	};

	virtual Type dt2(const Type& from, const Type& to, float64 t, float64 tScale = 1.0) const
	{
		return 2.0 * tScale * tScale * (to - from);
	};
};