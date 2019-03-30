#pragma once

#include "veFramework/veBaseTypes.h"

template<typename Type>
class Interpolator
{
public:
	virtual ~Interpolator() = default;

	virtual Type operator() (const Type& from, const Type& to, float64 t, float64 duration = 1.0) const = 0;

	virtual Type dt(const Type& from, const Type& to, float64 t, float64 duration = 1.0) const = 0;

	virtual Type ddt(const Type& from, const Type& to, float64 t, float64 duration = 1.0) const = 0;
};
