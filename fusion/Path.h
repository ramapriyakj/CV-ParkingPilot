#pragma once

#include "veFramework/veBaseTypes.h"

template<typename Type>
class Path 
{
public:
	virtual ~Path() = default;

	virtual float64 duration() const = 0;

	virtual Type operator() (float64 t) const = 0;

	virtual Type dt(float64 t) const = 0;

	virtual Type ddt(float64 t) const = 0;
};
