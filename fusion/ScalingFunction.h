#pragma once

#include "UnivariateFunction.h"

template<typename ArgumentType>
class ScalingFunction : public UnivariateFunction<ArgumentType, ArgumentType>
{
public:
	constexpr ScalingFunction(ArgumentType scale)
		: m_scale(scale)
	{
	}

	constexpr virtual ArgumentType operator() (ArgumentType t) const override
	{
		return m_scale * t;
	}

	constexpr virtual ArgumentType dt(ArgumentType t) const override
	{
		return m_scale;
	}

	constexpr virtual ArgumentType ddt(ArgumentType t) const override
	{
		return ArgumentType(0);
	}

private:
	ArgumentType m_scale;
};
