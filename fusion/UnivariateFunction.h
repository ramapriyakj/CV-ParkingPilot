#pragma once

template<typename ReturnType, typename ArgumentType>
class UnivariateFunction
{
public:
	constexpr UnivariateFunction() = default;

	virtual ~UnivariateFunction() = default;

	constexpr virtual ReturnType operator() (ArgumentType t) const = 0;

	constexpr virtual ReturnType dt(ArgumentType t) const = 0;

	constexpr virtual ReturnType ddt(ArgumentType t) const = 0;
};
