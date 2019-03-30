#pragma once

#include "LinearInterpolator.h"
#include "QuadraticInInterpolator.h"
#include "QuadraticInOutInterpolator.h"

#include "veFramework/veBaseTypes.h"

#include <memory>

template<typename Type>
static inline std::unique_ptr<LinearInterpolator<Type>> createLinear()
{
	return std::make_unique<LinearInterpolator<Type>>();
}

template<typename Type>
static inline std::unique_ptr<QuadraticInInterpolator<Type>> createQuadraticIn()
{
	return std::make_unique<QuadraticInInterpolator<Type>>();
}

template<typename Type>
static inline std::unique_ptr<QuadraticInOutInterpolator<Type>> createQuadraticInOut()
{
	return std::make_unique<QuadraticInOutInterpolator<Type>>();
}
