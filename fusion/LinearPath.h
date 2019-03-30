#pragma once

#include "Interpolator.h"
#include "Path.h"

#include "veFramework/veBaseTypes.h"

#include <assert.h>
#include <memory>
#include <map>

template<typename Type>
class LinearPath : public Path<Type>
{
private:
	struct Knot
	{
		Type                                position;
		float64                             t;
		std::shared_ptr<Interpolator<Type>> interpolator;
	};

public:
	LinearPath(std::unique_ptr<Interpolator<Type>>&& interpolator)
	{
		m_interpolator = std::move(interpolator);
	}

	void addKnot(Type position, float64 t)
	{
		m_knots[t] = { position, t, m_interpolator };
	}

	void addKnot(Type position, float64 t, std::unique_ptr<Interpolator<Type>>&& interpolator)
	{
		m_knots[t] = { position, t, std::move(interpolator) };
	}

	void findSegment(float64 t, Knot& start, Knot& end) const
	{
		for (auto it = std::begin(m_knots); it != std::end(m_knots); ++it)
		{
			auto it_next = std::next(it);

			if (it_next == std::end(m_knots))
			{
				start = (it->second);
				end = (it->second);
				return;
			}

			if (t >= it->first && t <= it_next->first)
			{
				start = (it->second);
				end = (it_next->second);
				return;
			}
		}

		// Should never be reached
		assert(0);
	}

	inline float64 duration() const override
	{
		if (m_knots.empty())
		{
			return 0.0;
		}
		else
		{
			return (--std::end(m_knots))->first;
		}
	}

	Type operator() (float64 t) const override
	{
		Knot start;
		Knot end;
		findSegment(t, start, end);

		return (*(start.interpolator))(start.position, end.position, t - start.t, end.t - start.t);
	}

	Type dt(float64 t) const override
	{
		Knot start;
		Knot end;
		findSegment(t, start, end);

		return (*(start.interpolator)).dt(start.position, end.position, t - start.t, end.t - start.t);
	}

	Type ddt(float64 t) const override
	{
		Knot start;
		Knot end;
		findSegment(t, start, end);

		return (*(start.interpolator)).ddt(start.position, end.position, t - start.t, end.t - start.t);
	}

private:
	std::shared_ptr<Interpolator<Type>> m_interpolator;
	std::map<float64, Knot>             m_knots;
};