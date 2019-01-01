/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include <algorithm>

#include "../common.hpp"

namespace isomesh
{

class ZeroFinder {
public:
	// Preconditions: f0 * f1 <= 0 and |f0| + |f1| > 0
	// These are assumed to be satisfied when findAlong* is called

	// f0 = f (x0, y0, z0)
	// f1 = f (x1, y0, z0)
	virtual double findAlongX (double x0, double y0, double z0,
	                           double x1, double f0, double f1,
	                           SurfaceFunction &f) const noexcept = 0;
	// f0 = f (x0, y0, z0)
	// f1 = f (x0, y1, z0)
	virtual double findAlongY (double x0, double y0, double z0,
	                           double y1, double f0, double f1,
	                           SurfaceFunction &f) const noexcept = 0;
	// f0 = f (x0, y0, z0)
	// f1 = f (x0, y0, z1)
	virtual double findAlongZ (double x0, double y0, double z0,
	                           double z1, double f0, double f1,
	                           SurfaceFunction &f) const noexcept = 0;
};

class StepCountedZeroFinder : public ZeroFinder {
public:
	StepCountedZeroFinder () noexcept = default;
	explicit StepCountedZeroFinder (int steps) noexcept { setStepCount (steps); }

	void setStepCount (int steps) noexcept { m_stepCount = std::max (1, steps); }
	int stepCount () const noexcept { return m_stepCount; }
protected:
	int m_stepCount = 8;
};

class BisectionZeroFinder : public StepCountedZeroFinder {
public:
	BisectionZeroFinder () noexcept = default;
	explicit BisectionZeroFinder (int steps) noexcept : StepCountedZeroFinder (steps) {}

	double findAlongX (double x0, double y0, double z0,
	                   double x1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept override;
	double findAlongY (double x0, double y0, double z0,
	                   double y1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept override;
	double findAlongZ (double x0, double y0, double z0,
	                   double z1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept override;
private:
	template<size_t C>
	double findAlongC (double x0, double y0, double z0,
	                   double c1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept;
};

class RegulaFalsiZeroFinder : public StepCountedZeroFinder {
public:
	RegulaFalsiZeroFinder () noexcept = default;
	explicit RegulaFalsiZeroFinder (int steps) noexcept : StepCountedZeroFinder (steps) {}

	double findAlongX (double x0, double y0, double z0,
	                   double x1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept override;
	double findAlongY (double x0, double y0, double z0,
	                   double y1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept override;
	double findAlongZ (double x0, double y0, double z0,
	                   double z1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept override;
private:
	template<size_t C>
	double findAlongC (double x0, double y0, double z0,
	                   double c1, double f0, double f1,
	                   SurfaceFunction &f) const noexcept;
};

}
