/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include <functional>

#include <glm/glm.hpp>

namespace isomesh
{

struct SurfaceFunction {
	std::function<double (glm::dvec3)> f;
	double operator () (glm::dvec3 p) const noexcept { return f (p); }
};

class ZeroFinder {
public:
	// f (p0) <= 0, f (x1, p0.y, p0.z) > 0
	// or it won't work
	virtual double findAlongX (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept = 0;
	virtual double findAlongY (SurfaceFunction &f, glm::dvec3 p0, double y1) const noexcept = 0;
	virtual double findAlongZ (SurfaceFunction &f, glm::dvec3 p0, double z1) const noexcept = 0;
};

class StepCountedZeroFinder : public ZeroFinder {
public:
	StepCountedZeroFinder () noexcept = default;
	StepCountedZeroFinder (int steps) noexcept { setStepCount (steps); }
	
	void setStepCount (int steps) noexcept { m_stepCount = std::max (1, steps); }
	int stepCount () const noexcept { return m_stepCount; }
protected:
	int m_stepCount = 8;
};

class BisectionZeroFinder : public StepCountedZeroFinder {
public:
	BisectionZeroFinder () noexcept = default;
	BisectionZeroFinder (int steps) noexcept : StepCountedZeroFinder (steps) {}

	double findAlongX (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept override;
	double findAlongY (SurfaceFunction &f, glm::dvec3 p0, double y1) const noexcept override;
	double findAlongZ (SurfaceFunction &f, glm::dvec3 p0, double z1) const noexcept override;
private:
	template<size_t c>
	double findAlongC (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept;
};

class RegulaFalsiZeroFinder : public StepCountedZeroFinder {
public:
	RegulaFalsiZeroFinder () noexcept = default;
	RegulaFalsiZeroFinder (int steps) noexcept : StepCountedZeroFinder (steps) {}
	
	double findAlongX (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept override;
	double findAlongY (SurfaceFunction &f, glm::dvec3 p0, double y1) const noexcept override;
	double findAlongZ (SurfaceFunction &f, glm::dvec3 p0, double z1) const noexcept override;
};

}

