/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <algorithm>

#include <isomesh/util/zero_finder.hpp>

namespace isomesh
{

template<size_t c>
double BisectionZeroFinder::findAlongC (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept {
	double x0 = p0[c];
	for (int i = 0; i < m_stepCount; i++) {
		double x = (x0 + x1) * 0.5;
		p0[c] = x;
		double val = f (p0);
		if (val <= 0.0)
			x0 = x;
		else x1 = x;
	}
	return (x0 + x1) * 0.5;
}

double BisectionZeroFinder::findAlongX (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept {
	return findAlongC<0> (f, p0, x1);
}

double BisectionZeroFinder::findAlongY (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept {
	return findAlongC<1> (f, p0, x1);
}

double BisectionZeroFinder::findAlongZ (SurfaceFunction &f, glm::dvec3 p0, double x1) const noexcept {
	return findAlongC<2> (f, p0, x1);
}

}

