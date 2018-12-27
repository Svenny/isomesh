/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <algorithm>

#include <isomesh/util/zero_finder.hpp>

namespace isomesh
{

template<size_t C>
double BisectionZeroFinder::findAlongC (double x0, double y0, double z0,
                                        double c1, double f0, double f1,
                                        SurfaceFunction &f) const noexcept {
	glm::dvec3 p (x0, y0, z0);
	double c0 = std::get<C> (p);

	for (int i = 0; i < m_stepCount; i++) {
		double mid = (c0 + c1) * 0.5;
		std::get<C> (p) = mid;
		double val = f (p);
		if (val * f0 > 0) {
			c0 = mid;
			f0 = val;
		}
		else if (val * f1 > 0) {
			c1 = mid;
			f1 = val;
		}
		else {
			val = fabs (val);
			f0 = fabs (f0);
			f1 = fabs (f1);
			if (val <= std::min (f0, f1))
				return mid;
			if (f0 <= f1)
				return c0;
			return c1;
		}
	}
	if (fabs (f0) <= fabs (f1))
		return c0;
	return c1;
}

double BisectionZeroFinder::findAlongX (double x0, double y0, double z0,
                                        double x1, double f0, double f1,
                                        SurfaceFunction &f) const noexcept {
	return findAlongC<0> (x0, y0, z0, x1, f0, f1, f);
}

double BisectionZeroFinder::findAlongY (double x0, double y0, double z0,
                                        double y1, double f0, double f1,
                                        SurfaceFunction &f) const noexcept {
	return findAlongC<1> (x0, y0, z0, y1, f0, f1, f);
}

double BisectionZeroFinder::findAlongZ (double x0, double y0, double z0,
                                        double z1, double f0, double f1,
                                        SurfaceFunction &f) const noexcept {
	return findAlongC<2> (x0, y0, z0, z1, f0, f1, f);
}

}
