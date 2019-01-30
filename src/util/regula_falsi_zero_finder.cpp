/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <algorithm>

#include <isomesh/util/zero_finder.hpp>

namespace isomesh
{

// Anderson-Björk algorithm
template<size_t C>
double RegulaFalsiZeroFinder::findAlongC (double x0, double y0, double z0,
                                          double c1, double f0, double f1,
                                          const SurfaceFunction &f) const {
	glm::dvec3 p (x0, y0, z0);
	double c0 = std::get<C> (p);
	int side = 0;
	double mid;
	for (int i = 0; i < m_stepCount; i++) {
		mid = (c0 * f1 - c1 * f0) / (f1 - f0);
		std::get<C> (p) = mid;
		double val = f (p);
		
		if (val * f0 > 0) {
			c0 = mid;
			if (side == -1) {
				double m = 1.0 - val / f0;
				if (m <= 0) m = 0.5;
				f1 *= m;
			}
			f0 = val;
			side = -1;
		}
		else if (val * f1 > 0) {
			c1 = mid;
			if (side == +1) {
				double m = 1.0 - val / f1;
				if (m <= 0) m = 0.5;
				f0 *= m;
			}
			f1 = val;
			side = +1;
		}
		else break;
	}
	return mid;
}

double RegulaFalsiZeroFinder::findAlongX (double x0, double y0, double z0,
                                          double x1, double f0, double f1,
                                          const SurfaceFunction &f) const {
	return findAlongC<0> (x0, y0, z0, x1, f0, f1, f);
}

double RegulaFalsiZeroFinder::findAlongY (double x0, double y0, double z0,
                                          double y1, double f0, double f1,
                                          const SurfaceFunction &f) const {
	return findAlongC<1> (x0, y0, z0, y1, f0, f1, f);
}

double RegulaFalsiZeroFinder::findAlongZ (double x0, double y0, double z0,
                                          double z1, double f0, double f1,
                                          const SurfaceFunction &f) const {
	return findAlongC<2> (x0, y0, z0, z1, f0, f1, f);
}

}
