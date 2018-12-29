/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
// Tests for default Isomesh zero finders
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <isomesh/util/zero_finder.hpp>

using std::cerr;
using std::clog;
using std::endl;
using std::fixed;
using std::setprecision;
using std::vector;

isomesh::BisectionZeroFinder bisect;
isomesh::RegulaFalsiZeroFinder regula;

// Solvers have 8 steps by default, so error should decrease at least 2^8 times.
// Bisection must always pass this test (if coded properly), but regula falsi
// might fail in some rare cases. It will converge to the root nonetheless.
const double kMaxError = 1.0 / 256.0;

bool testX (isomesh::SurfaceFunction &f, double x0, double x1, vector<double> roots);
bool testY (isomesh::SurfaceFunction &f, double y0, double y1, vector<double> roots);
bool testZ (isomesh::SurfaceFunction &f, double z0, double z1, vector<double> roots);

// Polynomials
bool testPoly () {
	{
		clog << "Testing f(x) = x^2 - 8x + 15" << endl;
		isomesh::SurfaceFunction poly2;
		poly2.f = [] (glm::dvec3 p) {
			return p.x * p.x - 8.0 * p.x + 15.0;
		};
		poly2.grad = [] (glm::dvec3 p) {
			return glm::dvec3 (2.0 * p.x - 8.0, 0, 0);
		};
		if (!testX (poly2, 0.0, 4.0, { 3.0 })) return false;
		if (!testX (poly2, 3.0, 4.0, { 3.0 })) return false;
		if (!testX (poly2, 4.0, 5.0, { 5.0 })) return false;
		if (!testX (poly2, 4.0, 5.5, { 5.0 })) return false;
	}
	{
		clog << "Testing f(x) = x^3 - 6x^2 + 11x - 6" << endl;
		isomesh::SurfaceFunction poly3;
		poly3.f = [] (glm::dvec3 p) {
			return p.x * p.x * p.x - 6.0 * p.x * p.x + 11.0 * p.x - 6.0;
		};
		poly3.grad = [] (glm::dvec3 p) {
			return glm::dvec3 (3.0 * p.x * p.x - 12.0 * p.x + 11.0, 0, 0);
		};
		if (!testX (poly3, 0.0, 1.5, { 1.0 })) return false;
		if (!testX (poly3, 1.5, 2.5, { 2.0 })) return false;
		if (!testX (poly3, 2.5, 3.5, { 3.0 })) return false;
		if (!testX (poly3, 0.0, 2.0, { 1.0, 2.0 })) return false;
		if (!testX (poly3, 1.0, 2.5, { 1.0, 2.0 })) return false;
		if (!testX (poly3, 0.0, 3.5, { 1.0, 2.0, 3.0 })) return false;
	}
	return true;
}

// Exponential functions
bool testExp () {
	{
		clog << "Testing f(y, z) = e^y + e^z - 4" << endl;
		isomesh::SurfaceFunction f;
		f.f = [] (glm::dvec3 p) {
			return exp (p.y) + exp (p.z) - 4.0;
		};
		f.grad = [] (glm::dvec3 p) {
			return glm::dvec3 (0, exp (p.y), exp (p.z));
		};
		if (!testY (f, 1.0, 2.0, { log (3.0) })) return false;
		if (!testZ (f, 1.0, 2.0, { log (3.0) })) return false;
	}
	{
		// This function is designed to defeat regula falsi.
		// Looking at its plot you can see that its linear interpolation
		// gives results far off from the actual root, slowing down convergence.
		clog << "Testing f(z) = ln(z) if z <= 1, else 1 - exp(-1.9*(z-1))" << endl;
		isomesh::SurfaceFunction f;
		f.f = [] (glm::dvec3 p) {
			if (p.z <= 1.0)
				return log (p.z);
			return 1.0 - exp (-1.9 * (p.z - 1.0));
		};
		f.grad = [] (glm::dvec3 p) {
			if (p.z <= 1.0)
				return glm::dvec3 (0, 0, 1.0 / p.z);
			return glm::dvec3 (0, 0, 1.9 * (p.z - 1.0) * exp (-1.9 * (p.z - 1.0)));
		};
		if (!testZ (f, 0.5, 1.5, { 1.0 })) return false;
		if (!testZ (f, 0.25, 3.0, { 1.0 })) return false;
		if (!testZ (f, 0.9, 10.0, { 1.0 })) return false;
		if (!testZ (f, 0.1, 3.0, { 1.0 })) return false;
		if (!testZ (f, 0.4, 7.2, { 1.0 })) return false;
		// This test should be very tough for regula falsi because of very slow
		// convergence rate. Increasing the multiplier in exponent will make
		// things even worse. Setting it to 2 is just enough for regula falsi
		// to fail the "not worse than bisection" condition.
		if (!testZ (f, 0.1, 10.0, { 1.0 })) return false;
	}
	return true;
}

int main () {
	bool fail = false;
	if (!testPoly ())
		fail = true;
	if (!testExp ())
		fail = true;
	if (fail)
		return 1;
	return 0;
}

bool testX (isomesh::SurfaceFunction &f, double x0, double x1, vector<double> roots) {
	clog << std::defaultfloat << "Along X from " << x0 << " to " << x1;
	if (roots.size () == 1) clog << ", root is " << roots[0] << endl;
	else {
		clog << ", roots are ";
		for (double root : roots) clog << root << ' ';
		clog << endl;
	}

	double f0 = f (glm::dvec3 (x0, 0, 0));
	double f1 = f (glm::dvec3 (x1, 0, 0));
	double bi_root = bisect.findAlongX (x0, 0, 0, x1, f0, f1, f);
	double bi_error = fabs (bi_root - roots[0]);
	double re_root = regula.findAlongX (x0, 0, 0, x1, f0, f1, f);
	double re_error = fabs (re_root - roots[0]);
	for (double root : roots) {
		bi_error = std::min (bi_error, fabs (root - bi_root));
		re_error = std::min (re_error, fabs (root - re_root));
	}
	clog << "   Bisect found: " << fixed << setprecision (10) << bi_root
	     << ", error " << bi_error << endl;
	clog << "   Regula found: " << fixed << setprecision (10) << re_root
	     << ", error " << re_error << endl;
	double max_error = (x1 - x0) * kMaxError;
	clog << "   Maximal tolerated error is " << fixed << setprecision (10) << max_error << endl;
	if (bi_error > max_error) {
		cerr << "Bisection failed to give enough precision!" << endl;
		return false;
	}
	if (re_error > max_error) {
		cerr << "Regula falsi failed to give enough precision!" << endl;
		return false;
	}
	return true;
};

bool testY (isomesh::SurfaceFunction &f, double y0, double y1, vector<double> roots) {
	clog << std::defaultfloat << "Along Y from " << y0 << " to " << y1;
	if (roots.size () == 1) clog << ", root is " << roots[0] << endl;
	else {
		clog << ", roots are ";
		for (double root : roots) clog << root << ' ';
		clog << endl;
	}

	double f0 = f (glm::dvec3 (0, y0, 0));
	double f1 = f (glm::dvec3 (0, y1, 0));
	double bi_root = bisect.findAlongY (0, y0, 0, y1, f0, f1, f);
	double bi_error = fabs (bi_root - roots[0]);
	double re_root = regula.findAlongY (0, y0, 0, y1, f0, f1, f);
	double re_error = fabs (re_root - roots[0]);
	for (double root : roots) {
		bi_error = std::min (bi_error, fabs (root - bi_root));
		re_error = std::min (re_error, fabs (root - re_root));
	}
	clog << "   Bisect found: " << fixed << setprecision (10) << bi_root
	     << ", error " << bi_error << endl;
	clog << "   Regula found: " << fixed << setprecision (10) << re_root
	     << ", error " << re_error << endl;
	double max_error = (y1 - y0) * kMaxError;
	clog << "   Maximal tolerated error is " << fixed << setprecision (10) << max_error << endl;
	if (bi_error > max_error) {
		cerr << "Bisection failed to give enough precision!" << endl;
		return false;
	}
	if (re_error > max_error) {
		cerr << "Regula falsi failed to give enough precision!" << endl;
		return false;
	}
	return true;
};

bool testZ (isomesh::SurfaceFunction &f, double z0, double z1, vector<double> roots) {
	clog << std::defaultfloat << "Along Z from " << z0 << " to " << z1;
	if (roots.size () == 1) clog << ", root is " << roots[0] << endl;
	else {
		clog << ", roots are ";
		for (double root : roots) clog << root << ' ';
		clog << endl;
	}

	double f0 = f (glm::dvec3 (0, 0, z0));
	double f1 = f (glm::dvec3 (0, 0, z1));
	double bi_root = bisect.findAlongZ (0, 0, z0, z1, f0, f1, f);
	double bi_error = fabs (bi_root - roots[0]);
	double re_root = regula.findAlongZ (0, 0, z0, z1, f0, f1, f);
	double re_error = fabs (re_root - roots[0]);
	for (double root : roots) {
		bi_error = std::min (bi_error, fabs (root - bi_root));
		re_error = std::min (re_error, fabs (root - re_root));
	}
	clog << "   Bisect found: " << fixed << setprecision (10) << bi_root
	     << ", error " << bi_error << endl;
	clog << "   Regula found: " << fixed << setprecision (10) << re_root
	     << ", error " << re_error << endl;
	double max_error = (z1 - z0) * kMaxError;
	clog << "   Maximal tolerated error is " << fixed << setprecision (10) << max_error << endl;
	if (bi_error > max_error) {
		cerr << "Bisection failed to give enough precision!" << endl;
		return false;
	}
	if (re_error > max_error) {
		cerr << "Regula falsi failed to give enough precision!" << endl;
		return false;
	}
	return true;
};
