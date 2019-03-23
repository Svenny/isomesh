/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/qef/qef_solver_4d.hpp>

#include <stdexcept>
#include <valarray>

namespace isomesh
{

namespace qr_detail
{

// Column-major matrix
struct Matrix {
	Matrix (size_t rows) {
		for (int i = 0; i < 5; i++)
			a[i].resize (rows, 0);
	}
	std::valarray<float> a[5];
	size_t rows () const noexcept { return a[0].size (); }
	std::valarray<float> &operator [] (size_t idx) noexcept { return a[idx]; }
	const std::valarray<float> &operator [] (size_t idx) const noexcept { return a[idx]; }
};

// Performs QR decomposition using Householder reflections (matrix A is replaced with R)
void householder (Matrix &A) {
	std::valarray<float> v (A.rows ());
	for (size_t i = 0; i < 5; i++) {
		// norm of i-th subcolumn
		float norm = 0;
		for (size_t j = i; j < A.rows (); j++)
			norm += A.a[i][j] * A.a[i][j];
		float invgamma = 0;
		if (norm < std::numeric_limits<float>::min ()) {
			v = 0;
			v[i] = 1.0f;
			invgamma = 2.0f;
		}
		else {
			v = A.a[i];
			v[std::slice (0, i, 1)] = 0;
			v *= 1.0f / glm::sqrt (norm);
			if (v[i] >= 0)
				v[i] += 1.0f;
			else
				v[i] -= 1.0f;
			invgamma = 1.0f / glm::abs (v[i]);
		}
		for (size_t j = i; j < 5; j++) {
			float dot = 0;
			for (size_t k = i; k < A.rows (); k++)
				dot += A.a[j][k] * v[k];
			A.a[j] -= dot * invgamma * v;
		}
	}
}

glm::vec4 solveDeterminedR (const Matrix &A) {
	float vals[4];
	for (int i = 3; i >= 0; i--) {
		float coef = A[4][i];
		for (int j = i + 1; j < 4; j++)
			coef -= A[j][i] * vals[j];
		vals[i] = coef / A[i][i];
	}
	return { vals[0], vals[1], vals[2], vals[3] };
}

}

using namespace qr_detail;

glm::vec4 QrQefSolver4D::solve (glm::vec4 minPoint, glm::vec4 maxPoint) {
	if (m_numPlanes == 0)
		throw std::runtime_error ("Solver has no input");
	glm::vec4 mass_point = m_massPoint / float (m_numPlanes);
	Matrix A (m_numPlanes + 4);
	for (int i = 0; i < m_numPlanes; i++) {
		glm::vec4 normal = m_normals[i];
		float coef = m_coefs[i] - glm::dot (normal, mass_point);
		A[0][i] = normal.x;
		A[1][i] = normal.y;
		A[2][i] = normal.z;
		A[3][i] = normal.w;
		A[4][i] = coef;
	}
	/* Add fake planes to make the system always determined (so it always has unique solution).
	 This adds some bias to the solution, but it is generally negligible. */
	for (int i = 0; i < 4; i++)
		A[i][m_numPlanes + i] = 0.1f * m_tolerance;
	householder (A);
	glm::vec4 res = solveDeterminedR (A) + mass_point;
	/* Clamp solution point to cell boundaries if it lies outside. This is an ad-hoc method that
	 should ideally be replaced with a proper bounded-variable least squares (BVLS) solver. */
	return glm::clamp (res, minPoint, maxPoint);
}

}

