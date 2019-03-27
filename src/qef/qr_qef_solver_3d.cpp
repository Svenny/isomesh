/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/qef/qef_solver_3d.hpp>

#include <stdexcept>

namespace isomesh
{

QrQefSolver3D::QrQefSolver3D () noexcept {
	reset ();
}

QrQefSolver3D::QrQefSolver3D (const QefData &data) noexcept {
	reset ();
	A[0][0] = data.a_11;
	A[1][0] = data.a_12;
	A[2][0] = data.a_13;
	A[3][0] = data.b_1;
	A[1][1] = data.a_22;
	A[2][1] = data.a_23;
	A[3][1] = data.b_2;
	A[2][2] = data.a_33;
	A[3][2] = data.b_3;
	A[3][3] = data.r2;
	m_pointsSum = glm::vec3 (data.mpx, data.mpy, data.mpz);
	m_addedPoints = data.mp_cnt;
	m_featureDim = data.dim;
	m_usedRows = 4;
}

void QrQefSolver3D::addPlane (glm::vec3 point, glm::vec3 normal) {
	if (m_usedRows >= kRows)
		compressMatrix ();
	int idx = m_usedRows;
	A[0][idx] = normal.x;
	A[1][idx] = normal.y;
	A[2][idx] = normal.z;
	A[3][idx] = glm::dot (normal, point);
	m_usedRows++;
	m_pointsSum += point;
	m_addedPoints++;
}

glm::vec3 QrQefSolver3D::solve (glm::vec3 min_bound, glm::vec3 max_bound) {
	if (m_addedPoints == 0)
		throw std::runtime_error ("Solver has no input");
	compressMatrix ();
	// I know I am doing it wrong :P
	// TODO: Replace with linear algebra solver
	glm::vec3 P = m_pointsSum / float (m_addedPoints);
	for (int i = 0; i < 55; i++) {
		glm::vec3 grad (0);
		for (int j = 0; j < m_usedRows; j++) {
			glm::vec3 normal (A[0][j], A[1][j], A[2][j]);
			float coef = 2.0f * (glm::dot (normal, P) - A[3][j]);
			grad += coef * normal;
		}
		float buben = 1.0f + 0.1f * float (i);
		P -= (m_tolerance / buben) * grad;
		P = glm::clamp (P, min_bound, max_bound);
	}
	return P;
}

float QrQefSolver3D::eval (glm::vec3 point) const {
	float error = 0;
	for (int i = 0; i < m_usedRows; i++) {
		float dot = A[0][i] * point.x + A[1][i] * point.y + A[2][i] * point.z;
		float diff = dot - A[3][i];
		error += diff * diff;
	}
	return error;
}

void QrQefSolver3D::reset () {
	memset (A, 0, sizeof (A));
	m_pointsSum = glm::vec3 { 0 };
	m_addedPoints = 0;
	m_usedRows = 0;
	m_featureDim = -1;
}

void QrQefSolver3D::merge (const QefData &data) noexcept {
	if (m_usedRows > kRows - 4)
		compressMatrix ();
	int id = m_usedRows;
	A[0][id] = data.a_11;
	A[1][id] = data.a_12;
	A[2][id] = data.a_13;
	A[3][id] = data.b_1;
	A[1][id + 1] = data.a_22;
	A[2][id + 1] = data.a_23;
	A[3][id + 1] = data.b_2;
	A[2][id + 2] = data.a_33;
	A[3][id + 2] = data.b_3;
	A[3][id + 3] = data.r2;
	m_usedRows += 4;
	m_pointsSum += glm::vec3 (data.mpx, data.mpy, data.mpz);
	m_addedPoints += data.mp_cnt;
}

QrQefSolver3D::QefData QrQefSolver3D::data () noexcept {
	compressMatrix ();
	QefData res;
	res.a_11 = A[0][0];
	res.a_12 = A[1][0];
	res.a_13 = A[2][0];
	res.b_1 = A[3][0];
	res.a_22 = A[1][1];
	res.a_23 = A[2][1];
	res.b_2 = A[3][1];
	res.a_33 = A[2][2];
	res.b_3 = A[3][2];
	res.r2 = A[3][3];
	res.mpx = m_pointsSum.x;
	res.mpy = m_pointsSum.y;
	res.mpz = m_pointsSum.z;
	// Is it really safe to assume nobody will add 2^16 points?
	res.mp_cnt = int16_t (m_addedPoints);
	res.dim = int16_t (m_featureDim);
	return res;
}

void QrQefSolver3D::compressMatrix () {
	float v[kRows];
	for (int i = 0; i < 4; i++) {
		// norm of i-th subcolumn
		float norm = 0;
		for (int j = i; j < kRows; j++)
			norm += A[i][j] * A[i][j];
		memset (v, 0, sizeof (v));
		float invgamma = 0;
		if (norm < std::numeric_limits<float>::min ()) {
			v[i] = 1.0f;
			invgamma = 2.0f;
		}
		else {
			float mult = 1.0f / glm::sqrt (norm);
			for (int j = i; j < kRows; j++)
				v[j] = A[i][j] * mult;
			if (v[i] >= 0)
				v[i] += 1.0f;
			else
				v[i] -= 1.0f;
			invgamma = 1.0f / glm::abs (v[i]);
		}
		for (int j = i; j < 4; j++) {
			float dot = 0;
			for (int k = i; k < kRows; k++)
				dot += A[j][k] * v[k];
			for (int k = 0; k < kRows; k++)
				A[j][k] -= dot * invgamma * v[k];
		}
	}
	m_usedRows = 4;
}

}
