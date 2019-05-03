/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/qef/qef_solver_3d.hpp>

#include "householder.hpp"
#include "jacobi.hpp"

#include <cassert>
#include <limits>
#include <stdexcept>

namespace isomesh
{

QefSolver3D::QefSolver3D () noexcept {
	reset ();
}

QefSolver3D::QefSolver3D (const State &data) noexcept {
	reset ();
	merge (data);
}

void QefSolver3D::reset () noexcept {
	memset (A, 0, sizeof (A));
	m_pointsSum = glm::vec3 { 0 };
	m_pointsCount = 0;
	m_usedRows = 0;
	m_featureDim = 0;
}

void QefSolver3D::merge (const State &data) noexcept {
	// Mergee is of higher dimension, our mass point may be dismissed
	if (data.dim > m_featureDim) {
		m_featureDim = data.dim;
		m_pointsSum = glm::vec3 (data.mpx, data.mpy, data.mpz);
		m_pointsCount = data.mp_cnt;
	}
	// Mergee is of the same dimension, add mass points together
	else if (data.dim == m_featureDim) {
		m_pointsSum += glm::vec3 (data.mpx, data.mpy, data.mpz);
		m_pointsCount += data.mp_cnt;
	}
	// When mergee is of lesser dimension its mass point may be dismissed
	// We need four free rows
	if (m_usedRows > kMaxRows - 4)
		compressMatrix ();
	int id = m_usedRows;
	A[0][id] = data.a_11; A[1][id] = data.a_12; A[2][id] = data.a_13; A[3][id] = data.b_1;
	id++;
	A[0][id] =         0; A[1][id] = data.a_22; A[2][id] = data.a_23; A[3][id] = data.b_2;
	id++;
	A[0][id] =         0; A[1][id] =         0; A[2][id] = data.a_33; A[3][id] = data.b_3;
	id++;
	A[0][id] =         0; A[1][id] =         0; A[2][id] =         0; A[3][id] =  data.r2;
	m_usedRows += 4;
}

QefSolver3D::State QefSolver3D::state () noexcept {
	compressMatrix ();
	State data;
	data.a_11 = A[0][0]; data.a_12 = A[1][0]; data.a_13 = A[2][0]; data.b_1 = A[3][0];
	data.a_22 = A[1][1]; data.a_23 = A[2][1]; data.b_2 = A[3][1];
	data.a_33 = A[2][2]; data.b_3 = A[3][2];
	data.r2 = A[3][3];
	data.mpx = m_pointsSum.x;
	data.mpy = m_pointsSum.y;
	data.mpz = m_pointsSum.z;
	// Is it really safe to assume nobody will ever add 2^30 points? :P
	assert (m_pointsCount < (1 << 30));
	data.mp_cnt = m_pointsCount;
	data.dim = m_featureDim;
	return data;
}

void QefSolver3D::addPlane (glm::vec3 point, glm::vec3 normal) noexcept {
	if (m_usedRows == kMaxRows)
		compressMatrix ();
	int id = m_usedRows;
	A[0][id] = normal.x;
	A[1][id] = normal.y;
	A[2][id] = normal.z;
	A[3][id] = glm::dot (normal, point);
	m_usedRows++;
	m_pointsSum += point;
	m_pointsCount++;
}

float QefSolver3D::eval (glm::vec3 point) const noexcept {
	float error = 0;
	for (int i = 0; i < m_usedRows; i++) {
		float dot = A[0][i] * point.x + A[1][i] * point.y + A[2][i] * point.z;
		float diff = dot - A[3][i];
		error += diff * diff;
	}
	return error;
}

glm::vec3 QefSolver3D::solve (glm::vec3 min_point, glm::vec3 max_point) {
	compressMatrix ();
	glm::mat3 AT, ATA;
	{
		glm::mat3 M;
		M[0] = glm::vec3 (A[0][0], A[0][1], A[0][2]);
		M[1] = glm::vec3 (A[1][0], A[1][1], A[1][2]);
		M[2] = glm::vec3 (A[2][0], A[2][1], A[2][2]);
		AT = glm::transpose (M);
		ATA = AT * M;
	}
	auto[e, E] = jacobi (ATA, m_jacobiTolerance, m_maxJacobiIters, m_useFastFormulas);
	glm::mat3 sigma { 0.0f };
	m_featureDim = 3;
	for (int i = 0; i < 3; i++) {
		if (abs (e[i]) >= m_pinvTolerance)
			sigma[i][i] = 1.0f / e[i];
		else m_featureDim--;
	}
	glm::mat3 ATAp = E * sigma * glm::transpose (E);
	glm::vec3 p = m_pointsSum / float (m_pointsCount);
	glm::vec3 b (A[3][0], A[3][1], A[3][2]);
	glm::vec3 c = ATAp * (AT * b - ATA * p);
	return glm::clamp (c + p, min_point, max_point);
}

void QefSolver3D::compressMatrix () noexcept {
	householder<float, 4, kMaxRows> (A, m_usedRows);
	m_usedRows = 4;
}

}
