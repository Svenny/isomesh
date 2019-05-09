/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/qef/qef_solver_4d.hpp>

#include "householder.hpp"
#include "jacobi.hpp"

#include <cassert>
#include <limits>
#include <stdexcept>

namespace isomesh
{

QefSolver4D::QefSolver4D () noexcept {
	reset ();
}

QefSolver4D::QefSolver4D (const State &data) noexcept {
	reset ();
	merge (data);
}

void QefSolver4D::reset () noexcept {
	memset (A, 0, sizeof (A));
	m_pointsSum = glm::vec4 { 0 };
	m_pointsCount = 0;
	m_usedRows = 0;
	m_featureDim = 0;
}

void QefSolver4D::merge (const State &data) noexcept {
	// Mergee is of higher dimension, our mass point may be dismissed
	if (data.dim > m_featureDim) {
		m_featureDim = data.dim;
		m_pointsSum = glm::vec4 (data.mpx, data.mpy, data.mpz, data.mpw);
		m_pointsCount = data.mp_cnt;
	}
	// Mergee is of the same dimension, add mass points together
	else if (data.dim == m_featureDim) {
		m_pointsSum += glm::vec4 (data.mpx, data.mpy, data.mpz, data.mpw);
		m_pointsCount += data.mp_cnt;
	}
	// When mergee is of lesser dimension its mass point may be dismissed
	// We need five free rows
	if (m_usedRows > kMaxRows - 5)
		compressMatrix ();
	int id = m_usedRows;
	A[0][id] = data.a_11; A[1][id] = data.a_12; A[2][id] = data.a_13; A[3][id] = data.a_14; A[4][id] = data.b_1;
	id++;
	A[0][id] =         0; A[1][id] = data.a_22; A[2][id] = data.a_23; A[3][id] = data.a_24; A[4][id] = data.b_2;
	id++;
	A[0][id] =         0; A[1][id] =         0; A[2][id] = data.a_33; A[3][id] = data.a_34; A[4][id] = data.b_3;
	id++;
	A[0][id] =         0; A[1][id] =         0; A[2][id] =         0; A[3][id] = data.a_44; A[4][id] = data.b_4;
	id++;
	A[0][id] =         0; A[1][id] =         0; A[2][id] =         0; A[3][id] =         0; A[4][id] =  data.r2;
	m_usedRows += 5;
}

QefSolver4D::State QefSolver4D::state () noexcept {
	compressMatrix ();
	State data;
	data.a_11 = A[0][0]; data.a_12 = A[1][0]; data.a_13 = A[2][0]; data.a_14 = A[3][0]; data.b_1 = A[4][0];
	data.a_22 = A[1][1]; data.a_23 = A[2][1]; data.a_24 = A[3][1]; data.b_2 = A[4][1];
	data.a_33 = A[2][2]; data.a_34 = A[3][2]; data.b_3 =  A[4][2];
	data.a_44 = A[3][3]; data.b_4 = A[4][3];
	data.r2 = A[4][4];
	data.mpx = m_pointsSum.x;
	data.mpy = m_pointsSum.y;
	data.mpz = m_pointsSum.z;
	data.mpw = m_pointsSum.w;
	// Is it really safe to assume nobody will ever add 2^29 points? :P
	assert (m_pointsCount < (1 << 29));
	data.mp_cnt = m_pointsCount;
	data.dim = m_featureDim;
	return data;
}

void QefSolver4D::addPlane (glm::vec4 point, glm::vec4 normal) noexcept {
	if (m_usedRows == kMaxRows)
		compressMatrix ();
	int id = m_usedRows;
	A[0][id] = normal.x;
	A[1][id] = normal.y;
	A[2][id] = normal.z;
	A[3][id] = normal.w;
	A[4][id] = glm::dot (normal, point);
	m_usedRows++;
	m_pointsSum += point;
	m_pointsCount++;
}

float QefSolver4D::eval (glm::vec4 point) const noexcept {
	float error = 0;
	for (int i = 0; i < m_usedRows; i++) {
		float dot = A[0][i] * point.x + A[1][i] * point.y + A[2][i] * point.z + A[3][i] * point.w;
		float diff = dot - A[4][i];
		error += diff * diff;
	}
	return error;
}

glm::vec4 QefSolver4D::solve (glm::vec4 min_point, glm::vec4 max_point) {
	compressMatrix ();
	glm::mat4 AT, ATA;
	{
		glm::mat4 M;
		M[0] = glm::vec4 (A[0][0], A[0][1], A[0][2], A[0][3]);
		M[1] = glm::vec4 (A[1][0], A[1][1], A[1][2], A[1][3]);
		M[2] = glm::vec4 (A[2][0], A[2][1], A[2][2], A[2][3]);
		M[3] = glm::vec4 (A[3][0], A[3][1], A[3][2], A[3][3]);
		AT = glm::transpose (M);
		ATA = AT * M;
	}
	auto[e, E] = jacobi (ATA, m_jacobiTolerance, m_maxJacobiIters, m_useFastFormulas);
	glm::mat4 sigma { 0.0f };
	m_featureDim = 4;
	for (int i = 0; i < 4; i++) {
		if (abs (e[i]) >= m_pinvTolerance)
			sigma[i][i] = 1.0f / e[i];
		else m_featureDim--;
	}
	glm::mat4 ATAp = E * sigma * glm::transpose (E);
	glm::vec4 p = m_pointsSum / float (m_pointsCount);
	glm::vec4 b (A[4][0], A[4][1], A[4][2], A[4][3]);
	glm::vec4 c = ATAp * (AT * b - ATA * p);
	// TODO: replace this hack with proper bounded solver
	return glm::clamp (c + p, min_point, max_point);
}

void QefSolver4D::compressMatrix () noexcept {
	householder<float, 5, kMaxRows> (A, m_usedRows);
	m_usedRows = 5;
}

}
