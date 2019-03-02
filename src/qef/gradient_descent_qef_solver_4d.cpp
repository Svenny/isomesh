/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <isomesh/qef/qef_solver_4d.hpp>

#include <stdexcept>

namespace isomesh
{

glm::vec4 GradientDescentQefSolver4D::solve (glm::vec4 minPoint, glm::vec4 maxPoint) {
	if (m_numPlanes == 0)
		throw std::runtime_error ("Solver has no input");
	// For division by planes count
	float divisor = 1.0f / float (m_numPlanes);
	// Average of added points
	glm::vec4 MP = m_massPoint * divisor;
	// Solution point
	glm::vec4 P = MP;
	for (int i = 0; i < m_stepCount; i++) {
		glm::vec4 grad (0);
		for (int j = 0; j < m_numPlanes; j++) {
			float coef = 2.0f * (glm::dot (m_normals[j], P) - m_coefs[j]);
			grad += coef * divisor * m_normals[j];
		}
		P -= m_gradStep * grad;
		P = glm::clamp (P, minPoint, maxPoint);
	}
	return P;
}

}
