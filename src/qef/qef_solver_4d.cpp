/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <isomesh/qef/qef_solver_4d.hpp>

#include <stdexcept>

namespace isomesh
{

void BaseQefSolver4D::addPlane (glm::vec4 point, glm::vec4 normal) {
	if (m_numPlanes >= kMaxPlanes)
		throw std::runtime_error ("Maximal number of planes exceeded");
	m_normals[m_numPlanes] = normal;
	m_coefs[m_numPlanes] = glm::dot (normal, point);
	m_massPoint += point;
	m_numPlanes++;
}

float BaseQefSolver4D::eval (glm::vec4 point) const {
	float error = 0;
	for (int i = 0; i < m_numPlanes; i++) {
		float diff = glm::dot (m_normals[i], point) - m_coefs[i];
		error += diff * diff;
	}
	return error;
}

void BaseQefSolver4D::reset () {
	m_massPoint = glm::vec4 (0);
	m_numPlanes = 0;
}

}
