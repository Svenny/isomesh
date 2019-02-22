/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <isomesh/qef/qef_solver_3d.hpp>

#include <stdexcept>

namespace isomesh
{

void BaseQefSolver3D::addPlane (glm::vec3 point, glm::vec3 normal) {
	if (m_numPlanes >= kMaxPlanes)
		throw std::runtime_error ("Maximal number of planes exceeded");
	m_normals[m_numPlanes] = normal;
	m_coefs[m_numPlanes] = glm::dot (normal, point);
	m_massPoint += point;
	m_numPlanes++;
}

float BaseQefSolver3D::eval (glm::vec3 point) const {
	float error = 0;
	for (int i = 0; i < m_numPlanes; i++) {
		float diff = glm::dot (m_normals[i], point) - m_coefs[i];
		error += diff * diff;
	}
	return error;
}

void BaseQefSolver3D::reset () {
	m_massPoint = glm::vec3 (0);
	m_numPlanes = 0;
}

}
