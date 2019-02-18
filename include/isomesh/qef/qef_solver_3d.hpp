/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include "../common.hpp"

namespace isomesh
{

struct QefSolution3D {
	glm::vec3 point;
	float error;
};

/** \brief Interface for Quadratic Error Function (QEF) minimizer
*/
class QefSolver3D {
public:
	/** \brief Adds a plane to the solver
	
		\param[in] normal Normal vector of the plane, must have unit length
		\param[in] point Any point belonging to the plane
	*/
	virtual void addPlane (glm::vec3 normal, glm::vec3 point) = 0;
	/** \brief Solves QEF problem
	
		\param[in] minPoint Lower bound of solution space
		\param[in] maxPoint Upper bound of solution space
		\return Found solution and QEF value in that point
	*/
	virtual QefSolution3D solve (glm::vec3 minPoint, glm::vec3 maxPoint) = 0;
};

class BaseQefSolver3D : public QefSolver3D {
public:
	virtual void addPlane (glm::vec3 normal, glm::vec3 point) final override;
	
	const static int kMaxPlanes = 12;
protected:
	glm::vec3 m_normals[kMaxPlanes];
	float m_coefs[kMaxPlanes];
	glm::vec3 m_massPoint = glm::vec3 (0);
	int m_numPlanes = 0;
};

class GradientDescentQefSolver3D : public BaseQefSolver3D {
public:
	
protected:
	int m_stepCount = 10;
};

}
