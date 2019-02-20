/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include "../common.hpp"

namespace isomesh
{

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
		\return Point which minimizes QEF value
	*/
	virtual glm::vec3 solve (glm::vec3 minPoint, glm::vec3 maxPoint) = 0;
	/** \brief Evaluates QEF value at a given point

		\param[in] point Point where QEF needs to be evaluated
		\return Error value
	*/
	virtual float eval (glm::vec3 point) const = 0;
	/** \brief Resets solver to initial statte
	*/
	virtual void reset () = 0;
};

class BaseQefSolver3D : public QefSolver3D {
public:
	virtual void addPlane (glm::vec3 normal, glm::vec3 point) override;
	virtual float eval (glm::vec3 point) const override;
	virtual void reset () override;

	const static int kMaxPlanes = 12;
protected:
	glm::vec3 m_normals[kMaxPlanes];
	float m_coefs[kMaxPlanes];
	/// Sum of added points
	glm::vec3 m_massPoint = glm::vec3 (0);
	int m_numPlanes = 0;
};

class GradientDescentQefSolver3D : public BaseQefSolver3D {
public:
	virtual glm::vec3 solve (glm::vec3 minPoint, glm::vec3 maxPoint) override;
	
	void setStepCount (int value) { m_stepCount = value; }
	void setGradStep (float value) { m_gradStep = value; }

protected:
	int m_stepCount = 10;
	float m_gradStep = 0.75f;
};

}
