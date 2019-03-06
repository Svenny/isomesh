/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include "../common.hpp"

namespace isomesh
{

/** \brief Interface for Quadratic Error Function (QEF) minimizer
*/
class QefSolver4D {
public:
	/** \brief Adds a plane to the solver
	
		\param[in] normal Normal vector of the plane, must have unit length
		\param[in] point Any point belonging to the plane
	*/
	virtual void addPlane (glm::vec4 point, glm::vec4 normal) = 0;
	/** \brief Finds QEF minimizer
	
		Solution space is limited by a box [minPoint; maxPoint] to prevent 'spikes'
		in the isosurface. You may still return a value outside of this box, just
		remember of the possible consequences. In case of multiple solutions we advise
		to prefer the one closest to the 'mass point' (average of all points added
		through \ref addPlane). This makes the problem always have unique solution.
		\param[in] minPoint Lower bound of solution space
		\param[in] maxPoint Upper bound of solution space
		\return Point which minimizes QEF value
	*/
	virtual glm::vec4 solve (glm::vec4 minPoint, glm::vec4 maxPoint) = 0;
	/** \brief Evaluates QEF value at a given point

		\param[in] point Point where QEF needs to be evaluated
		\return Error value
	*/
	virtual float eval (glm::vec4 point) const = 0;
	/** \brief Resets solver to initial state
	*/
	virtual void reset () = 0;
};

class BaseQefSolver4D : public QefSolver4D {
public:
	virtual void addPlane (glm::vec4 point, glm::vec4 normal) override;
	virtual float eval (glm::vec4 point) const override;
	virtual void reset () override;

	const static int kMaxPlanes = 200;
protected:
	glm::vec4 m_normals[kMaxPlanes];
	float m_coefs[kMaxPlanes];
	/// Sum of added points
	glm::vec4 m_massPoint = glm::vec4 (0);
	int m_numPlanes = 0;
};

class GradientDescentQefSolver4D : public BaseQefSolver4D {
public:
	virtual glm::vec4 solve (glm::vec4 minPoint, glm::vec4 maxPoint) override;
	
	void setStepCount (int value) { m_stepCount = value; }
	void setGradStep (float value) { m_gradStep = value; }

protected:
	int m_stepCount = 20;
	float m_gradStep = 0.25f;
};

}
