/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** @file
 * @brief Interface for 3D QEF minimizer
 */
#pragma once

#include "../common.hpp"

namespace isomesh
{

/** \brief Interface for Quadratic Error Function (QEF) minimizer
*/
class QefSolver3D {
public:
	/** \brief QEF solver state for external storage
	
		This struct holds all data necessary to initialize QEF solver in
		compact form, suitable for storage in octree nodes.
	*/
	struct QefSolverState {
		// Compressed matrix, only nonzero elements
		float a_11, a_12, a_13, b_1;
		float       a_22, a_23, b_2;
		float             a_33, b_3;
		float                    r2;
		// Sum of added points
		float mpx, mpy, mpz;
		// Added points count and feature dimension
		// Using int16_t to fit them in four bytes (saving other four)
		int16_t mp_cnt, dim;
	};

	QefSolver3D () noexcept;
	QefSolver3D (const QefSolverState &data) noexcept;
	/** \brief Resets solver to its initial state
	*/
	void reset () noexcept;
	/** \brief Add data from external storage to solver state

		\param[in] data Data to be added
	*/
	void merge (const QefSolverState &data) noexcept;
	/** \brief Obtain solver state for external storage

		\return Solver state in compact form
	*/
	QefSolverState state () noexcept;
	/** \brief Adds a plane to the solver

		\param[in] normal Normal vector of the plane, must have unit length
		\param[in] point Any point belonging to the plane
	*/
	void addPlane (glm::vec3 point, glm::vec3 normal) noexcept;
	/** \brief Evaluates QEF value at a given point

		\param[in] point Point where QEF needs to be evaluated
		\return Error value
	*/
	float eval (glm::vec3 point) const noexcept;
	/** \brief Finds QEF minimizer

		Solution space is bounded by an axis-aligned box [minPoint; maxPoint]. An
		implementation may still return a value outside of this box, but this can break
		invariants in some algorithms. In case of multiple solutions we advise to prefer
		the one closest to the 'mass point' (centroid of all points added using
		\ref addPlane), making the problem always have a unique solution. This is not
		mandatory though.
		\param[in] minPoint Lower bound of solution space
		\param[in] maxPoint Upper bound of solution space
		\return Point which minimizes QEF value
	*/
	glm::vec3 solve (glm::vec3 min_point, glm::vec3 max_point);
	
	float pinvTolerance () const noexcept { return m_pinvTolerance; }
	float jacobiTolerance () const noexcept { return m_jacobiTolerance; }
	int maxJacobiIters () const noexcept { return m_maxJacobiIters; }
	bool fastFormulasUsed () const noexcept { return m_useFastFormulas; }
	
	void setPinvTolerance (float value) noexcept { m_pinvTolerance = glm::max (0.0f, value); }
	void setJacobiTolerance (float value) noexcept { m_jacobiTolerance = glm::max (0.0f, value); }
	void setMaxJacobiIters (int value) noexcept { m_maxJacobiIters = glm::max (1, value); }
	void useFastFormulas (bool value) noexcept { m_useFastFormulas = value; }

protected:
	/// Maximal number of used rows
	constexpr static int kMaxRows = 8;
	/** \brief Column-major matrix A* = (A b)
	*/
	float A[4][kMaxRows];
	/// Count of matrix rows occupied with useful data
	int m_usedRows;
	/// Algebraic sum of points added to the solver
	glm::vec3 m_pointsSum;
	/// Count of points added to the solver
	int m_pointsCount;
	/** \brief Feature dimension
	
		Minimizer space is a set of all points in solution space where the QEF value
		reaches its minimum (i.e. points which may be returned by \ref solve call).
		Feature dimension is defined as three minus dimension of minimizer space (three
		minus the number of singular values truncated to zero during computing the
		pseudoinverse matrix).
	*/
	int m_featureDim;
	/// Singular values with absolute value less than tolerance will be truncated to zero
	float m_pinvTolerance = 0.01f;
	/// Stopping condition in Jacobi eigenvalue algorithm
	float m_jacobiTolerance = 0.0025f;
	/// Maximal number of Jacobi eigenvalue algorithm iterations
	int m_maxJacobiIters = 15;
	/// Whether to use more accurate or faster formulae in Jacobi eigenvalue algorithm
	bool m_useFastFormulas = true;

	void compressMatrix () noexcept;
};

}
