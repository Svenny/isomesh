/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** @file
 * @brief Interface for 4D QEF minimizer
 */
#pragma once

#include "../common.hpp"

namespace isomesh
{

/** \brief Interface for Quadratic Error Function (QEF) minimizer
*/
class QefSolver4D {
public:
	/** \brief QEF solver state for external storage
	
		This struct holds solver's internal state in compact form, suitable
		for storage in octree nodes. You may use this struct to merge QEF's
		together or to initialize another solver.
		\note Solver's tunable options, such as tolerance or maximal number
		of iterations, are not preserved. You need to used some other way
		if you plan to store these options externally.
	*/
	struct State {
		// Compressed matrix, only nonzero elements
		float a_11, a_12, a_13, a_14, b_1;
		float       a_22, a_23, a_24, b_2;
		float             a_33, a_34, b_3;
		float                   a_44, b_4;
		float                          r2;
		// Sum of added points
		float mpx, mpy, mpz, mpw;
		// Added points count and feature dimension
		// Using bit fields to fit them in four bytes, saving other four
		uint32_t mp_cnt : 29;
		uint32_t dim : 3;
	};

	QefSolver4D () noexcept;
	QefSolver4D (const State &data) noexcept;
	/** \brief Resets solver to initial state
	*/
	void reset () noexcept;
	/** \brief Adds data from external storage to the solver state
	*/
	void merge (const State &data) noexcept;
	/** \brief Obtains solver state for external storage

		\return Solver state in compact form
	*/
	State state () noexcept;
	/** \brief Adds a plane to the solver

		\param[in] normal Normal vector of the plane, must have unit length
		\param[in] point Any point belonging to the plane
	*/
	void addPlane (glm::vec4 point, glm::vec4 normal) noexcept;
	/** \brief Evaluates QEF value at a given point

		\param[in] point Point where QEF needs to be evaluated
		\return Error value
	*/
	float eval (glm::vec4 point) const noexcept;
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
	glm::vec4 solve (glm::vec4 minPoint, glm::vec4 maxPoint);
	
	float pinvTolerance () const noexcept { return m_pinvTolerance; }
	float jacobiTolerance () const noexcept { return m_jacobiTolerance; }
	int maxJacobiIters () const noexcept { return m_maxJacobiIters; }
	bool fastFormulasUsed () const noexcept { return m_useFastFormulas; }
	
	void setPinvTolerance (float value) noexcept { m_pinvTolerance = glm::max (0.0f, value); }
	void setJacobiTolerance (float value) noexcept { m_jacobiTolerance = glm::max (0.0f, value); }
	void setMaxJacobiIters (int value) noexcept { m_maxJacobiIters = glm::max (1, value); }
	void useFastFormulas (bool value) noexcept { m_useFastFormulas = value; }
	
private:
	/// Maximal number of used rows
	constexpr static int kMaxRows = 16;
	/** \brief Column-major matrix A* = (A b)
	*/
	float A[5][kMaxRows];
	/// Count of matrix rows occupied with useful data
	int m_usedRows;
	/// Algebraic sum of points added to the solver
	glm::vec4 m_pointsSum;
	/// Count of points added to the solver
	uint32_t m_pointsCount;
	/** \brief Feature dimension
	
		Minimizer space is a set of all points in solution space where the QEF value
		reaches its minimum (i.e. points which may be returned by \ref solve call).
		Feature dimension is defined as three minus dimension of minimizer space (three
		minus the number of singular values truncated to zero during computing the
		pseudoinverse matrix).
	*/
	uint32_t m_featureDim;
	/// Singular values with absolute value less than tolerance will be truncated to zero
	float m_pinvTolerance = 0.01f;
	/// Stopping condition in Jacobi eigenvalue algorithm
	float m_jacobiTolerance = 0.0025f;
	/// Maximal number of Jacobi eigenvalue algorithm iterations
	int m_maxJacobiIters = 20;
	/// Whether to use more accurate or faster formulae in Jacobi eigenvalue algorithm
	bool m_useFastFormulas = true;
	
	void compressMatrix () noexcept;
};

}
