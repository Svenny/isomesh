/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Uniform grid data storage for use in algorithms
*/
#pragma once

#include "../common.hpp"
#include "../field/scalar_field.hpp"
#include "../util/zero_finder.hpp"
#include "grid_edge_storage.hpp"

#include <array>
#include <memory>

namespace isomesh
{

// YXZ traversal order (to match Voxen's layout)
// Local coordinates are [-size/2; size/2]
// Global coordinates define point position in the world
// Size must be a power of two (size >= 2 and size <= 1024), this is needed
// to make adaptive compression algorithms (like octree) work on grids
/* Size can't be bigger than 1024, because using larger sizes will overflow 32-bit integers, which are
 used everywhere throughout the library. However, grid sizes of 256 and higher are impractical because of
 their huge RAM and computing power requirements, therefore this limitation shouldn't cause any issues.
 Running some algorithm on a 1024 grid will require ~10 GB (!) of RAM (and nearly infinite time if your
 computer runs out of RAM and begins using swapfile). */
class UniformGrid {
public:
	explicit UniformGrid (uint32_t size, const glm::dvec3 &globalPos = glm::dvec3 (0.0), double gridStep = 1.0);
	/** \brief Fills the grid using provided scalar field
	
		\param[in] field Scalar field to sample data from
		\param[in] solver Solver to find zeros along grid edges
		\param[in] material Material selector
	*/
	void fill (const ScalarField &field, const ZeroFinder &solver);
	// Local-coordinates indexing
	Material at (int32_t x, int32_t y, int32_t z) const;
	Material operator [] (const glm::ivec3 &v) const;
	// Raw indexing
	const Material *data () const noexcept { return m_mat.get (); }
	// Properties
	uint32_t dataSize () const noexcept { return (m_size + 1) * (m_size + 1) * (m_size + 1); }
	uint32_t gridSize () const noexcept { return m_size; }
	int32_t maxCoord () const noexcept { return m_halfSize; }
	int32_t minCoord () const noexcept { return -m_halfSize; }
	glm::dvec3 globalPosition () const noexcept { return m_globalPos; }
	double gridStep () const noexcept { return m_gridStep; }
	// Mappings between local and global coordinate spaces
	glm::dvec3 localToGlobal (const glm::dvec3 &L) const noexcept { return L * m_gridStep + m_globalPos; }
	glm::dvec3 globalToLocal (const glm::dvec3 &G) const noexcept { return (G - m_globalPos) / m_gridStep; }

	// ---------------------------------------
	// -------------- NEW API ----------------
	// ---------------------------------------

	uint32_t pointToIndex (int32_t x, int32_t y, int32_t z) const noexcept;
	uint32_t pointToIndex (const glm::ivec3 &p) const noexcept { return pointToIndex (p.x, p.y, p.z); }
	glm::ivec3 indexToPoint (uint32_t idx) const noexcept;
	// -------------------------
	// Operations on vertices
	// -------------------------
	bool isVertexInGrid (int32_t x, int32_t y, int32_t z) const noexcept;
	bool isVertexInGrid (const glm::ivec3 &vtxPos) const noexcept {
		return isVertexInGrid (vtxPos.x, vtxPos.y, vtxPos.z);
	}

	bool isVertexOnBorder (int32_t x, int32_t y, int32_t z) const noexcept;
	bool isVertexOnBorder (const glm::ivec3 &vtxPos) const noexcept {
		return isVertexOnBorder (vtxPos.x, vtxPos.y, vtxPos.z);
	}
	// -----------------------
	// Operations on edges
	// -----------------------
	template<int D>
	bool isEdgeInGrid (const glm::ivec3 &edgePos) const noexcept;
	// Returns whether an edge lies on the grid border, i.e. it has less than four adjacent cells
	template<int D>
	bool isEdgeOnBorder (const glm::ivec3 &edgePos) const noexcept;
	// Returns indices of (up to four) cells adjacent to this edge
	template<int D>
	std::array<uint32_t, 4> adjacentCellsForEdge (const glm::ivec3 &edgePos) const noexcept;
	// Returns indices of two endpoints of an edge
	template<int D>
	std::array<uint32_t, 2> adjacentVerticesForEdge (const glm::ivec3 &edgePos) const noexcept;
	// -----------------------
	// Operations on faces
	// -----------------------
	template<int D>
	bool isFaceInGrid (const glm::ivec3 &facePos) const noexcept;
	template<int D>
	bool isFaceOnBorder (const glm::ivec3 &facePos) const noexcept;
	// -----------------------
	// Operations on cells
	// -----------------------
	bool isCellInGrid (const glm::ivec3 &cellPos) const noexcept;
	bool isCellOnBorder (const glm::ivec3 &cellPos) const noexcept;
	std::array<uint32_t, 8> adjacentVerticesForCell (uint32_t cellIdx) const noexcept;
	std::array<Material, 8> materialsOfCell (uint32_t cellIdx) const noexcept;
	// Edge storages access
	template<int D> const UniformGridEdgeStorage &edges () const noexcept;

	/// This index marks a vertex/edge/cell that does not exist
	constexpr static uint32_t kBadIndex = ~uint32_t (0);
private:
	const uint32_t m_size;
	const int32_t m_halfSize;

	std::unique_ptr<Material[]> m_mat;
	
	UniformGridEdgeStorage m_edgeX, m_edgeY, m_edgeZ;

	glm::dvec3 m_globalPos;
	double m_gridStep;
};

template<>
bool UniformGrid::isEdgeInGrid<0> (const glm::ivec3 &edgePos) const noexcept;
template<>
bool UniformGrid::isEdgeInGrid<1> (const glm::ivec3 &edgePos) const noexcept;
template<>
bool UniformGrid::isEdgeInGrid<2> (const glm::ivec3 &edgePos) const noexcept;

template<>
bool UniformGrid::isEdgeOnBorder<0> (const glm::ivec3 &edgePos) const noexcept;
template<>
bool UniformGrid::isEdgeOnBorder<1> (const glm::ivec3 &edgePos) const noexcept;
template<>
bool UniformGrid::isEdgeOnBorder<2> (const glm::ivec3 &edgePos) const noexcept;

template<>
std::array<uint32_t, 4> UniformGrid::adjacentCellsForEdge<0> (const glm::ivec3 &edgePos) const noexcept;
template<>
std::array<uint32_t, 4> UniformGrid::adjacentCellsForEdge<1> (const glm::ivec3 &edgePos) const noexcept;
template<>
std::array<uint32_t, 4> UniformGrid::adjacentCellsForEdge<2> (const glm::ivec3 &edgePos) const noexcept;

template<>
std::array<uint32_t, 2> UniformGrid::adjacentVerticesForEdge<0> (const glm::ivec3 &edgePos) const noexcept;
template<>
std::array<uint32_t, 2> UniformGrid::adjacentVerticesForEdge<1> (const glm::ivec3 &edgePos) const noexcept;
template<>
std::array<uint32_t, 2> UniformGrid::adjacentVerticesForEdge<2> (const glm::ivec3 &edgePos) const noexcept;

template<>
bool UniformGrid::isFaceInGrid<0> (const glm::ivec3 &facePos) const noexcept;
template<>
bool UniformGrid::isFaceInGrid<1> (const glm::ivec3 &facePos) const noexcept;
template<>
bool UniformGrid::isFaceInGrid<2> (const glm::ivec3 &facePos) const noexcept;

template<>
bool UniformGrid::isFaceOnBorder<0> (const glm::ivec3 &facePos) const noexcept;
template<>
bool UniformGrid::isFaceOnBorder<1> (const glm::ivec3 &facePos) const noexcept;
template<>
bool UniformGrid::isFaceOnBorder<2> (const glm::ivec3 &facePos) const noexcept;

template<>
inline const UniformGridEdgeStorage &UniformGrid::edges<0> () const noexcept { return m_edgeX; }
template<>
inline const UniformGridEdgeStorage &UniformGrid::edges<1> () const noexcept { return m_edgeY; }
template<>
inline const UniformGridEdgeStorage &UniformGrid::edges<2> () const noexcept { return m_edgeZ; }

}
