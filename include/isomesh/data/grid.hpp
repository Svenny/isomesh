/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Uniform grid data storage for use in algorithms
*/
#pragma once

#include "../common.hpp"
#include "grid_edge_storage.hpp"

#include <memory>

namespace isomesh
{

class ZeroFinder;
class MaterialSelector;

// YXZ traversal order (to match Voxen's layout)
// Local coordinates are [-size/2; size/2]
// Global coordinates define point position in the world
// Size must be a power of two (size >= 2 and size <= 128), this is needed
// to make adaptive compression algorithms (like octree) work on grids
/* Size can't be bigger than 256, it's a technical limitation. I use 28-bit surface-crossing edge
 indexing in some places, so the grid shouldn't have more than 2^28 surface-crossing edges. As the
 grid has at most 3*size*(size + 1)^2 edges, the maximal size meeting this limit is 446, therefore the
 largest suitable power of two is 256. However, grid sizes of 256 and higher are too impractical,
 therefore this limitation shouldn't cause any issues. */
class UniformGrid {
public:
	explicit UniformGrid (uint32_t size, const glm::dvec3 &globalPos = glm::dvec3 (0.0), double gridStep = 1.0);
	// Fills grid using given surface function & material selection method
	void fill (const SurfaceFunction &f, const ZeroFinder &solver, const MaterialSelector &material);
	// Converts local-coordinates point to YXZ-order traversal (raw) index
	uint32_t pointToRawIndex (int32_t x, int32_t y, int32_t z) const;
	uint32_t pointToRawIndex (const glm::ivec3 &v) const;
	// Converts YXZ-order traversal (raw) index to local-coordinates point
	glm::ivec3 rawIndexToPoint (uint32_t idx) const noexcept;
	// Local-coordinates indexing
	Material at (int32_t x, int32_t y, int32_t z) const;
	Material operator [] (const glm::ivec3 &v) const;
	// Raw indexing
	const Material *data () const noexcept { return m_mat.get (); }
	// Edge storages
	const UniformGridEdgeStorage &xEdges () const noexcept { return m_edgeX; }
	const UniformGridEdgeStorage &yEdges () const noexcept { return m_edgeY; }
	const UniformGridEdgeStorage &zEdges () const noexcept { return m_edgeZ; }
	// Properties
	uint32_t dataSize () const noexcept { return (m_size + 1) * (m_size + 1) * (m_size + 1); }
	uint32_t gridSize () const noexcept { return m_size; }
	int32_t maxCoord () const noexcept { return m_halfSize; }
	int32_t minCoord () const noexcept { return -m_halfSize; }
	glm::dvec3 globalPosition () const noexcept { return m_globalPos; }
	double gridStep () const noexcept { return m_gridStep; }
	// Coordinate system conversions
	glm::dvec3 localToGlobal (const glm::dvec3 &L) const noexcept { return L * m_gridStep + m_globalPos; }
	glm::dvec3 globalToLocal (const glm::dvec3 &G) const noexcept { return (G - m_globalPos) / m_gridStep; }
private:
	const uint32_t m_size;
	const int32_t m_halfSize;

	std::unique_ptr<Material[]> m_mat;
	
	UniformGridEdgeStorage m_edgeX, m_edgeY, m_edgeZ;

	glm::dvec3 m_globalPos;
	double m_gridStep;
	
	// Checks whether point (x,y,z) lies in valid local coordinates
	void checkPoint (int32_t x, int32_t y, int32_t z) const;
};

}
