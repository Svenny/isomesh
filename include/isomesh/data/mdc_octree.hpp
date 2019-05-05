/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Octree for MDC algorithm
*/
#pragma once

#include "../common.hpp"
#include "../data/mesh.hpp"
#include "../data/grid.hpp"
#include "../qef/qef_solver_3d.hpp"
#include "mdc_octree_node.hpp"

namespace isomesh
{

// TODO: make generic octree
class MDC_Octree {
public:
	explicit MDC_Octree (int32_t root_size, glm::dvec3 global_pos = glm::dvec3 (0), double global_scale = 1);
	
	void build (const UniformGrid &G, QefSolver3D &solver);
	Mesh contour (float epsilon);
	
	// Mappings between local and global coordinate spaces
	glm::dvec3 localToGlobal (const glm::dvec3 &L) const noexcept { return L * m_globalScale + m_globalPos; }
	glm::dvec3 globalToLocal (const glm::dvec3 &G) const noexcept { return (G - m_globalPos) / m_globalScale; }
private:
	const glm::dvec3 m_globalPos;
	const double m_globalScale;
	const int32_t m_rootSize;

	MDC_OctreeNode m_root;
	
	struct BuildArgs {
		const UniformGrid &grid;
		QefSolver3D &solver;
	};

	void buildNode (MDC_OctreeNode *node, glm::ivec3 min_corner, int32_t size, BuildArgs &args);
	void buildLeaf (MDC_OctreeNode *node, glm::ivec3 min_corner, int32_t size, BuildArgs &args);
};

}
