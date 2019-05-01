/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Node of MDC octree
*/
#pragma once

#include "../common.hpp"
#include "../qef/qef_solver_3d.hpp"

#include <vector>

namespace isomesh
{

struct MDC_Vertex {
	MDC_Vertex () noexcept : normal (0), parent (nullptr) {}
	glm::vec3 dual_vertex;
	glm::vec3 normal;
	QefSolver3D::QefSolverState qef;
	uint32_t vertex_id;
	uint16_t edge_mask;
	bool collapsible;
	MDC_Vertex *parent;
};

struct MDC_OctreeNode {
	MDC_OctreeNode () noexcept;
	~MDC_OctreeNode () noexcept;
	
	void subdivide ();
	void collapse () noexcept;
	bool isSubdivided () const noexcept { return m_children != nullptr; }
	bool isLeaf () const noexcept { return m_children == nullptr; }
	int16_t depth () const noexcept { return m_depth; }
	
	MDC_OctreeNode *child (int num) const noexcept { return m_children + num; }
	MDC_OctreeNode *operator[] (int num) const noexcept { return m_children + num; }
	
	std::vector<MDC_Vertex> vertices;
	
	inline const static glm::ivec3 kCornerOffset[8] = {
		glm::ivec3 (0, 0, 0),
		glm::ivec3 (0, 0, 1),
		glm::ivec3 (1, 0, 0),
		glm::ivec3 (1, 0, 1),
		glm::ivec3 (0, 1, 0),
		glm::ivec3 (0, 1, 1),
		glm::ivec3 (1, 1, 0),
		glm::ivec3 (1, 1, 1)
	};
private:
	MDC_OctreeNode *m_children;
	int16_t m_depth = 0;
};

}
