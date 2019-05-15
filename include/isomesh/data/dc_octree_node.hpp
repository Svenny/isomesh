/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** @file
 * @brief Node of DC octree
 */
#pragma once

#include "../common.hpp"
#include "../qef/qef_solver_3d.hpp"

#include <array>

namespace isomesh 
{

struct DC_OctreeNode {
	DC_OctreeNode () noexcept;
	~DC_OctreeNode () noexcept;

	void subdivide ();
	void collapse ();
	bool isSubdivided () const noexcept { return !is_leaf; }
	bool isHomogenous () const noexcept;
	int16_t depth () const noexcept { return m_depth; }

	DC_OctreeNode *operator[] (int num) const noexcept { return children[num]; }

	union {
		struct {
			glm::vec3 dual_vertex;
			glm::vec3 normal;
			std::array<Material, 8> corners;
			QefSolver3D::State qef;
			uint32_t vertex_id;
		} leaf_data;
		DC_OctreeNode *children[8];
	};

private:
	bool is_leaf;
	int16_t m_depth = 0;
};

}
