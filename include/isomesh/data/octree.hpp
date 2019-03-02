/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Octree data storage for use in algorithms
*/
#pragma once

#include "../common.hpp"
#include "../data/mesh.hpp"
#include "../field/scalar_field.hpp"
#include "../qef/qef_solver_4d.hpp"
#include "../util/material_selector.hpp"
#include "octree_node.hpp"

namespace isomesh
{

// TODO: make generic octree
class DMC_Octree {
public:
	explicit DMC_Octree (int32_t root_size, glm::dvec3 global_pos = glm::dvec3 (0), double global_scale = 1);
	~DMC_Octree () noexcept;
	
	void build (const ScalarField &field, const MaterialSelector &material,
	            QefSolver4D &solver, const float epsilon);
	Mesh contour () const;
private:
	const glm::dvec3 m_globalPos;
	const double m_globalScale;
	const int32_t m_rootSize;
	
	DMC_OctreeNode *root;

	
};

}
