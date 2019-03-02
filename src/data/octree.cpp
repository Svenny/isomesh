/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/octree.hpp>

#include <stdexcept>

namespace isomesh
{

DMC_Octree::DMC_Octree (int32_t root_size, glm::dvec3 global_pos, double global_scale) :
	m_rootSize (root_size), m_globalPos (global_pos), m_globalScale (global_scale) {
	if (root_size <= 0 || (root_size & (root_size - 1)))
		throw std::invalid_argument ("Octree size is not a power of two");
	root = new DMC_OctreeNode;
}

DMC_Octree::~DMC_Octree () noexcept {
	delete root;
}

void DMC_Octree::build (const ScalarField &field, const MaterialSelector &material,
                        QefSolver4D &solver, const float epsilon) {
	// TODO
}

Mesh DMC_Octree::contour () const {
	Mesh mesh;
	// TODO
	return mesh;
}

}
