/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/octree.hpp>

#include <cassert>
#include <limits>
#include <stdexcept>

namespace isomesh
{

DMC_Octree::DMC_Octree (int32_t root_size, glm::dvec3 global_pos, double global_scale) :
	m_rootSize (root_size), m_globalPos (global_pos), m_globalScale (global_scale) {
	if (root_size <= 0 || (root_size & (root_size - 1)))
		throw std::invalid_argument ("Octree size is not a power of two");
	m_root = nullptr;
}

DMC_Octree::~DMC_Octree () noexcept {
	delete m_root;
}

void DMC_Octree::build (const ScalarField &field, const MaterialSelector &material,
                        QefSolver4D &solver, float epsilon) {
	glm::ivec3 min_corner = glm::ivec3 (-m_rootSize) / 2;
	BuildArgs args { field, material, solver, epsilon };
	try {
		delete m_root;
		m_root = new DMC_OctreeNode (min_corner, m_rootSize);
		buildNode (m_root, args);
	}
	catch (...) {
		auto e = std::current_exception ();
		delete m_root;
		m_root = nullptr;
		std::rethrow_exception (e);
	}
}

Mesh DMC_Octree::contour () const {
	Mesh mesh;
	// TODO
	return mesh;
}

void DMC_Octree::buildNode (DMC_OctreeNode *node, BuildArgs &args) {
	assert (node);
	// Perform unconditional subdivision up to some depth
	if (node->size > 1 && node->size * 8 >= m_rootSize) {
		node->subdivide ();
		for (int i = 0; i < 8; i++)
			buildNode ((*node)[i], args);
		return;
	}
	// Sample field at some points
	double offset_local = double (node->size) * 0.8;
	glm::dvec3 base_local = glm::dvec3 (node->minCorner) + 0.5 * (1.0 - offset_local);
	double offset_global = offset_local * m_globalScale;
	glm::dvec3 base_global = localToGlobal (base_local);
	const glm::dvec3 offset_local_table[8] = {
		glm::dvec3 (0, 0, 0) * offset_local,
		glm::dvec3 (0, 0, 1) * offset_local,
		glm::dvec3 (1, 0, 0) * offset_local,
		glm::dvec3 (1, 0, 1) * offset_local,
		glm::dvec3 (0, 1, 0) * offset_local,
		glm::dvec3 (0, 1, 1) * offset_local,
		glm::dvec3 (1, 1, 0) * offset_local,
		glm::dvec3 (1, 1, 1) * offset_local
	};
	const glm::dvec3 offset_global_table[8] = {
		glm::dvec3 (0, 0, 0) * offset_global,
		glm::dvec3 (0, 0, 1) * offset_global,
		glm::dvec3 (1, 0, 0) * offset_global,
		glm::dvec3 (1, 0, 1) * offset_global,
		glm::dvec3 (0, 1, 0) * offset_global,
		glm::dvec3 (0, 1, 1) * offset_global,
		glm::dvec3 (1, 1, 0) * offset_global,
		glm::dvec3 (1, 1, 1) * offset_global
	};
	QefSolver4D &solver = args.solver;
	const ScalarField &field = args.field;
	solver.reset ();
	for (int i = 0; i < 8; i++) {
		glm::dvec3 p = base_global + offset_global_table[i];
		float value = float (field (p));
		glm::vec3 grad = glm::vec3 (field.grad (p));
		glm::vec3 p_f = glm::vec3 (base_local + offset_local_table[i]);
		glm::vec4 point (p_f, value);
		glm::vec4 normal (grad, -1.0f);
		solver.addPlane (point, normal);
	}
	constexpr float min_value = std::numeric_limits<float>::lowest ();
	constexpr float max_value = std::numeric_limits<float>::max ();
	glm::vec4 lower_bound (node->minCorner, min_value);
	glm::vec4 upper_bound (node->minCorner + node->size, max_value);
	upper_bound.w = max_value;
	node->dualVertex = solver.solve (lower_bound, upper_bound);
	float error = solver.eval (node->dualVertex);
	if (node->size > 1 && error > args.epsilon) {
		node->subdivide ();
		for (int i = 0; i < 8; i++)
			buildNode ((*node)[i], args);
	}
}

}
