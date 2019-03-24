/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/octree.hpp>

#include <cassert>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <glm/gtc/constants.hpp>

#include "../algo/marching_cubes_tables.h"

namespace isomesh
{

DMC_Octree::DMC_Octree (int32_t root_size, glm::dvec3 global_pos, double global_scale) :
	m_globalPos (global_pos), m_globalScale (global_scale), m_rootSize (root_size) {
	if (root_size <= 0 || (root_size & (root_size - 1)))
		throw std::invalid_argument ("Octree size is not a power of two");
}

void DMC_Octree::build (const ScalarField &field, const MaterialSelector &material,
                        QefSolver4D &solver, float epsilon, bool use_simple_split_policy,
                        bool use_random_sampling, bool use_early_split_stop, uint32_t seed) {
	// Scale epsilon according to QEF scale (when translating from global coordinates to
	// local QEF value is scaled by 1/(scale^2))
	float scaled_epsilon = epsilon / float (m_globalScale * m_globalScale);
	BuildArgs args { field, material, solver, scaled_epsilon, use_simple_split_policy,
	                 use_random_sampling, use_early_split_stop, std::mt19937 (seed) };
	try {
		m_root.collapse ();
		glm::ivec3 min_corner (-m_rootSize / 2);
		buildNode (&m_root, min_corner, m_rootSize, args);
	}
	catch (...) {
		auto e = std::current_exception ();
		m_root.collapse ();
		std::rethrow_exception (e);
	}
}

namespace dmc_detail
{

struct NodeHash {
	size_t operator ()(const std::pair<const DMC_OctreeNode *, const DMC_OctreeNode *> &node) const noexcept {
		uintptr_t h1 = uintptr_t (node.first);
		uintptr_t h2 = uintptr_t (node.second);
		return h1 ^ (h2 << 1);
	}
};

using VertexMap = std::unordered_map<std::pair<const DMC_OctreeNode *, const DMC_OctreeNode *>,
                                     uint32_t, NodeHash>;

glm::vec3 lerp (const glm::vec3 &a, float w_a, const glm::vec3 &b, float w_b) {
	return (a * w_b + b * w_a) / (w_a + w_b);
}

void vertProc (std::array<const DMC_OctreeNode *, 8> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int octreeToMcOrder[8] = { 0, 3, 1, 2, 4, 7, 5, 6 };
	constexpr int mcOrderEdges[12][2] = {
		{ 0, 2 }, { 2, 3 }, { 1, 3 }, { 0, 1 }, { 4, 6 }, { 6, 7 },
		{ 5, 7 }, { 4, 5 }, { 0, 4 }, { 2, 6 }, { 3, 7 }, { 1, 5 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3] &&
	        nodes[4] && nodes[5] && nodes[6] && nodes[7]);
	const DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DMC_OctreeNode *n = nodes[i];
		if (n->isSubdivided ()) {
			sub[i] = (*n)[7 - i];
			has_lesser = true;
		}
		else sub[i] = n;
	}
	if (has_lesser) {
		vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
		return;
	}
	// Apply MC to this dual cell
	uint32_t vertex_mask = 0;
	for (int i = 0; i < 8; i++) {
		float value = sub[i]->dualVertex.w;
		if (value <= 0.0f)
			vertex_mask |= uint32_t (1 << octreeToMcOrder[i]);
	}
	uint32_t edge_mask = edgeTable[vertex_mask];
	uint32_t edge_vertex[12];
	constexpr uint32_t bad_index = ~uint32_t (0);
	for (int i = 0; i < 12; i++) {
		edge_vertex[i] = bad_index;
		if (edge_mask & uint32_t (1 << i)) {
			int i1 = mcOrderEdges[i][0];
			int i2 = mcOrderEdges[i][1];
			auto subs = std::minmax (sub[i1], sub[i2]);
			std::pair<const DMC_OctreeNode *, const DMC_OctreeNode *> edge (subs.first, subs.second);
			auto iter = vtx_map.find (edge);
			if (iter == vtx_map.end ()) {
				float w1 = glm::abs (sub[i1]->dualVertex.w);
				float w2 = glm::abs (sub[i2]->dualVertex.w);
				glm::vec3 point = lerp (sub[i1]->dualVertex, w1, sub[i2]->dualVertex, w2);
				glm::vec3 normal = glm::normalize (lerp (sub[i1]->normal, w1, sub[i2]->normal, w2));
				Material mat = sub[i1]->material == Material::Empty ? sub[i2]->material : sub[i1]->material;
				uint32_t idx = mesh.addVertex (point, normal, mat);
				std::tie (iter, std::ignore) = vtx_map.emplace (edge, idx);
			}
			edge_vertex[i] = iter->second;
		}
	}
	for (int i = 0; triTable[vertex_mask][i] != -1; i += 3) {
		int i1 = triTable[vertex_mask][i];
		int i2 = triTable[vertex_mask][i + 1];
		int i3 = triTable[vertex_mask][i + 2];
		assert (edge_vertex[i1] != bad_index &&
		        edge_vertex[i2] != bad_index &&
		        edge_vertex[i3] != bad_index);
		mesh.addTriangle (edge_vertex[i1], edge_vertex[i2], edge_vertex[i3]);
	}
}

constexpr int edgeTableX[2][4] = { { 0, 4, 5, 1 }, { 2, 6, 7, 3 } };
constexpr int edgeTableY[2][4] = { { 0, 1, 3, 2 }, { 4, 5, 7, 6 } };
constexpr int edgeTableZ[2][4] = { { 0, 2, 6, 4 }, { 1, 3, 7, 5 } };

void edgeProcX (std::array<const DMC_OctreeNode *, 4> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 5 }, { 3, 4 }, { 0, 7 }, { 3, 6 },
		{ 1, 1 }, { 2, 0 }, { 1, 3 }, { 2, 2 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
	const DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DMC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = (*n)[subTable[i][1]];
			has_lesser = true;
		}
		else sub[i] = n;
	}
	if (!has_lesser)
		return;
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableX[i][0];
		int i2 = edgeTableX[i][1];
		int i3 = edgeTableX[i][2];
		int i4 = edgeTableX[i][3];
		edgeProcX ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
}

void edgeProcY (std::array<const DMC_OctreeNode *, 4> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 3 }, { 1, 2 }, { 3, 1 }, { 2, 0 },
		{ 0, 7 }, { 1, 6 }, { 3, 5 }, { 2, 4 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
	const DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DMC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = (*n)[subTable[i][1]];
			has_lesser = true;
		}
		else sub[i] = n;
	}
	if (!has_lesser)
		return;
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableY[i][0];
		int i2 = edgeTableY[i][1];
		int i3 = edgeTableY[i][2];
		int i4 = edgeTableY[i][3];
		edgeProcY ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
}

void edgeProcZ (std::array<const DMC_OctreeNode *, 4> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		 { 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 },
		{ 3, 2 }, { 3, 3 }, { 2, 0 }, { 2, 1 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
	const DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DMC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = (*n)[subTable[i][1]];
			has_lesser = true;
		}
		else sub[i] = n;
	}
	if (!has_lesser)
		return;
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableZ[i][0];
		int i2 = edgeTableZ[i][1];
		int i3 = edgeTableZ[i][2];
		int i4 = edgeTableZ[i][3];
		edgeProcZ ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
}

constexpr int faceTableX[4][2] = { { 0, 2 }, { 4, 6 }, { 5, 7 }, { 1, 3 } };
constexpr int faceTableY[4][2] = { { 0, 4 }, { 1, 5 }, { 3, 7 }, { 2, 6 } };
constexpr int faceTableZ[4][2] = { { 0, 1 }, { 2, 3 }, { 6, 7 }, { 4, 5 } };

void faceProcX (std::array<const DMC_OctreeNode *, 2> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 2 }, { 0, 3 }, { 1, 0 }, { 1, 1 },
		{ 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 }
	};
	assert (nodes[0] && nodes[1]);
	const DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DMC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = (*n)[subTable[i][1]];
			has_lesser = true;
		}
		else sub[i] = n;
	}
	if (!has_lesser)
		return;
	for (int i = 0; i < 4; i++) {
		int i1 = faceTableX[i][0];
		int i2 = faceTableX[i][1];
		faceProcX ({ sub[i1], sub[i2] }, vtx_map, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableY[i][0];
		int i2 = edgeTableY[i][1];
		int i3 = edgeTableY[i][2];
		int i4 = edgeTableY[i][3];
		edgeProcY ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableZ[i][0];
		int i2 = edgeTableZ[i][1];
		int i3 = edgeTableZ[i][2];
		int i4 = edgeTableZ[i][3];
		edgeProcZ ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
}


void faceProcY (std::array<const DMC_OctreeNode *, 2> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 4 }, { 0, 5 }, { 0, 6 }, { 0, 7 },
		{ 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 }
	};
	assert (nodes[0] && nodes[1]);
	const DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DMC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = (*n)[subTable[i][1]];
			has_lesser = true;
		}
		else sub[i] = n;
	}
	if (!has_lesser)
		return;
	for (int i = 0; i < 4; i++) {
		int i1 = faceTableY[i][0];
		int i2 = faceTableY[i][1];
		faceProcY ({ sub[i1], sub[i2] }, vtx_map, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableX[i][0];
		int i2 = edgeTableX[i][1];
		int i3 = edgeTableX[i][2];
		int i4 = edgeTableX[i][3];
		edgeProcX ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableZ[i][0];
		int i2 = edgeTableZ[i][1];
		int i3 = edgeTableZ[i][2];
		int i4 = edgeTableZ[i][3];
		edgeProcZ ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
}

void faceProcZ (std::array<const DMC_OctreeNode *, 2> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 1 }, { 1, 0 }, { 0, 3 }, { 1, 2 },
		{ 0, 5 }, { 1, 4 }, { 0, 7 }, { 1, 6 }
	};
	assert (nodes[0] && nodes[1]);
	const DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DMC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = (*n)[subTable[i][1]];
			has_lesser = true;
		}
		else sub[i] = n;
	}
	if (!has_lesser)
		return;
	for (int i = 0; i < 4; i++) {
		int i1 = faceTableZ[i][0];
		int i2 = faceTableZ[i][1];
		faceProcZ ({ sub[i1], sub[i2] }, vtx_map, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableX[i][0];
		int i2 = edgeTableX[i][1];
		int i3 = edgeTableX[i][2];
		int i4 = edgeTableX[i][3];
		edgeProcX ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableY[i][0];
		int i2 = edgeTableY[i][1];
		int i3 = edgeTableY[i][2];
		int i4 = edgeTableY[i][3];
		edgeProcY ({ sub[i1], sub[i2], sub[i3], sub[i4] }, vtx_map, mesh);
	}
	vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
}

void cellProc (const DMC_OctreeNode *node, VertexMap &vtx_map, Mesh &mesh) {
	assert (node);
	if (!node->isSubdivided ())
		return;
	const DMC_OctreeNode *sub[8];
	for (int i = 0; i < 8; i++) {
		sub[i] = (*node)[i];
		cellProc (sub[i], vtx_map, mesh);
	}
	for (int i = 0; i < 4; i++) {
		faceProcX ({ sub[faceTableX[i][0]], sub[faceTableX[i][1]] }, vtx_map, mesh);
		faceProcY ({ sub[faceTableY[i][0]], sub[faceTableY[i][1]] }, vtx_map, mesh);
		faceProcZ ({ sub[faceTableZ[i][0]], sub[faceTableZ[i][1]] }, vtx_map, mesh);
	}
	for (int i = 0; i < 2; i++) {
		edgeProcX ({ sub[edgeTableX[i][0]], sub[edgeTableX[i][1]],
		             sub[edgeTableX[i][2]], sub[edgeTableX[i][3]] }, vtx_map, mesh);
		edgeProcY ({ sub[edgeTableY[i][0]], sub[edgeTableY[i][1]],
		             sub[edgeTableY[i][2]], sub[edgeTableY[i][3]] }, vtx_map, mesh);
		edgeProcZ ({ sub[edgeTableZ[i][0]], sub[edgeTableZ[i][1]],
		             sub[edgeTableZ[i][2]], sub[edgeTableZ[i][3]] }, vtx_map, mesh);
	}
	vertProc ({ sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7] }, vtx_map, mesh);
}

}

using namespace dmc_detail;

Mesh DMC_Octree::contour () const {
	Mesh mesh;
	VertexMap vertex_map;
	cellProc (&m_root, vertex_map, mesh);
	return mesh;
}

void DMC_Octree::buildNode (DMC_OctreeNode *node, glm::ivec3 min_corner,
                            int32_t size, BuildArgs &args) {
	assert (node);
	if (size == 1) {
		generateDualVertex (node, min_corner, size, args);
		return;
	}
	if (args.use_early_split_stop) {
		if (shouldStopSplitting (min_corner, size, args)) {
			generateDualVertex (node, min_corner, size, args);
			return;
		}
	}
	if (args.use_simple_split_policy) {
		if (size == m_rootSize || shouldSplit (min_corner, size, args)) {
			node->subdivide ();
			int32_t child_size = size / 2;
			for (int i = 0; i < 8; i++) {
				glm::ivec3 child_min_corner = min_corner + child_size * DMC_OctreeNode::kCornerOffset[i];
				buildNode ((*node)[i], child_min_corner, child_size, args);
			}
			return;
		}
		generateDualVertex (node, min_corner, size, args);
	}
	else {
		// Perform dual vertex generation only after some unconditional subdivison
		if (size * 8 < m_rootSize)
			if (generateDualVertex (node, min_corner, size, args))
				return;
		node->subdivide ();
		int32_t child_size = size / 2;
		for (int i = 0; i < 8; i++) {
			glm::ivec3 child_min_corner = min_corner + child_size * DMC_OctreeNode::kCornerOffset[i];
			buildNode ((*node)[i], child_min_corner, child_size, args);
		}
	}
}

bool DMC_Octree::generateDualVertex (DMC_OctreeNode *node, glm::ivec3 min_corner,
                                     int32_t size, BuildArgs &args) {
	QefSolver4D &solver = args.solver;
	const ScalarField &field = args.field;
	const MaterialSelector &material = args.material;

	const glm::vec3 base_point { min_corner };
	solver.reset ();
	glm::dvec3 avg_normal { 0 };

	if (args.use_random_sampling) {
		const int points_cnt = int (6.0 * glm::sqrt (size));
		std::uniform_real_distribution<float> odist (0, size);
		for (int i = 0; i < points_cnt; i++) {
			glm::vec3 offset (odist (args.rng), odist (args.rng), odist (args.rng));
			glm::vec3 point_local = base_point + offset;
			glm::dvec3 point_global = localToGlobal (point_local);
			double value_global = field (point_global);
			float value_local = float (value_global / m_globalScale);
			glm::dvec3 grad = field.grad (point_global);
			glm::vec4 point_4d { point_local, value_local };
			glm::vec4 normal_4d { grad, -1.0f };
			solver.addPlane (point_4d, normal_4d);
			avg_normal += grad;
		}
	}
	else {
		const float min_offset = 0.25f * float (size);
		const float max_offset = float (size) - min_offset;
		const glm::vec3 sample_offset_table[8] = {
			glm::vec3 (min_offset, min_offset, min_offset),
			glm::vec3 (min_offset, min_offset, max_offset),
			glm::vec3 (max_offset, min_offset, min_offset),
			glm::vec3 (max_offset, min_offset, max_offset),
			glm::vec3 (min_offset, max_offset, min_offset),
			glm::vec3 (min_offset, max_offset, max_offset),
			glm::vec3 (max_offset, max_offset, min_offset),
			glm::vec3 (max_offset, max_offset, max_offset)
		};
		const int points_cnt = int (sizeof (sample_offset_table) / sizeof (sample_offset_table[0]));
		for (int i = 0; i < points_cnt; i++) {
			glm::vec3 point_local = base_point + sample_offset_table[i];
			glm::dvec3 point_global = localToGlobal (point_local);
			double value_global = field (point_global);
			float value_local = float (value_global / m_globalScale);
			glm::dvec3 grad = field.grad (point_global);
			glm::vec4 point_4d { point_local, value_local };
			glm::vec4 normal_4d { grad, -1.0f };
			solver.addPlane (point_4d, normal_4d);
			avg_normal += grad;
		}
	}

	node->normal = glm::vec3 (glm::normalize (avg_normal));
	glm::vec4 lower_bound (min_corner, 0);
	glm::vec4 upper_bound (min_corner + size, 0);
	glm::vec4 vertex = solver.solve (lower_bound, upper_bound);
	float error = solver.eval (vertex);
	if (error > args.epsilon) {
		lower_bound.w = std::numeric_limits<float>::lowest ();
		upper_bound.w = std::numeric_limits<float>::max ();
		vertex = solver.solve (lower_bound, upper_bound);
		error = solver.eval (vertex);
	}
	node->dualVertex = vertex;
	glm::dvec3 vertex_global = localToGlobal (vertex);
	//node->normal = glm::vec3 (glm::normalize (field.grad (vertex_global)));
	if (vertex.w <= 0) {
		double value_global = double (vertex.w) * m_globalScale;
		node->material = material.select (vertex_global, value_global);
	}
	return error <= args.epsilon;
}

bool DMC_Octree::shouldSplit (glm::ivec3 min_corner, int32_t size, BuildArgs &args) {
	const ScalarField &field = args.field;
	const double epsilon = args.epsilon;

	glm::dvec3 corner[8];
	corner[0] = localToGlobal (min_corner);
	double side = size * m_globalScale;
	for (int i = 1; i < 8; i++)
		corner[i] = corner[0] + glm::dvec3 (DMC_OctreeNode::kCornerOffset[i]) * side;

	double values[8];
	for (int i = 0; i < 8; i++)
		values[i] = field (corner[i]);

	double error = 0;
	double halfside = side * 0.5;
	// Edge midpoints
	const int dim1_corners[3][4] = {
		{ 0, 1, 4, 5 }, // X
		{ 0, 1, 2, 3 }, // Y
		{ 0, 2, 4, 6 }  // Z
	};
	const int dim1_id_offsets[3] = { 2, 4, 1 };
	const glm::dvec3 dim1_offsets[3] = {
		glm::dvec3 (halfside, 0, 0),
		glm::dvec3 (0, halfside, 0),
		glm::dvec3 (0, 0, halfside)
	};
	for (int dim = 0; dim <= 2; dim++) {
		for (int i = 0; i < 4; i++) {
			int id1 = dim1_corners[dim][i];
			int id2 = id1 + dim1_id_offsets[dim];
			glm::dvec3 p = corner[id1] + dim1_offsets[dim];
			double predict = (values[id1] + values[id2]) * 0.5;
			double actual = field (p);
			double k = glm::max (1.0, glm::length (field.grad (p)));
			error += glm::abs (predict - actual) / k;
			if (error > epsilon)
				return true;
		}
	}
	// Face midpoints
	const int dim2_corners[3][2] = {
		{ 0, 1 }, // XY
		{ 0, 4 }, // XZ
		{ 0, 2 }  // YZ
	};
	const int dim2_id_offsets[3][3] = {
		{ 2, 4, 6 }, // XY
		{ 1, 2, 3 }, // XZ
		{ 1, 4, 5 }  // YZ
	};
	const glm::dvec3 dim2_offsets[3] = {
		glm::dvec3 (halfside, halfside, 0),
		glm::dvec3 (halfside, 0, halfside),
		glm::dvec3 (0, halfside, halfside)
	};
	for (int dim = 0; dim <= 2; dim++) {
		for (int i = 0; i < 2; i++) {
			int id1 = dim2_corners[dim][i];
			int id2 = id1 + dim2_id_offsets[dim][0];
			int id3 = id1 + dim2_id_offsets[dim][1];
			int id4 = id1 + dim2_id_offsets[dim][2];
			glm::dvec3 p = corner[id1] + dim2_offsets[dim];
			double predict = (values[id1] + values[id2] + values[id3] + values[id4]) * 0.25;
			double actual = field (p);
			double k = glm::max (1.0, glm::length (field.grad (p)));
			error += glm::abs (predict - actual) / k;
			if (error > epsilon)
				return true;
		}
	}
	// Cell midpoint
	{
		glm::dvec3 p = corner[0] + halfside;
		double predict = 0;
		for (int i = 0; i < 8; i++)
			predict += values[i];
		predict *= 0.125;
		double actual = field (p);
		double k = glm::max (1.0, glm::length (field.grad (p)));
		error += glm::abs (predict - actual) / k;
		if (error > epsilon)
			return true;
	}
	return false;
}

bool DMC_Octree::shouldStopSplitting (glm::ivec3 min_corner, int32_t size, BuildArgs &args) {
	const ScalarField &field = args.field;
	const double epsilon = args.epsilon;

	const glm::dvec3 base_point = localToGlobal (min_corner);
	const double side = size * m_globalScale;
	const double diag = side * glm::root_three<double> ();

	for (int i = 0; i < 8; i++) {
		glm::dvec3 point = base_point + side * glm::dvec3 (DMC_OctreeNode::kCornerOffset[i]);
		double value = field (point);
		if (glm::abs (value) <= diag)
			return false;
	}
	return true;
}

}
