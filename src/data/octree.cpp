/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/octree.hpp>

#include <cassert>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "../algo/marching_cubes_tables.h"

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

namespace dmc_detail
{

struct NodeHash {
	size_t operator ()(const std::pair<DMC_OctreeNode *, DMC_OctreeNode *> &node) const noexcept {
		uintptr_t h1 = uintptr_t (node.first);
		uintptr_t h2 = uintptr_t (node.second);
		return h1 ^ (h2 << 1);
	}
};

using VertexMap = std::unordered_map<std::pair<DMC_OctreeNode *, DMC_OctreeNode *>, uint32_t, NodeHash>;

glm::vec3 lerp (const glm::vec4 &a, const glm::vec4 &b) {
	float w_a = glm::abs (a.w);
	float w_b = glm::abs (b.w);
	glm::vec3 p_a (a.x, a.y, a.z);
	glm::vec3 p_b (b.x, b.y, b.z);
	return (p_a * w_b + p_b * w_a) / (w_a + w_b);
}

void vertProc (std::array<DMC_OctreeNode *, 8> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int octreeToMcOrder[8] = { 0, 3, 1, 2, 4, 7, 5, 6 };
	constexpr int mcOrderEdges[12][2] = {
		{ 0, 2 }, { 2, 3 }, { 1, 3 }, { 0, 1 }, { 4, 6 }, { 6, 7 },
		{ 5, 7 }, { 4, 5 }, { 0, 4 }, { 2, 6 }, { 3, 7 }, { 1, 5 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3] &&
	        nodes[4] && nodes[5] && nodes[6] && nodes[7]);
	DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		DMC_OctreeNode *n = nodes[i];
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
			std::pair<DMC_OctreeNode *, DMC_OctreeNode *> edge (subs.first, subs.second);
			auto iter = vtx_map.find (edge);
			if (iter == vtx_map.end ()) {
				glm::vec3 point = lerp (sub[i1]->dualVertex, sub[i2]->dualVertex);
				glm::vec3 normal = glm::normalize (sub[i1]->normal + sub[i2]->normal);
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

void edgeProcX (std::array<DMC_OctreeNode *, 4> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 5 }, { 3, 4 }, { 0, 7 }, { 3, 6 },
		{ 1, 1 }, { 2, 0 }, { 1, 3 }, { 2, 2 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
	DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		DMC_OctreeNode *n = nodes[subTable[i][0]];
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

void edgeProcY (std::array<DMC_OctreeNode *, 4> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 3 }, { 1, 2 }, { 3, 1 }, { 2, 0 },
		{ 0, 7 }, { 1, 6 }, { 3, 5 }, { 2, 4 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
	DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		DMC_OctreeNode *n = nodes[subTable[i][0]];
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

void edgeProcZ (std::array<DMC_OctreeNode *, 4> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		 { 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 },
		{ 3, 2 }, { 3, 3 }, { 2, 0 }, { 2, 1 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
	DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		DMC_OctreeNode *n = nodes[subTable[i][0]];
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

void faceProcX (std::array<DMC_OctreeNode *, 2> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 2 }, { 0, 3 }, { 1, 0 }, { 1, 1 },
		{ 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 }
	};
	assert (nodes[0] && nodes[1]);
	DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		DMC_OctreeNode *n = nodes[subTable[i][0]];
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


void faceProcY (std::array<DMC_OctreeNode *, 2> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 4 }, { 0, 5 }, { 0, 6 }, { 0, 7 },
		{ 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 }
	};
	assert (nodes[0] && nodes[1]);
	DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		DMC_OctreeNode *n = nodes[subTable[i][0]];
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

void faceProcZ (std::array<DMC_OctreeNode *, 2> nodes, VertexMap &vtx_map, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 1 }, { 1, 0 }, { 0, 3 }, { 1, 2 },
		{ 0, 5 }, { 1, 4 }, { 0, 7 }, { 1, 6 }
	};
	assert (nodes[0] && nodes[1]);
	DMC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		DMC_OctreeNode *n = nodes[subTable[i][0]];
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

void cellProc (DMC_OctreeNode *node, VertexMap &vtx_map, Mesh &mesh) {
	assert (node);
	if (!node->isSubdivided ())
		return;
	DMC_OctreeNode *sub[8];
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
	if (m_root) {
		VertexMap vertex_map;
		cellProc (m_root, vertex_map, mesh);
	}
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
	// TODO: this is a oversimplified debug version
	// TODO: fix commented code below and remove this
	{
		if (node->size > 1) {
			node->subdivide ();
			for (int i = 0; i < 8; i++)
				buildNode ((*node)[i], args);
			return;
		}
		glm::vec3 point_local = glm::vec3 (node->minCorner) + 0.5f * float (node->size);
		glm::dvec3 point_global = localToGlobal (point_local);
		float value = float (args.field (point_global));
		node->dualVertex = glm::vec4 (point_local, value);
		node->normal = glm::vec3 (args.field.grad (point_global));
		node->material = args.material.select (point_global, value);
		return;
	}
	/*
	// Sample field at some points
	float offset = float (node->size) * 0.8f;
	glm::vec3 base = glm::vec3 (node->minCorner) + 0.5f * (1.0f - offset);
	const glm::vec3 offset_table[8] = {
		glm::vec3 (0, 0, 0) * offset,
		glm::vec3 (0, 0, 1) * offset,
		glm::vec3 (1, 0, 0) * offset,
		glm::vec3 (1, 0, 1) * offset,
		glm::vec3 (0, 1, 0) * offset,
		glm::vec3 (0, 1, 1) * offset,
		glm::vec3 (1, 1, 0) * offset,
		glm::vec3 (1, 1, 1) * offset
	};
	QefSolver4D &solver = args.solver;
	const ScalarField &field = args.field;
	solver.reset ();
	glm::vec3 avg_normal (0);
	for (int i = 0; i < 8; i++) {
		glm::vec3 local_pos = base + offset_table[i];
		glm::dvec3 global_pos = localToGlobal (local_pos);
		float value = float (field (global_pos));
		glm::vec3 grad = glm::vec3 (field.grad (global_pos));
		avg_normal += grad;
		glm::vec4 point (local_pos, value);
		glm::vec4 normal (grad, -1.0f);
		normal = glm::normalize (normal);
		solver.addPlane (point, normal);
	}
	constexpr float min_value = std::numeric_limits<float>::lowest ();
	constexpr float max_value = std::numeric_limits<float>::max ();
	glm::vec4 lower_bound (node->minCorner, min_value);
	glm::vec4 upper_bound (node->minCorner + node->size, max_value);
	node->dualVertex = solver.solve (lower_bound, upper_bound);
	if (glm::abs (node->dualVertex.w) < args.epsilon)
		node->dualVertex.w = 0;
	node->normal = glm::normalize (avg_normal);
	float error = solver.eval (node->dualVertex);
	float estim_value = node->dualVertex.w;
	float real_value;
	{
		glm::dvec3 global_dual_vtx = localToGlobal (node->dualVertex);
		double value = node->dualVertex.w;
		double realValue = field (global_dual_vtx);
		node->material = args.material.select (global_dual_vtx, value);
		node->dualVertex.w = float (realValue);
		real_value = float (realValue);
	}
	float error2 = solver.eval (node->dualVertex);
	if (node->size > 1 && error > args.epsilon) {
		node->subdivide ();
		for (int i = 0; i < 8; i++)
			buildNode ((*node)[i], args);
	}
	*/
}

}
