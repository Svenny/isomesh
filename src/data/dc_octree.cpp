/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/dc_octree.hpp>
#include <isomesh/util/material_filter.hpp>
#include <isomesh/util/tables.hpp>

#include <cassert>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace isomesh
{

DC_Octree::DC_Octree (int32_t root_size, glm::dvec3 global_pos, double global_scale) :
	m_globalPos (global_pos), m_globalScale (global_scale), m_rootSize (root_size) {
	if (root_size <= 0 || (root_size & (root_size - 1)))
		throw std::invalid_argument ("Octree size is not a power of two");
}

void DC_Octree::build (const UniformGrid &grid, QefSolver3D &solver, float epsilon,
                       bool use_octree_simplification) {
	// Scale epsilon according to QEF scale (when translating from global coordinates to
	// local QEF value is scaled by 1/(scale^2))
	float scaled_epsilon = epsilon / float (m_globalScale * m_globalScale);
	BuildArgs args {
		grid, solver,
		scaled_epsilon,
		use_octree_simplification
	};
	try {
		if (m_root.isSubdivided ())
			m_root.collapse ();
		glm::ivec3 min_corner (-m_rootSize / 2);
		buildNode (&m_root, min_corner, m_rootSize, args);
	}
	catch (...) {
		if (m_root.isSubdivided())
			m_root.collapse ();
		throw;
	}
}

namespace dc_detail
{

/* Quadruples of node children sharing an edge along some axis. First dimension - axis,
 second dimension - quadruple, third dimension - children. */
constexpr int edgeTable[3][2][4] = {
	{ { 0, 4, 5, 1 }, { 2, 6, 7, 3 } }, // X
	{ { 0, 1, 3, 2 }, { 4, 5, 7, 6 } }, // Y
	{ { 0, 2, 6, 4 }, { 1, 3, 7, 5 } }  // Z
};

template<int D>
void edgeProc (std::array<const DC_OctreeNode *, 4> nodes, Mesh &mesh) {
	/* For a quadruple of nodes sharing an edge along some axis there are two quadruples
	 of their children nodes sharing the same edge. This table maps node to its child. First
	 dimension - axis, second dimension - position of child. Two values - parent number (in
	 edgeProc's numbering) and its child number (in octree numbering). Is it better understandable
	 by looking at the code? */
	constexpr int subTable[3][8][2] = {
		{ { 0, 5 }, { 3, 4 }, { 0, 7 }, { 3, 6 },
			{ 1, 1 }, { 2, 0 }, { 1, 3 }, { 2, 2 } }, // X
		{ { 0, 3 }, { 1, 2 }, { 3, 1 }, { 2, 0 },
			{ 0, 7 }, { 1, 6 }, { 3, 5 }, { 2, 4 } }, // Y
		{ { 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 },
			{ 3, 2 }, { 3, 3 }, { 2, 0 }, { 2, 1 } }  // Z
	};
	// Almost the same as above
	constexpr int cornersTable[3][4][2] = {
		{ { 5, 7 }, { 1, 3 }, { 0, 2 }, { 4, 6 } }, // X
		{ { 3, 7 }, { 2, 6 }, { 0, 4 }, { 1, 5 } }, // Y
		{ { 6, 7 }, { 4, 5 }, { 0, 1 }, { 2, 3 } }  // Z
	};
	/* If at least one node is not present then (by topological safety test guarantee) this edge
	 is not surface-crossing (otherwise there was an unsafe node collapse). */
	if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3])
		return;
	const DC_OctreeNode *sub[8];
	bool all_leaves = true;
	for (int i = 0; i < 8; i++) {
		const DC_OctreeNode *n = nodes[subTable[D][i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->children[subTable[D][i][1]];
			all_leaves = false;
		}
		else sub[i] = n;
	}
	if (!all_leaves) {
		for (int i = 0; i < 2; i++) {
			int i1 = edgeTable[D][i][0];
			int i2 = edgeTable[D][i][1];
			int i3 = edgeTable[D][i][2];
			int i4 = edgeTable[D][i][3];
			edgeProc<D> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
		}
		return;
	}
	Material mat1 = Material::Empty, mat2 = Material::Empty;
	/* Find the minimal node, i.e. the node with the maximal depth. By looking at its
	 materials on endpoints of this edge we may know whether the edge is surface-crossing
	 and if we need to flip the triangles winding order. */
	int16_t max_depth = -1;
	for (int i = 0; i < 4; i++) {
		if (nodes[i]->depth () > max_depth) {
			max_depth = nodes[i]->depth ();
			mat1 = nodes[i]->leaf_data.corners[cornersTable[D][i][0]];
			mat2 = nodes[i]->leaf_data.corners[cornersTable[D][i][1]];
		}
	}
	if (mat1 == mat2)
		return; // Not a surface-crossing edge
	if (mat1 != Material::Empty && mat2 != Material::Empty)
		return; // Ditto
	/* We assume that lower endpoint is solid. If this is not the case, the triangles'
	 winding order should be flipped to remain facing outside of the surface. */
	bool flip = (mat1 == Material::Empty);
	uint32_t id0 = nodes[0]->leaf_data.vertex_id;
	uint32_t id1 = nodes[1]->leaf_data.vertex_id;
	uint32_t id2 = nodes[2]->leaf_data.vertex_id;
	uint32_t id3 = nodes[3]->leaf_data.vertex_id;
	if (!flip) {
		mesh.addTriangle (id0, id1, id2);
		mesh.addTriangle (id0, id2, id3);
	}
	else {
		mesh.addTriangle (id0, id2, id1);
		mesh.addTriangle (id0, id3, id2);
	}
}

constexpr int faceTableX[4][2] = { { 0, 2 }, { 4, 6 }, { 5, 7 }, { 1, 3 } };
constexpr int faceTableY[4][2] = { { 0, 4 }, { 1, 5 }, { 3, 7 }, { 2, 6 } };
constexpr int faceTableZ[4][2] = { { 0, 1 }, { 2, 3 }, { 6, 7 }, { 4, 5 } };

void faceProcX (std::array<const DC_OctreeNode *, 2> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 2 }, { 0, 3 }, { 1, 0 }, { 1, 1 },
		{ 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 }
	};
	assert (nodes[0] && nodes[1]);
	const DC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DC_OctreeNode *n = nodes[subTable[i][0]];
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
		faceProcX ({ sub[i1], sub[i2] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTable[1][i][0];
		int i2 = edgeTable[1][i][1];
		int i3 = edgeTable[1][i][2];
		int i4 = edgeTable[1][i][3];
		edgeProc<1> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTable[2][i][0];
		int i2 = edgeTable[2][i][1];
		int i3 = edgeTable[2][i][2];
		int i4 = edgeTable[2][i][3];
		edgeProc<2> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
}

void faceProcY (std::array<const DC_OctreeNode *, 2> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 4 }, { 0, 5 }, { 0, 6 }, { 0, 7 },
		{ 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 }
	};
	assert (nodes[0] && nodes[1]);
	const DC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DC_OctreeNode *n = nodes[subTable[i][0]];
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
		faceProcY ({ sub[i1], sub[i2] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTable[0][i][0];
		int i2 = edgeTable[0][i][1];
		int i3 = edgeTable[0][i][2];
		int i4 = edgeTable[0][i][3];
		edgeProc<0> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTable[2][i][0];
		int i2 = edgeTable[2][i][1];
		int i3 = edgeTable[2][i][2];
		int i4 = edgeTable[2][i][3];
		edgeProc<2> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
}

void faceProcZ (std::array<const DC_OctreeNode *, 2> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 1 }, { 1, 0 }, { 0, 3 }, { 1, 2 },
		{ 0, 5 }, { 1, 4 }, { 0, 7 }, { 1, 6 }
	};
	assert (nodes[0] && nodes[1]);
	const DC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const DC_OctreeNode *n = nodes[subTable[i][0]];
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
		faceProcZ ({ sub[i1], sub[i2] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTable[0][i][0];
		int i2 = edgeTable[0][i][1];
		int i3 = edgeTable[0][i][2];
		int i4 = edgeTable[0][i][3];
		edgeProc<0> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTable[1][i][0];
		int i2 = edgeTable[1][i][1];
		int i3 = edgeTable[1][i][2];
		int i4 = edgeTable[1][i][3];
		edgeProc<1> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
}

void cellProc (const DC_OctreeNode *node, Mesh &mesh) {
	assert (node);
	if (!node->isSubdivided ())
		return;
	const DC_OctreeNode *sub[8];
	for (int i = 0; i < 8; i++) {
		sub[i] = (*node)[i];
		cellProc (sub[i], mesh);
	}
	for (int i = 0; i < 4; i++) {
		faceProcX ({ sub[faceTableX[i][0]], sub[faceTableX[i][1]] }, mesh);
		faceProcY ({ sub[faceTableY[i][0]], sub[faceTableY[i][1]] }, mesh);
		faceProcZ ({ sub[faceTableZ[i][0]], sub[faceTableZ[i][1]] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		edgeProc<0> ({ sub[edgeTable[0][i][0]], sub[edgeTable[0][i][1]],
		               sub[edgeTable[0][i][2]], sub[edgeTable[0][i][3]] }, mesh);
		edgeProc<1> ({ sub[edgeTable[1][i][0]], sub[edgeTable[1][i][1]],
		               sub[edgeTable[1][i][2]], sub[edgeTable[1][i][3]] }, mesh);
		edgeProc<2> ({ sub[edgeTable[2][i][0]], sub[edgeTable[2][i][1]],
		               sub[edgeTable[2][i][2]], sub[edgeTable[2][i][3]] }, mesh);
	}
}

void makeVertices (DC_OctreeNode *node, Mesh &mesh) {
	MaterialFilter filter;
	if (!node->isSubdivided ()) {
		if (!node->isHomogenous ()) {
			glm::vec3 vertex = node->leaf_data.dual_vertex;
			glm::vec3 normal = node->leaf_data.normal;
			filter.reset ();
			filter.add (node->leaf_data.corners);
			Material mat = filter.select ();
			node->leaf_data.vertex_id = mesh.addVertex (vertex, normal, mat);
		}
		else node->leaf_data.vertex_id = std::numeric_limits<uint32_t>::max ();
	}
	else for (int i = 0; i < 8; i++)
		makeVertices (node->children[i], mesh);
}

}

using namespace dc_detail;

Mesh DC_Octree::contour () {
	Mesh mesh;
	makeVertices (&m_root, mesh);
	cellProc (&m_root, mesh);
	mesh.setGlobalPos (m_globalPos);
	mesh.setGlobalScale (m_globalScale);
	return mesh;
}

void DC_Octree::buildNode (DC_OctreeNode *node, glm::ivec3 min_corner,
                           int32_t size, BuildArgs &args) {
	assert (node);
	if (size == 1) {
		buildLeaf (node, min_corner, size, args);
		return;
	}
	node->subdivide ();
	int32_t child_size = size / 2;
	for (int i = 0; i < 8; i++) {
		glm::ivec3 child_min_corner = min_corner + child_size * kCellCornerOffset[i];
		buildNode(node->children[i], child_min_corner, child_size, args);
	}
	// All children are build and possibly simplified, try to do simplification of this node
	std::array<Material, 8> corners;
	corners.fill (Material::Empty);
	bool all_children_homogenous = true;
	for (int i = 0; i < 8; i++) {
		// We can't do any simplification if there is a least one non-leaf child
		if (node->children[i]->isSubdivided ())
			return;
		if (!node->children[i]->isHomogenous ())
			all_children_homogenous = false;
		corners[i] = node->children[i]->leaf_data.corners[i];
	}
	// If all children are homogenous, simply drop them (this octree is not to be used as a storage)
	if (all_children_homogenous) {
		node->collapse ();
		node->leaf_data.corners = corners;
		return;
	}
	if (args.use_octree_simplification) {
		CubeMaterials mats;
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {
				auto offset = kCellCornerOffset[i] + kCellCornerOffset[j];
				mats[offset.y][offset.x][offset.z] = node->children[i]->leaf_data.corners[j];
			}
		}
		if (!checkTopoSafety (mats))
			return;
		args.solver.reset ();
		glm::vec3 avg_normal { 0 };
		for (int i = 0; i < 8; i++) {
			if (!node->children[i]->isHomogenous ()) {
				avg_normal += node->children[i]->leaf_data.normal;
				args.solver.merge (node->children[i]->leaf_data.qef);
			}
		}
		glm::vec3 lower_bound (min_corner);
		glm::vec3 upper_bound (min_corner + size);
		glm::vec3 vertex = args.solver.solve (lower_bound, upper_bound);
		float error = args.solver.eval (vertex);
		if (error > args.epsilon)
			return;
		node->collapse ();
		node->leaf_data.dual_vertex = vertex;
		node->leaf_data.normal = glm::normalize (avg_normal);
		node->leaf_data.corners = corners;
		node->leaf_data.qef = args.solver.state ();
	}
}

void DC_Octree::buildLeaf (DC_OctreeNode *node, glm::ivec3 min_corner,
                           int32_t size, BuildArgs &args) {
	QefSolver3D &solver = args.solver;
	const UniformGrid &G = args.grid;

	solver.reset ();
	glm::vec3 avg_normal { 0 };

	node->leaf_data.corners = G.materialsOfCell (G.pointToIndex (min_corner));
	
	bool has_edges = false;

	constexpr int edge_table[3][4][2] = {
		{ { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 } }, // X
		{ { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } }, // Y
		{ { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 } }  // Z
	};
	for (int dim = 0; dim <= 2; dim++) {
		for (int i = 0; i < 4; i++) {
			Material mat1 = node->leaf_data.corners[edge_table[dim][i][0]];
			Material mat2 = node->leaf_data.corners[edge_table[dim][i][1]];
			if (mat1 == mat2)
				continue;
			if (mat1 != Material::Empty && mat2 != Material::Empty)
				continue;
			has_edges = true;
			// C++ really needs 'for static' like in D language...
			const auto &storage = (dim == 0 ? G.edges<0> () : dim == 1 ? G.edges<1> () : G.edges<2> ());
			auto edge_pos = min_corner + kCellCornerOffset[edge_table[dim][i][0]];
			auto iter = storage.findEdge (edge_pos.x, edge_pos.y, edge_pos.z);
			assert (iter != storage.end ());
			glm::vec3 vertex = iter->surfacePoint ();
			glm::vec3 normal = iter->surfaceNormal ();
			solver.addPlane (vertex, normal);
			avg_normal += normal;
		}
	}

	if (has_edges) {
		node->leaf_data.normal = glm::normalize (avg_normal);
		glm::vec3 lower_bound (min_corner);
		glm::vec3 upper_bound (min_corner + size);
		node->leaf_data.dual_vertex = solver.solve (lower_bound, upper_bound);
		node->leaf_data.qef = solver.state ();
	}
}

bool DC_Octree::checkTopoSafety (const CubeMaterials &mats) noexcept {
	constexpr int dcToMc[8] = { 0, 3, 1, 2, 4, 7, 5, 6 };
	constexpr bool isManifold[256] = {
		true, true, true, true, true, false, true, true, true, true, false, true, true, true, true, true,
		true, true, false, true, false, false, false, true, false, true, false, true, false, true, false, true,
		true, false, true, true, false, false, true, true, false, false, false, true, false, false, true, true,
		true, true, true, true, false, false, true, true, false, true, false, true, false, false, false, true,
		true, false, false, false, true, false, true, true, false, false, false, false, true, true, true, true,
		false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
		true, false, true, true, true, false, true, true, false, false, false, false, true, false, true, true,
		true, true, true, true, true, false, true, true, false, false, false, false, false, false, false, true,
		true, false, false, false, false, false, false, false, true, true, false, true, true, true, true, true,
		true, true, false, true, false, false, false, false, true, true, false, true, true, true, false, true,
		false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
		true, true, true, true, false, false, false, false, true, true, false, true, false, false, false, true,
		true, false, false, false, true, false, true, false, true, true, false, false, true, true, true, true,
		true, true, false, false, true, false, false, false, true, true, false, false, true, true, false, true,
		true, false, true, false, true, false, true, false, true, false, false, false, true, false, true, true,
		true, true, true, true, true, false, true, true, true, true, false, true, true, true, true, true,
	};
	uint32_t mask = 0;
	for (int i = 0; i < 8; i++) {
		auto pos = 2 * kCellCornerOffset[i];
		auto mat = mats[pos.y][pos.x][pos.z];
		if (mat != Material::Empty)
			mask |= uint32_t (1 << dcToMc[i]);
	}
	if (!isManifold[mask])
		return false;
	for (int i = 0; i < 8; i++) {
		uint32_t submask = 0;
		for (int j = 0; j < 8; j++) {
			auto pos = kCellCornerOffset[i] + kCellCornerOffset[j];
			auto mat = mats[pos.y][pos.x][pos.z];
			if (mat != Material::Empty)
				submask |= uint32_t (1 << dcToMc[j]);
		}
		if (!isManifold[submask])
			return false;
	}
	// Check edge midpoint signs
	for (int c1 = 0; c1 < 3; c1++) {
		for (int c2 = 0; c2 < 3; c2++) {
			if (mats[1][c1][c2] != mats[0][c1][c2] && mats[1][c1][c2] != mats[2][c1][c2])
				return false;
			if (mats[c1][1][c2] != mats[c1][0][c2] && mats[c1][1][c2] != mats[c1][2][c2])
				return false;
			if (mats[c1][c2][1] != mats[c1][c2][0] && mats[c1][c2][1] != mats[c1][c2][2])
				return false;
		}
	}
	// Check face midpoint signs
	for (int c1 = 0; c1 < 3; c1++) {
		Material mat;
		mat = mats[1][1][c1];
		if (mat != mats[0][0][c1] && mat != mats[0][2][c1] && mat != mats[2][0][c1] && mat != mats[2][2][c1])
			return false;
		mat = mats[1][c1][1];
		if (mat != mats[0][c1][0] && mat != mats[0][c1][2] && mat != mats[2][c1][0] && mat != mats[2][c1][2])
			return false;
		mat = mats[c1][1][1];
		if (mat != mats[c1][0][0] && mat != mats[c1][0][2] && mat != mats[c1][2][0] && mat != mats[c1][2][2])
			return false;
	}
	// Check cube midpoint sign
	auto mat = mats[1][1][1];
	for (int i = 0; i < 8; i++) {
		auto pos = 2 * kCellCornerOffset[i];
		if (mat == mats[pos.y][pos.x][pos.z])
			return true;
	}
	return false;
}

}
