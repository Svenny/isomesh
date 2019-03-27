/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/dc_octree.hpp>

#include <cassert>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <glm/gtc/constants.hpp>

namespace isomesh
{

DC_Octree::DC_Octree (int32_t root_size, glm::dvec3 global_pos, double global_scale) :
	m_globalPos (global_pos), m_globalScale (global_scale), m_rootSize (root_size) {
	if (root_size <= 0 || (root_size & (root_size - 1)))
		throw std::invalid_argument ("Octree size is not a power of two");
}

void DC_Octree::build (const UniformGrid &grid, QrQefSolver3D &solver, float epsilon,
                       bool use_octree_simplification) {
	// Scale epsilon according to QEF scale (when translating from global coordinates to
	// local QEF value is scaled by 1/(scale^2))
	float scaled_epsilon = epsilon / float (m_globalScale * m_globalScale);
	BuildArgs args {
		grid, solver,
		scaled_epsilon,
		use_octree_simplification
	};
	//try {
		if (m_root.isSubdivided ())
			m_root.collapse ();
		glm::ivec3 min_corner (-m_rootSize / 2);
		buildNode (&m_root, min_corner, m_rootSize, args);
	/*}
	catch (...) {
		auto e = std::current_exception ();
		m_root.collapse ();
		std::rethrow_exception (e);
	}*/
}

namespace dc_detail
{

constexpr int edgeTableX[2][4] = { { 0, 4, 5, 1 }, { 2, 6, 7, 3 } };
constexpr int edgeTableY[2][4] = { { 0, 1, 3, 2 }, { 4, 5, 7, 6 } };
constexpr int edgeTableZ[2][4] = { { 0, 2, 6, 4 }, { 1, 3, 7, 5 } };

void edgeProcX (std::array<const DC_OctreeNode *, 4> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 5 }, { 3, 4 }, { 0, 7 }, { 3, 6 },
		{ 1, 1 }, { 2, 0 }, { 1, 3 }, { 2, 2 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
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
	if (!has_lesser) {
		Material mat1, mat2;
		{
			int16_t max_depth = -1;
			int max_idx = -1;
			for (int i = 0; i < 4; i++) {
				if (nodes[i]->depth () > max_depth) {
					max_depth = nodes[i]->depth ();
					max_idx = i;
				}
			}
			bool mat1_found = false;
			for (int i = 0; i < 8; i++) {
				if (subTable[i][0] == max_idx) {
					if (mat1_found)
						mat2 = nodes[max_idx]->corners[subTable[i][1]];
					else {
						mat1 = nodes[max_idx]->corners[subTable[i][1]];
						mat1_found = true;
					}
				}
			}
		}
		if (mat1 == mat2)
			return;
		if (mat1 != Material::Empty && mat2 != Material::Empty)
			return;
		bool flip = (mat1 == Material::Empty);
		uint32_t id0 = nodes[0]->vertex_id;
		uint32_t id1 = nodes[1]->vertex_id;
		uint32_t id2 = nodes[2]->vertex_id;
		uint32_t id3 = nodes[3]->vertex_id;
		if (!flip) {
			mesh.addTriangle (id0, id1, id2);
			mesh.addTriangle (id0, id2, id3);
		}
		else {
			mesh.addTriangle (id0, id2, id1);
			mesh.addTriangle (id0, id3, id2);
		}
		return;
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableX[i][0];
		int i2 = edgeTableX[i][1];
		int i3 = edgeTableX[i][2];
		int i4 = edgeTableX[i][3];
		edgeProcX ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
}

void edgeProcY (std::array<const DC_OctreeNode *, 4> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 3 }, { 1, 2 }, { 3, 1 }, { 2, 0 },
		{ 0, 7 }, { 1, 6 }, { 3, 5 }, { 2, 4 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
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
	if (!has_lesser) {
		Material mat1, mat2;
		{
			int16_t max_depth = -1;
			int max_idx = -1;
			for (int i = 0; i < 4; i++) {
				if (nodes[i]->depth () > max_depth) {
					max_depth = nodes[i]->depth ();
					max_idx = i;
				}
			}
			bool mat1_found = false;
			for (int i = 0; i < 8; i++) {
				if (subTable[i][0] == max_idx) {
					if (mat1_found)
						mat2 = nodes[max_idx]->corners[subTable[i][1]];
					else {
						mat1 = nodes[max_idx]->corners[subTable[i][1]];
						mat1_found = true;
					}
				}
			}
		}
		if (mat1 == mat2)
			return;
		if (mat1 != Material::Empty && mat2 != Material::Empty)
			return;
		bool flip = (mat1 == Material::Empty);
		uint32_t id0 = nodes[0]->vertex_id;
		uint32_t id1 = nodes[1]->vertex_id;
		uint32_t id2 = nodes[2]->vertex_id;
		uint32_t id3 = nodes[3]->vertex_id;
		if (!flip) {
			mesh.addTriangle (id0, id1, id2);
			mesh.addTriangle (id0, id2, id3);
		}
		else {
			mesh.addTriangle (id0, id2, id1);
			mesh.addTriangle (id0, id3, id2);
		}
		return;
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableY[i][0];
		int i2 = edgeTableY[i][1];
		int i3 = edgeTableY[i][2];
		int i4 = edgeTableY[i][3];
		edgeProcY ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
}

void edgeProcZ (std::array<const DC_OctreeNode *, 4> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 },
		{ 3, 2 }, { 3, 3 }, { 2, 0 }, { 2, 1 }
	};
	assert (nodes[0] && nodes[1] && nodes[2] && nodes[3]);
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
	if (!has_lesser) {
		Material mat1, mat2;
		{
			int16_t max_depth = -1;
			int max_idx = -1;
			for (int i = 0; i < 4; i++) {
				if (nodes[i]->depth () > max_depth) {
					max_depth = nodes[i]->depth ();
					max_idx = i;
				}
			}
			bool mat1_found = false;
			for (int i = 0; i < 8; i++) {
				if (subTable[i][0] == max_idx) {
					if (mat1_found)
						mat2 = nodes[max_idx]->corners[subTable[i][1]];
					else {
						mat1 = nodes[max_idx]->corners[subTable[i][1]];
						mat1_found = true;
					}
				}
			}
		}
		if (mat1 == mat2)
			return;
		if (mat1 != Material::Empty && mat2 != Material::Empty)
			return;
		bool flip = (mat1 == Material::Empty);
		uint32_t id0 = nodes[0]->vertex_id;
		uint32_t id1 = nodes[1]->vertex_id;
		uint32_t id2 = nodes[2]->vertex_id;
		uint32_t id3 = nodes[3]->vertex_id;
		if (!flip) {
			mesh.addTriangle (id0, id1, id2);
			mesh.addTriangle (id0, id2, id3);
		}
		else {
			mesh.addTriangle (id0, id2, id1);
			mesh.addTriangle (id0, id3, id2);
		}
		return;
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableZ[i][0];
		int i2 = edgeTableZ[i][1];
		int i3 = edgeTableZ[i][2];
		int i4 = edgeTableZ[i][3];
		edgeProcZ ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
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
		int i1 = edgeTableY[i][0];
		int i2 = edgeTableY[i][1];
		int i3 = edgeTableY[i][2];
		int i4 = edgeTableY[i][3];
		edgeProcY ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableZ[i][0];
		int i2 = edgeTableZ[i][1];
		int i3 = edgeTableZ[i][2];
		int i4 = edgeTableZ[i][3];
		edgeProcZ ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
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
		int i1 = edgeTableX[i][0];
		int i2 = edgeTableX[i][1];
		int i3 = edgeTableX[i][2];
		int i4 = edgeTableX[i][3];
		edgeProcX ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableZ[i][0];
		int i2 = edgeTableZ[i][1];
		int i3 = edgeTableZ[i][2];
		int i4 = edgeTableZ[i][3];
		edgeProcZ ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
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
		int i1 = edgeTableX[i][0];
		int i2 = edgeTableX[i][1];
		int i3 = edgeTableX[i][2];
		int i4 = edgeTableX[i][3];
		edgeProcX ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
	}
	for (int i = 0; i < 2; i++) {
		int i1 = edgeTableY[i][0];
		int i2 = edgeTableY[i][1];
		int i3 = edgeTableY[i][2];
		int i4 = edgeTableY[i][3];
		edgeProcY ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh);
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
		edgeProcX ({ sub[edgeTableX[i][0]], sub[edgeTableX[i][1]],
						 sub[edgeTableX[i][2]], sub[edgeTableX[i][3]] }, mesh);
		edgeProcY ({ sub[edgeTableY[i][0]], sub[edgeTableY[i][1]],
						 sub[edgeTableY[i][2]], sub[edgeTableY[i][3]] }, mesh);
		edgeProcZ ({ sub[edgeTableZ[i][0]], sub[edgeTableZ[i][1]],
						 sub[edgeTableZ[i][2]], sub[edgeTableZ[i][3]] }, mesh);
	}
}

void makeVertices (DC_OctreeNode *node, const MaterialFilter &filter, Mesh &mesh) {
	if (!node->isSubdivided ()) {
		if (!node->isHomogenous ()) {
			glm::vec3 vertex = node->dualVertex;
			glm::vec3 normal = node->normal;
			Material mat = filter.select (node->corners, 0xFF);
			node->vertex_id = mesh.addVertex (vertex, normal, mat);
		}
		else node->vertex_id = std::numeric_limits<uint32_t>::max ();
		return;
	}
	for (int i = 0; i < 8; i++)
		makeVertices (node->children[i], filter, mesh);
}

}

using namespace dc_detail;

Mesh DC_Octree::contour (const MaterialFilter &filter) {
	Mesh mesh;
	makeVertices (&m_root, filter, mesh);
	cellProc (&m_root, mesh);
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
		glm::ivec3 child_min_corner = min_corner + child_size * DC_OctreeNode::kCornerOffset[i];
		buildNode ((*node)[i], child_min_corner, child_size, args);
	}
	// Homogenous collapse
	{
		bool all_homo = true;
		std::array<Material, 8> corners;
		for (int i = 0; i < 8; i++) {
			if (!node->children[i]->isHomogenous ()) {
				all_homo = false;
				break;
			}
			corners[i] = node->children[i]->corners[i];
		}
		if (all_homo) {
			node->collapse ();
			node->corners = corners;
			return;
		}
	}
	if (args.use_octree_simplification) {
		bool all_leaves = true;
		CubeMaterials mats;
		std::array<Material, 8> corners;
		for (int i = 0; i < 8; i++) {
			if (node->children[i]->isSubdivided ()) {
				all_leaves = false;
				break;
			}
			corners[i] = node->children[i]->corners[i];
			for (int j = 0; j < 8; j++) {
				auto offset = DC_OctreeNode::kCornerOffset[i] + DC_OctreeNode::kCornerOffset[j];
				mats[offset.y][offset.x][offset.z] = node->children[i]->corners[j];
			}
		}
		if (!all_leaves)
			return;
		if (!checkTopoSafety (mats))
			return;
		args.solver.reset ();
		glm::vec3 avg_normal { 0 };
		for (int i = 0; i < 8; i++) {
			if (!node->children[i]->isHomogenous ()) {
				avg_normal += node->children[i]->normal;
				args.solver.merge (node->children[i]->qef);
			}
		}
		glm::vec3 lower_bound (min_corner);
		glm::vec3 upper_bound (min_corner + size);
		glm::vec3 vertex = args.solver.solve (lower_bound, upper_bound);
		float error = args.solver.eval (vertex);
		if (error > args.epsilon)
			return;
		node->collapse ();
		node->dualVertex = vertex;
		node->normal = glm::normalize (avg_normal);
		node->corners = corners;
		node->qef = args.solver.data ();
	}
}

void DC_Octree::buildLeaf (DC_OctreeNode *node, glm::ivec3 min_corner,
                           int32_t size, BuildArgs &args) {
	QrQefSolver3D &solver = args.solver;
	const UniformGrid &G = args.grid;

	solver.reset ();
	glm::vec3 avg_normal { 0 };

	for (int i = 0; i < 8; i++)
		node->corners[i] = G[min_corner + DC_OctreeNode::kCornerOffset[i]];
	
	bool has_edges = false;

	const int edge_table[3][4][2] = {
		{ { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 } }, // X
		{ { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } }, // Y
		{ { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 } }  // Z
	};
	for (int dim = 0; dim <= 2; dim++) {
		for (int i = 0; i < 4; i++) {
			Material mat1 = node->corners[edge_table[dim][i][0]];
			Material mat2 = node->corners[edge_table[dim][i][1]];
			if (mat1 == mat2)
				continue;
			if (mat1 != Material::Empty && mat2 != Material::Empty)
				continue;
			has_edges = true;
			// C++ really needs 'for static' like in D language...
			const auto &storage = (dim == 0 ? G.edges<0> () : dim == 1 ? G.edges<1> () : G.edges<2> ());
			auto edge_pos = min_corner + DC_OctreeNode::kCornerOffset[edge_table[dim][i][0]];
			auto iter = storage.findEdge (edge_pos.x, edge_pos.y, edge_pos.z);
			assert (iter != storage.end ());
			glm::vec3 vertex = iter->surfacePoint ();
			glm::vec3 normal = iter->surfaceNormal ();
			solver.addPlane (vertex, normal);
			avg_normal += normal;
		}
	}

	if (has_edges) {
		node->normal = glm::normalize (avg_normal);
		glm::vec3 lower_bound (min_corner);
		glm::vec3 upper_bound (min_corner + size);
		node->dualVertex = solver.solve (lower_bound, upper_bound);
		node->qef = solver.data ();
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
		auto pos = 2 * DC_OctreeNode::kCornerOffset[i];
		auto mat = mats[pos.y][pos.x][pos.z];
		if (mat != Material::Empty)
			mask |= uint32_t (1 << dcToMc[i]);
	}
	if (!isManifold[mask])
		return false;
	for (int i = 0; i < 8; i++) {
		uint32_t submask = 0;
		for (int j = 0; j < 8; j++) {
			auto pos = DC_OctreeNode::kCornerOffset[i] + DC_OctreeNode::kCornerOffset[j];
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
		auto pos = 2 * DC_OctreeNode::kCornerOffset[i];
		if (mat == mats[pos.y][pos.x][pos.z])
			return true;
	}
	return false;
}

}
