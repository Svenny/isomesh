/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/mdc_octree.hpp>

#include "../algo/edge_sets_table.h"

namespace isomesh
{

MDC_Octree::MDC_Octree (int32_t root_size, glm::dvec3 global_pos, double global_scale) :
	m_globalPos (global_pos), m_globalScale (global_scale), m_rootSize (root_size) {
	if (root_size <= 0 || (root_size & (root_size - 1)))
		throw std::invalid_argument ("Octree size is not a power of two");
}

void MDC_Octree::build (const UniformGrid &G, QefSolver3D &solver, float epsilon) {
	// Scale epsilon according to QEF scale (when translating from global coordinates to
	// local QEF value is scaled by 1/(scale^2))
	float scaled_epsilon = epsilon / float (m_globalScale * m_globalScale);
	BuildArgs args { G, solver, scaled_epsilon };
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

namespace mdc_detail
{

/* Quadruples of node children sharing an edge along some axis. First dimension - axis,
 second dimension - quadruple, third dimension - children. */
constexpr int edgeTable[3][2][4] = {
	{ { 0, 4, 5, 1 }, { 2, 6, 7, 3 } }, // X
	{ { 0, 1, 3, 2 }, { 4, 5, 7, 6 } }, // Y
	{ { 0, 2, 6, 4 }, { 1, 3, 7, 5 } }  // Z
};

template<int D>
void edgeProc (std::array<const MDC_OctreeNode *, 4> nodes, Mesh &mesh) {
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
	/* Maps node to the number of its edge (in MC order) which is shared with
	 the other nodes. Again, code may give a better description. */
	constexpr int sharedEdgeTable[3][4] = {
		{ 6, 2, 0, 4 }, // X
		{ 10, 9, 8, 11 }, // Y
		{ 5, 7, 3, 1 }  // Z
	};
	// If at least one node is not present then it is not possible to get four dual vertices
	if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3])
		return;
	const MDC_OctreeNode *sub[8];
	bool all_leaves = true;
	for (int i = 0; i < 8; i++) {
		const MDC_OctreeNode *n = nodes[subTable[D][i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (subTable[D][i][1]);
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
	const MDC_Vertex *vertices[4];
	for (int i = 0; i < 4; i++) {
		int myedge = sharedEdgeTable[D][i];
		vertices[i] = nullptr;
		for (const auto &v : nodes[i]->vertices) {
			if (v.edge_mask & uint16_t (1 << myedge)) {
				vertices[i] = &v;
				break;
			}
		}
		// Suitable vertex was not found, cannot make quad
		if (!vertices[i])
			return;
	}
	/* We assume that lower endpoint is solid. If this is not the case, the triangles'
	 winding order should be flipped to remain facing outside of the surface. */
	bool flip = true; // TODO: fix it
	uint32_t id0 = vertices[0]->vertex_id;
	uint32_t id1 = vertices[1]->vertex_id;
	uint32_t id2 = vertices[2]->vertex_id;
	uint32_t id3 = vertices[3]->vertex_id;
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

void faceProcX (std::array<const MDC_OctreeNode *, 2> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 2 }, { 0, 3 }, { 1, 0 }, { 1, 1 },
		{ 0, 6 }, { 0, 7 }, { 1, 4 }, { 1, 5 }
	};
	assert (nodes[0] && nodes[1]);
	const MDC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const MDC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (subTable[i][1]);
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

void faceProcY (std::array<const MDC_OctreeNode *, 2> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 4 }, { 0, 5 }, { 0, 6 }, { 0, 7 },
		{ 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 }
	};
	assert (nodes[0] && nodes[1]);
	const MDC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const MDC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (subTable[i][1]);
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

void faceProcZ (std::array<const MDC_OctreeNode *, 2> nodes, Mesh &mesh) {
	constexpr int subTable[8][2] = {
		{ 0, 1 }, { 1, 0 }, { 0, 3 }, { 1, 2 },
		{ 0, 5 }, { 1, 4 }, { 0, 7 }, { 1, 6 }
	};
	assert (nodes[0] && nodes[1]);
	const MDC_OctreeNode *sub[8];
	bool has_lesser = false;
	for (int i = 0; i < 8; i++) {
		const MDC_OctreeNode *n = nodes[subTable[i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (subTable[i][1]);
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

void cellProc (const MDC_OctreeNode *node, Mesh &mesh) {
	assert (node);
	if (node->isLeaf ())
		return;
	const MDC_OctreeNode *sub[8];
	for (int i = 0; i < 8; i++) {
		sub[i] = node->child (i);
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

bool hasCollapsibleAncestor (const MDC_Vertex *vtx) {
	while (vtx->parent) {
		vtx = vtx->parent;
		if (vtx->collapsible)
			return true;
	}
	return false;
}

void makeVertices (MDC_OctreeNode *node, Mesh &mesh) {
	for (auto &v : node->vertices) {
		if (v.collapsible && !hasCollapsibleAncestor (&v)) {
			glm::vec3 vertex = v.dual_vertex;
			glm::vec3 normal = v.normal;
			Material mat = Material::Stone; // MDC doesn't support multi-material surfaces (currently?)
			v.vertex_id = mesh.addVertex (vertex, normal, mat);
		}
	}
	if (node->isSubdivided ())
		for (int i = 0; i < 8; i++)
			makeVertices (node->child (i), mesh);
}

}

using namespace mdc_detail;

Mesh MDC_Octree::contour () {
	Mesh mesh;
	makeVertices (&m_root, mesh);
	cellProc (&m_root, mesh);
	mesh.setGlobalPos (m_globalPos);
	mesh.setGlobalScale (m_globalScale);
	return mesh;
}

void MDC_Octree::buildNode (MDC_OctreeNode *node, glm::ivec3 min_corner, int32_t size, BuildArgs &args) {
	assert (node);
	if (size == 1) {
		buildLeaf (node, min_corner, size, args);
		return;
	}
	// TODO: adaptive simplification
	node->subdivide ();
	int32_t child_size = size / 2;
	for (int i = 0; i < 8; i++) {
		glm::ivec3 child_min_corner = min_corner + child_size * MDC_OctreeNode::kCornerOffset[i];
		buildNode (node->child (i), child_min_corner, child_size, args);
	}
}

void MDC_Octree::buildLeaf (MDC_OctreeNode *node, glm::ivec3 min_corner, int32_t size, BuildArgs &args) {
	QefSolver3D &solver = args.solver;
	const UniformGrid &G = args.grid;
	// Obtain solid/empty vertex mask for the grid cell
	uint32_t vertex_mask = 0;
	constexpr int dcToMc[8] = { 0, 3, 1, 2, 4, 7, 5, 6 };
	for (int i = 0; i < 8; i++)
		if (G[min_corner + MDC_OctreeNode::kCornerOffset[i]] != Material::Empty)
			vertex_mask |= uint32_t (1 << dcToMc[i]);
	// Edge endpoints, edges are enumerated in MC order, vertices are in octree order
	constexpr int edge_table[12][2] = {
		{ 0, 2 }, { 2, 3 }, { 1, 3 }, { 0, 3 },
		{ 4, 6 }, { 6, 7 }, { 5, 7 }, { 4, 5 },
		{ 0, 4 }, { 2, 6 }, { 3, 7 }, { 1, 5 }
	};
	// Edge dimensions, edges are enumerated in MC order, dimensions are in XYZ order
	constexpr int edge_dim_table[12] = {
		0, 2, 0, 2, 0, 2, 0, 2, 1, 1, 1, 1
	};
	// This mask generates up to four surface patches
	int surface_cnt = 0;
	for (int i = 0; i < 4; i++) {
		if (edgeSetsTable[vertex_mask][i] == 0)
			break;
		surface_cnt++;
	}
	node->vertices.resize (surface_cnt);
	for (int i = 0; i < surface_cnt; i++) {
		uint16_t edge_mask = edgeSetsTable[vertex_mask][i];
		solver.reset ();
		MDC_Vertex &octree_vertex = node->vertices[i];
		octree_vertex.edge_mask = edge_mask;
		for (int j = 0; j < 12; j++) {
			if (!(edge_mask & uint16_t (1 << j)))
				continue;
			int dim = edge_dim_table[j];
			// C++ really needs 'for static' like in D language...
			const auto &storage = (dim == 0 ? G.edges<0> () : dim == 1 ? G.edges<1> () : G.edges<2> ());
			auto edge_pos = min_corner + MDC_OctreeNode::kCornerOffset[edge_table[j][0]];
			auto iter = storage.findEdge (edge_pos.x, edge_pos.y, edge_pos.z);
			assert (iter != storage.end ());
			glm::vec3 vertex = iter->surfacePoint ();
			glm::vec3 normal = iter->surfaceNormal ();
			solver.addPlane (vertex, normal);
			octree_vertex.normal += normal;
		}
		octree_vertex.normal = glm::normalize (octree_vertex.normal);
		glm::vec3 lower_bound (min_corner);
		glm::vec3 upper_bound (min_corner + size);
		octree_vertex.dual_vertex = solver.solve (lower_bound, upper_bound);
		octree_vertex.qef = solver.state ();
		octree_vertex.collapsible = true;
	}
}

}
