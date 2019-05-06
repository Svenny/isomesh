/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/mdc_octree.hpp>
#include <isomesh/util/material_filter.hpp>
#include <isomesh/util/tables.hpp>

#include "../private/disjoint_set_union.hpp"

#include <cassert>

namespace isomesh
{

MDC_Octree::MDC_Octree (int32_t root_size, glm::dvec3 global_pos, double global_scale) :
	m_globalPos (global_pos), m_globalScale (global_scale), m_rootSize (root_size) {
	if (root_size <= 0 || (root_size & (root_size - 1)))
		throw std::invalid_argument ("Octree size is not a power of two");
}

namespace mdc_detail
{

using std::array;

template<int D>
void edgeProcLeaves (array<const MDC_OctreeNode *, 4> nodes, Mesh &mesh) {
	const MDC_Vertex *vertices[4];
	for (int i = 0; i < 4; i++) {
		int myedge = kEdgeProcSharedEdge[D][i];
		int idx = kMdcEdgeSetIndex[nodes[i]->vertexMask ()][myedge];
		if (idx < 0 || nodes[i]->m_vertices.size () <= idx)
			return;
		vertices[i] = nodes[i]->m_vertices[idx].highestCollapsibleAncestor ();
		if (!vertices[i])
			return;
	}
	/* We assume that lower endpoint of the minimal edge is solid. If this is not the case,
	 the triangles' winding order should be flipped to remain facing outside the surface. */
	bool flip;
	{
		int deepest_idx = 0;
		auto max_depth = nodes[0]->depth ();
		for (int i = 1; i < 4; i++) {
			if (nodes[i]->depth () > max_depth) {
				max_depth = nodes[i]->depth ();
				deepest_idx = i;
			}
		}
		int myedge = kEdgeProcSharedEdge[D][deepest_idx];
		const MDC_OctreeNode *n = nodes[deepest_idx];
		if (n->corner (kCellEdgeEndpoint[myedge][0]) == Material::Empty)
			flip = true;
		else flip = false;
	}
	uint32_t id0 = vertices[0]->m_vertexIdx;
	uint32_t id1 = vertices[1]->m_vertexIdx;
	uint32_t id2 = vertices[2]->m_vertexIdx;
	uint32_t id3 = vertices[3]->m_vertexIdx;
	if (!flip) {
		mesh.addTriangle (id0, id1, id2);
		mesh.addTriangle (id0, id2, id3);
	}
	else {
		mesh.addTriangle (id0, id2, id1);
		mesh.addTriangle (id0, id3, id2);
	}
}

#define callSubEdgeProc(DIM) \
	for (int i = 0; i < 2; i++) { \
		auto i1 = kEdgeProcCallTable[DIM][i][0]; \
		auto i2 = kEdgeProcCallTable[DIM][i][1]; \
		auto i3 = kEdgeProcCallTable[DIM][i][2]; \
		auto i4 = kEdgeProcCallTable[DIM][i][3]; \
		edgeProc<DIM> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, mesh); \
	}

#define callSubFaceProc(DIM) \
	for (int i = 0; i < 4; i++) { \
		auto i1 = kFaceProcCallTable[DIM][i][0]; \
		auto i2 = kFaceProcCallTable[DIM][i][1]; \
		faceProc<DIM> ({ sub[i1], sub[i2] }, mesh); \
	}

template<int D>
void edgeProc (array<const MDC_OctreeNode *, 4> nodes, Mesh &mesh) {
	const MDC_OctreeNode *sub[8];
	bool all_leaves = true;
	for (int i = 0; i < 8; i++) {
		const MDC_OctreeNode *n = nodes[kEdgeProcChildTable[D][i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (kEdgeProcChildTable[D][i][1]);
			all_leaves = false;
		}
		else sub[i] = n;
	}
	if (all_leaves) {
		edgeProcLeaves<D> (nodes, mesh);
		return;
	}
	callSubEdgeProc (D);
}

template<int D>
void faceProc (array<const MDC_OctreeNode *, 2> nodes, Mesh &mesh) {
	const MDC_OctreeNode *sub[8];
	bool all_leaves = true;
	for (int i = 0; i < 8; i++) {
		const MDC_OctreeNode *n = nodes[kFaceProcChildTable[D][i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (kFaceProcChildTable[D][i][1]);
			all_leaves = false;
		}
		else sub[i] = n;
	}
	if (all_leaves)
		return;
	callSubFaceProc (D);
	constexpr int D2 = (D + 1) % 3;
	constexpr int D3 = (D + 2) % 3;
	callSubEdgeProc (D2);
	callSubEdgeProc (D3);
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
	callSubFaceProc (0);
	callSubFaceProc (1);
	callSubFaceProc (2);
	callSubEdgeProc (0);
	callSubEdgeProc (1);
	callSubEdgeProc (2);
}

void addVerticesToMesh (MDC_OctreeNode *node, Mesh &mesh, float epsilon) {
	for (auto &v : node->m_vertices) {
		if (node->isLeaf ())
			v.m_collapsible = true;
		else
			v.m_collapsible = v.m_error <= epsilon;
		if (v.m_collapsible && !v.hasCollapsibleAncestor ()) {
			glm::vec3 vertex = v.m_position;
			glm::vec3 normal = v.m_normal;
			Material mat = v.m_material;
			v.m_vertexIdx = mesh.addVertex (vertex, normal, mat);
		}
		else v.m_vertexIdx = kBadIndex;
	}
	if (node->isSubdivided ())
		for (int i = 0; i < 8; i++)
			addVerticesToMesh (node->child (i), mesh, epsilon);
}

using DSU = DisjointSetUnion<uint32_t>;
using VertexArray = std::vector<MDC_Vertex *>;

template<int D>
void clusterEdgeLeaves (array<MDC_OctreeNode *, 4> nodes, DSU &dsu, VertexArray &vertices) {
	MDC_Vertex *vertex[4];
	for (int i = 0; i < 4; i++) {
		int myedge = kEdgeProcSharedEdge[D][i];
		int idx = kMdcEdgeSetIndex[nodes[i]->vertexMask ()][myedge];
		if (idx < 0 || nodes[i]->m_vertices.size () <= idx)
			return;
		vertex[i] = nodes[i]->m_vertices[idx].highestAncestor ();
		if (vertex[i]->m_surfaceIdx == kBadIndex) {
			vertices.emplace_back (vertex[i]);
			vertex[i]->m_surfaceIdx = dsu.addSet ();
		}
	}
	dsu.mergeSets (vertex[0]->m_surfaceIdx, vertex[1]->m_surfaceIdx);
	dsu.mergeSets (vertex[2]->m_surfaceIdx, vertex[3]->m_surfaceIdx);
	dsu.mergeSets (vertex[0]->m_surfaceIdx, vertex[2]->m_surfaceIdx);
}

#define callSubClusterEdge(DIM) \
	for (int i = 0; i < 2; i++) { \
		auto i1 = kEdgeProcCallTable[DIM][i][0]; \
		auto i2 = kEdgeProcCallTable[DIM][i][1]; \
		auto i3 = kEdgeProcCallTable[DIM][i][2]; \
		auto i4 = kEdgeProcCallTable[DIM][i][3]; \
		clusterEdge<DIM> ({ sub[i1], sub[i2], sub[i3], sub[i4] }, dsu, vertices); \
	}

#define callSubClusterFace(DIM) \
	for (int i = 0; i < 4; i++) { \
		auto i1 = kFaceProcCallTable[DIM][i][0]; \
		auto i2 = kFaceProcCallTable[DIM][i][1]; \
		clusterFace<DIM> ({ sub[i1], sub[i2] }, dsu, vertices); \
	}

template<int D>
void clusterEdge (array<MDC_OctreeNode *, 4> nodes, DSU &dsu, VertexArray &vertices) {
	MDC_OctreeNode *sub[8];
	bool all_leaves = true;
	for (int i = 0; i < 8; i++) {
		MDC_OctreeNode *n = nodes[kEdgeProcChildTable[D][i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (kEdgeProcChildTable[D][i][1]);
			all_leaves = false;
		}
		else sub[i] = n;
	}
	if (all_leaves) {
		clusterEdgeLeaves<D> (nodes, dsu, vertices);
		return;
	}
	callSubClusterEdge (D);
}

template<int D>
void clusterFace (array<MDC_OctreeNode *, 2> nodes, DSU &dsu, VertexArray &vertices) {
	MDC_OctreeNode *sub[8];
	bool all_leaves = true;
	for (int i = 0; i < 8; i++) {
		MDC_OctreeNode *n = nodes[kFaceProcChildTable[D][i][0]];
		if (n->isSubdivided ()) {
			sub[i] = n->child (kFaceProcChildTable[D][i][1]);
			all_leaves = false;
		}
		else sub[i] = n;
	}
	if (all_leaves)
		return;
	callSubClusterFace (D);
	constexpr int D2 = (D + 1) % 3;
	constexpr int D3 = (D + 2) % 3;
	callSubClusterEdge (D2);
	callSubClusterEdge (D3);
}

void clusterCell (MDC_OctreeNode *node, glm::ivec3 min_corner, int32_t size, QefSolver3D &orig_solver) {
	assert (node);
	if (node->isLeaf ())
		return;
	MDC_OctreeNode *sub[8];
	for (int i = 0; i < 8; i++) {
		sub[i] = node->child (i);
		int32_t sub_size = size / 2;
		glm::ivec3 sub_pos = min_corner + sub_size * kCellCornerOffset[i];
		clusterCell (sub[i], sub_pos, sub_size, orig_solver);
	}
	DSU dsu;
	VertexArray vertices;
	callSubClusterFace (0);
	callSubClusterFace (1);
	callSubClusterFace (2);
	callSubClusterEdge (0);
	callSubClusterEdge (1);
	callSubClusterEdge (2);
	uint32_t new_cnt = 0;
	std::vector<uint32_t> set_idx (dsu.size (), kBadIndex);
	for (uint32_t i = 0; i < dsu.size (); i++) {
		uint32_t leader = dsu.getSetLeader (i);
		if (set_idx[leader] == kBadIndex) {
			set_idx[leader] = new_cnt;
			new_cnt++;
		}
		set_idx[i] = set_idx[leader];
	}
	node->m_vertices.resize (new_cnt);
	orig_solver.reset ();
	std::vector<QefSolver3D> solver (new_cnt);
	for (auto &s : solver)
		s = orig_solver;
	std::vector<MaterialFilter> filter (new_cnt);
	std::vector<glm::vec3> avg_normal (new_cnt, glm::vec3 (0.0f));
	for (MDC_Vertex *v : vertices) {
		uint32_t idx = set_idx[v->m_surfaceIdx];
		v->m_surfaceIdx = kBadIndex;
		solver[idx].merge (v->m_qef);
		avg_normal[idx] += v->m_normal;
		filter[idx].add (v->m_material);
		v->m_parent = &node->m_vertices[idx];
	}
	const glm::vec3 lower_bound (min_corner);
	const glm::vec3 upper_bound (min_corner + size);
	for (uint32_t i = 0; i < new_cnt; i++) {
		glm::vec3 point = solver[i].solve (lower_bound, upper_bound);
		node->m_vertices[i].m_position = point;
		node->m_vertices[i].m_normal = glm::normalize (avg_normal[i]);
		node->m_vertices[i].m_material = filter[i].select ();
		node->m_vertices[i].m_error = solver[i].eval (point);
		node->m_vertices[i].m_qef = solver[i].state ();
	}
}

}

using namespace mdc_detail;

void MDC_Octree::build (const UniformGrid &G, QefSolver3D &solver) {
	BuildArgs args { G, solver };
	try {
		m_root.collapse ();
		glm::ivec3 min_corner (-m_rootSize / 2);
		buildNode (&m_root, min_corner, m_rootSize, args);
		clusterCell (&m_root, min_corner, m_rootSize, solver);
	}
	catch (...) {
		auto e = std::current_exception ();
		m_root.collapse ();
		std::rethrow_exception (e);
	}
}

Mesh MDC_Octree::contour (float epsilon) {
	// Scale epsilon according to QEF scale (when translating from global coordinates to
	// local the QEF value is scaled by 1/(scale^2))
	float scaled_epsilon = epsilon / float (m_globalScale * m_globalScale);
	Mesh mesh;
	addVerticesToMesh (&m_root, mesh, scaled_epsilon);
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
	node->subdivide ();
	int32_t child_size = size / 2;
	for (int i = 0; i < 8; i++) {
		glm::ivec3 child_min_corner = min_corner + child_size * kCellCornerOffset[i];
		buildNode (node->child (i), child_min_corner, child_size, args);
	}
}

void MDC_Octree::buildLeaf (MDC_OctreeNode *node, glm::ivec3 min_corner, int32_t size, BuildArgs &args) {
	// Obtain solid/empty vertex mask for the grid cell
	auto corners = args.grid.materialsOfCell (args.grid.pointToIndex (min_corner));
	uint8_t vertex_mask = 0;
	for (uint8_t i = 0; i < 8; i++)
		if (corners[i] != Material::Empty)
			vertex_mask |= (1 << i);
	node->setCorners (corners);
	node->setVertexMask (vertex_mask);
	// Up to four vertices will be generated, so we'll need to keep up to four
	// solver states, material filters and average normals simultaneously
	args.solver.reset ();
	QefSolver3D solver[4];
	MaterialFilter filter[4];
	glm::vec3 avg_normal[4];
	for (int i = 0; i < 4; i++) {
		solver[i] = args.solver;
		avg_normal[i] = glm::vec3 (0);
	}
	// This flag array prevents adding the same vertex to the filter
	// multiple times (from multiple edges), thus making material filtering fair
	std::array<bool, 8> corner_used;
	corner_used.fill (false);
	const UniformGridEdgeStorage *storages[3] = {
		&args.grid.edges<0> (), &args.grid.edges<1> (), &args.grid.edges<2> ()
	};
	int surface_cnt = 0;
	for (int i = 0; i < 12; i++) {
		int idx = kMdcEdgeSetIndex[vertex_mask][i];
		if (idx < 0)
			continue;
		surface_cnt = std::max (surface_cnt, idx + 1);
		const auto &storage = *storages[kCellEdgeDirection[i]];
		auto edge_pos = min_corner + kCellCornerOffset[kCellEdgeEndpoint[i][0]];
		auto iter = storage.findEdge (edge_pos.x, edge_pos.y, edge_pos.z);
		assert (iter != storage.end ());
		glm::vec3 point = iter->surfacePoint ();
		glm::vec3 normal = iter->surfaceNormal ();
		solver[idx].addPlane (point, normal);
		avg_normal[idx] += normal;
		Material mat = iter->solidEndpointMaterial ();
		int corner_idx = (iter->isLesserEndpointSolid () ? kCellEdgeEndpoint[i][0] : kCellEdgeEndpoint[i][1]);
		if (!corner_used[corner_idx]) {
			filter[idx].add (mat);
			corner_used[corner_idx] = true;
		}
	}
	node->m_vertices.resize (surface_cnt);
	const glm::vec3 lower_bound (min_corner);
	const glm::vec3 upper_bound (min_corner + size);
	for (int i = 0; i < surface_cnt; i++) {
		glm::vec3 point = solver[i].solve (lower_bound, upper_bound);
		node->m_vertices[i].m_position = point;
		node->m_vertices[i].m_normal = glm::normalize (avg_normal[i]);
		node->m_vertices[i].m_material = filter[i].select ();
		node->m_vertices[i].m_error = solver[i].eval (point);
		node->m_vertices[i].m_qef = solver[i].state ();
	}
}

}
