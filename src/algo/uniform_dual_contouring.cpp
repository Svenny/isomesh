/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/algo/uniform_dual_contouring.hpp>
#include <isomesh/util/material_filter.hpp>

#include <algorithm>
#include <vector>

namespace isomesh
{

namespace dc_detail
{

// Edge-cell relationship information for surface-crossing edges
struct EdgeEntry {
	EdgeEntry (uint32_t cell_idx) noexcept :
		cellIndex (cell_idx) {}
	EdgeEntry (uint32_t cell_idx, UniformGridEdgeStorage::const_iterator edge_iter) noexcept :
		cellIndex (cell_idx), edgeIter (edge_iter) {}
	// Index of cell this edge belongs to
	uint32_t cellIndex;
	// Iterator to the edge
	UniformGridEdgeStorage::const_iterator edgeIter;
	// Sorting array of these structs will group them by cell id
	bool operator < (const EdgeEntry &e) const noexcept { return cellIndex < e.cellIndex; }
};

void generateDualVertices (const UniformGrid &G, QefSolver3D &solver, Mesh &mesh,
                           const std::vector<EdgeEntry> &cell_edges, std::vector<uint32_t> &dual_vertex_ids) {
	auto iter = cell_edges.begin ();
	MaterialFilter filter;
	while (iter != cell_edges.end ()) {
		uint32_t cell_idx = iter->cellIndex;
		solver.reset ();
		glm::vec3 avg_normal (0);
		do {
			glm::vec3 point = iter->edgeIter->surfacePoint ();
			glm::vec3 normal = iter->edgeIter->surfaceNormal ();
			solver.addPlane (point, normal);
			avg_normal += normal;
			++iter;
		} while (iter != cell_edges.end () && iter->cellIndex == cell_idx);
		glm::vec3 lower_bound = G.indexToPoint (cell_idx);
		glm::vec3 upper_bound = lower_bound + 1.0f;
		glm::vec3 dual_vertex = solver.solve (lower_bound, upper_bound);
		avg_normal = glm::normalize (avg_normal);
		filter.reset ();
		filter.add (G.materialsOfCell (cell_idx));
		Material mat = filter.select ();
		dual_vertex_ids[cell_idx] = mesh.addVertex (dual_vertex, avg_normal, mat);
	}
}

template<int D>
void collectCellEdges (std::vector<EdgeEntry> &cell_edges, const UniformGrid &G) {
	auto first = G.edges<D> ().cbegin ();
	auto last = G.edges<D> ().cend ();
	for (auto iter = first; iter != last; ++iter) {
		glm::ivec3 edge_pos = iter->lesserEndpoint ();
		auto cells = G.adjacentCellsForEdge<D> (edge_pos);
		for (uint32_t cell_idx : cells)
			if (cell_idx != kBadIndex)
				cell_edges.emplace_back (cell_idx, iter);
	}
}

template<int D>
void generateQuads (const std::vector<uint32_t> &dual_vertex_ids, Mesh &mesh, const UniformGrid &G) {
	for (const auto &edge : G.edges<D> ()) {
		glm::ivec3 edge_pos = edge.lesserEndpoint ();
		auto cells = G.adjacentCellsForEdge<D> (edge_pos);
		// Border edges lack some adjacent cells, skip them
		if (cells[0] == kBadIndex || cells[1] == kBadIndex ||
			 cells[2] == kBadIndex || cells[3] == kBadIndex)
			continue;
		uint32_t vtx0 = dual_vertex_ids[cells[0]];
		uint32_t vtx1 = dual_vertex_ids[cells[1]];
		uint32_t vtx2 = dual_vertex_ids[cells[2]];
		uint32_t vtx3 = dual_vertex_ids[cells[3]];
		assert (vtx0 != kBadIndex && vtx1 != kBadIndex &&
		        vtx2 != kBadIndex && vtx3 != kBadIndex);
		bool flip = !edge.isLesserEndpointSolid ();
		if (!flip) {
			mesh.addTriangle (vtx0, vtx1, vtx3);
			mesh.addTriangle (vtx1, vtx2, vtx3);
		}
		else {
			mesh.addTriangle (vtx0, vtx3, vtx1);
			mesh.addTriangle (vtx1, vtx3, vtx2);
		}
	}
}

}

using namespace dc_detail;

Mesh dualContouring (const UniformGrid &G, QefSolver3D &solver) {
	size_t edges_count = G.edges<0> ().size () + G.edges<1> ().size () + G.edges<2> ().size ();
	std::vector<EdgeEntry> cell_edges;
	// Each edge provides up to four entries
	cell_edges.reserve (4 * edges_count);
	collectCellEdges<0> (cell_edges, G); // X
	collectCellEdges<1> (cell_edges, G); // Y
	collectCellEdges<2> (cell_edges, G); // Z
	// Group edges by cell ids
	std::sort (cell_edges.begin (), cell_edges.end ());
	std::vector<uint32_t> dual_vertex_ids (G.dataSize (), kBadIndex);
	// Rough estimation that vertices count is equal to edges count
	// and each vertex is shared by six triangles
	Mesh mesh (edges_count, 6 * edges_count);
	generateDualVertices (G, solver, mesh, cell_edges, dual_vertex_ids);
	generateQuads<0> (dual_vertex_ids, mesh, G); // X
	generateQuads<1> (dual_vertex_ids, mesh, G); // Y
	generateQuads<2> (dual_vertex_ids, mesh, G); // Z
	mesh.setGlobalPos (G.globalPosition ());
	mesh.setGlobalScale (G.gridStep ());
	return mesh;
}

}
