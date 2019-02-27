/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <algorithm>
#include <vector>

#include <isomesh/algo/marching_cubes.hpp>
#include "marching_cubes_tables.h"

namespace isomesh
{

namespace mc_detail
{

// Edge-cell relationship information for surface-crossing edges
struct EdgeEntry {
	EdgeEntry (uint32_t cell_idx) noexcept :
		cellIndex (cell_idx) {}
	EdgeEntry (uint32_t cell_idx, uint32_t vertex_idx) noexcept :
		cellIndex (cell_idx), vertexIndex (vertex_idx) {}
	// Index of cell this edge belongs to
	uint32_t cellIndex;
	// Index of surface-crossing vertex in mesh
	uint32_t vertexIndex;
	// Sorting array of these structs will group them by cell id
	bool operator < (const EdgeEntry &e) const noexcept { return cellIndex < e.cellIndex; }
};

uint32_t getVertexMask (const UniformGrid &G, uint32_t cell_idx) {
	auto materials = G.materialsOfCell (cell_idx);
	uint32_t mask = 0;
	if (materials[0] != Material::Empty) mask |= uint32_t (1 << 0);
	if (materials[1] != Material::Empty) mask |= uint32_t (1 << 3);
	if (materials[2] != Material::Empty) mask |= uint32_t (1 << 1);
	if (materials[3] != Material::Empty) mask |= uint32_t (1 << 2);
	if (materials[4] != Material::Empty) mask |= uint32_t (1 << 4);
	if (materials[5] != Material::Empty) mask |= uint32_t (1 << 7);
	if (materials[6] != Material::Empty) mask |= uint32_t (1 << 5);
	if (materials[7] != Material::Empty) mask |= uint32_t (1 << 6);
	return mask;
}

template<int D>
void collectCellEdges (std::vector<EdgeEntry> &cell_edges_0, std::vector<EdgeEntry> &cell_edges_1,
                       std::vector<EdgeEntry> &cell_edges_2, std::vector<EdgeEntry> &cell_edges_3,
                       Mesh &mesh, const UniformGrid &G) {
	for (const auto &edge : G.edges<D> ()) {
		// Add this edge's vertex to mesh
		glm::vec3 point = edge.surfacePoint ();
		glm::vec3 normal = edge.surfaceNormal ();
		Material mat = edge.solidEndpointMaterial ();
		uint32_t vertex_idx = mesh.addVertex (point, normal, mat);
		// Add this edge to adjacent cells
		glm::ivec3 edge_pos = edge.lesserEndpoint ();
		auto cells = G.adjacentCellsForEdge<D> (edge_pos);
		if (cells[0] != G.kBadIndex)
			cell_edges_0.emplace_back (cells[0], vertex_idx);
		if (cells[1] != G.kBadIndex)
			cell_edges_1.emplace_back (cells[1], vertex_idx);
		if (cells[2] != G.kBadIndex)
			cell_edges_2.emplace_back (cells[2], vertex_idx);
		if (cells[3] != G.kBadIndex)
			cell_edges_3.emplace_back (cells[3], vertex_idx);
	}
}

}

using namespace mc_detail;

Mesh marchingCubes (const UniformGrid &G) {
	Mesh mesh;
	std::vector<EdgeEntry> cell_edges[12];
	collectCellEdges<0> (cell_edges[6], cell_edges[2], cell_edges[0], cell_edges[4], mesh, G); // X
	collectCellEdges<1> (cell_edges[10], cell_edges[9], cell_edges[8], cell_edges[11], mesh, G); // Y
	collectCellEdges<2> (cell_edges[5], cell_edges[7], cell_edges[3], cell_edges[1], mesh, G); // Z
	std::vector<EdgeEntry>::const_iterator cell_edge_iters[12];
	// Group edges by cell ids
	for (int i = 0; i < 12; i++) {
		std::sort (cell_edges[i].begin (), cell_edges[i].end ());
		cell_edge_iters[i] = cell_edges[i].begin ();
	}
	while (true) {
		uint32_t min_unprocessed_cell = G.kBadIndex;
		for (int i = 0; i < 12; i++) {
			if (cell_edge_iters[i] != cell_edges[i].end ())
				min_unprocessed_cell = glm::min (min_unprocessed_cell, cell_edge_iters[i]->cellIndex);
		}
		// All cells are processed
		if (min_unprocessed_cell == G.kBadIndex)
			break;
		uint32_t cell_idx = min_unprocessed_cell;
		uint32_t vertex_idx[12];
		uint32_t edge_mask = 0;
		for (int i = 0; i < 12; i++) {
			auto &iter = cell_edge_iters[i];
			if (iter != cell_edges[i].end () && iter->cellIndex == cell_idx) {
				vertex_idx[i] = iter->vertexIndex;
				edge_mask |= uint32_t (1 << i);
				++iter;
			}
		}
		uint32_t vertex_mask = getVertexMask (G, cell_idx);
		assert (edge_mask == edgeTable[vertex_mask]);
		for (int i = 0; triTable[vertex_mask][i] != -1; i += 3) {
			int i1 = triTable[vertex_mask][i];
			int i2 = triTable[vertex_mask][i + 1];
			int i3 = triTable[vertex_mask][i + 2];
			mesh.addTriangle (vertex_idx[i1], vertex_idx[i2], vertex_idx[i3]);
		}
	}
	return mesh;
}

}

