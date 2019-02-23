/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <algorithm>
#include <vector>

#include <isomesh/algo/marching_cubes.hpp>
#include "marching_cubes_tables.h"

namespace isomesh
{

static uint32_t getVertexMask (const UniformGrid &G, int32_t x, int32_t y, int32_t z) {
	uint32_t mask = 0;
	if (G.at (x + 0, y + 0, z + 0) != Material::Empty) mask |= 1;
	if (G.at (x + 1, y + 0, z + 0) != Material::Empty) mask |= 2;
	if (G.at (x + 1, y + 0, z + 1) != Material::Empty) mask |= 4;
	if (G.at (x + 0, y + 0, z + 1) != Material::Empty) mask |= 8;
	if (G.at (x + 0, y + 1, z + 0) != Material::Empty) mask |= 16;
	if (G.at (x + 1, y + 1, z + 0) != Material::Empty) mask |= 32;
	if (G.at (x + 1, y + 1, z + 1) != Material::Empty) mask |= 64;
	if (G.at (x + 0, y + 1, z + 1) != Material::Empty) mask |= 128;
	return mask;
}

// Edge-cell relationship information for surface-crossing edges
struct EdgeEntry {
	EdgeEntry (uint32_t cell, uint32_t vertex, uint8_t pos) noexcept :
		cell_idx (cell), vertex_idx (vertex), vertex_pos (pos) {}
	// Index of cell this edge belongs to
	uint32_t cell_idx;
	// Index of the corresponding vertex in output mesh
	uint32_t vertex_idx : 24;
	// Marching cubes index of the edge inside a cell
	uint32_t vertex_pos : 8;
	// Sorting array of these structs will group them by cell id
	bool operator < (const EdgeEntry &e) const noexcept { return cell_idx < e.cell_idx; }
};

Mesh marchingCubes (const UniformGrid &G) {
	Mesh mesh;
	std::vector<EdgeEntry> edges;
	const int32_t max_sz = G.maxCoord ();
	const int32_t min_sz = G.minCoord ();
	// Collect X edges
	{
		auto iter = G.xEdges ().begin ();
		auto last = G.xEdges ().end ();
		for (; iter != last; ++iter) {
			glm::ivec3 lc = iter->lesserEndpoint ();
			glm::vec3 point = iter->surfacePoint ();
			glm::vec3 normal = iter->surfaceNormal ();
			Material mat = iter->solidEndpointMaterial ();
			uint32_t id = mesh.addVertex (point, normal, mat);
			if (lc.y < max_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc), id, 0);
			if (lc.y < max_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 0, 1)), id, 2);
			if (lc.y > min_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 1, 0)), id, 4);
			if (lc.y > min_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 1, 1)), id, 6);
		}
	}
	// Collect Y edges
	{
		auto iter = G.yEdges ().begin ();
		auto last = G.yEdges ().end ();
		for (; iter != last; ++iter) {
			glm::ivec3 lc = iter->lesserEndpoint ();
			glm::vec3 point = iter->surfacePoint ();
			glm::vec3 normal = iter->surfaceNormal ();
			Material mat = iter->solidEndpointMaterial ();
			uint32_t id = mesh.addVertex (point, normal, mat);
			if (lc.x < max_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc), id, 8);
			if (lc.x < max_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 0, 1)), id, 11);
			if (lc.x > min_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 0, 0)), id, 9);
			if (lc.x > min_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 0, 1)), id, 10);
		}
	}
	// Collect Z edges
	{
		auto iter = G.zEdges ().begin ();
		auto last = G.zEdges ().end ();
		for (; iter != last; ++iter) {
			glm::ivec3 lc = iter->lesserEndpoint ();
			glm::vec3 pos = iter->surfacePoint ();
			glm::vec3 normal = iter->surfaceNormal ();
			Material mat = iter->solidEndpointMaterial ();
			uint32_t id = mesh.addVertex (pos, normal, mat);
			if (lc.x < max_sz && lc.y < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc), id, 3);
			if (lc.x < max_sz && lc.y > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 1, 0)), id, 7);
			if (lc.x > min_sz && lc.y < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 0, 0)), id, 1);
			if (lc.x > min_sz && lc.y > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 1, 0)), id, 5);
		}
	}
	// Group edges by cell ids
	std::sort (edges.begin (), edges.end ());
	// Add fake edge to make last cell processing work
	// TODO: there is a cleaner way to do this
	edges.emplace_back (~uint32_t (0), 0, 0);
	uint32_t indices[12];
	uint32_t edge_mask = 0;
	uint32_t last_cell = 0;
	for (const auto &e : edges) {
		if (e.cell_idx != last_cell) {
			auto pos = G.rawIndexToPoint (last_cell);
			uint32_t vertex_mask = getVertexMask (G, pos.x, pos.y, pos.z);
			assert (edge_mask == edgeTable[vertex_mask]);
			for (int i = 0; triTable[vertex_mask][i] != -1; i += 3) {
				int i1 = triTable[vertex_mask][i];
				int i2 = triTable[vertex_mask][i + 1];
				int i3 = triTable[vertex_mask][i + 2];
				mesh.addTriangle (indices[i1], indices[i2], indices[i3]);
			}
			last_cell = e.cell_idx;
			vertex_mask = 0;
			edge_mask = 0;
		}
		indices[e.vertex_pos] = e.vertex_idx;
		edge_mask |= (1 << e.vertex_pos);
	}
	return mesh;
}

}

