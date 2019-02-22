/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <algorithm>
#include <vector>

#include <isomesh/algo/uniform_dual_contouring.hpp>

namespace isomesh
{

// Edge-cell relationship information for surface-crossing edges
struct DualContouringEdgeEntry {
	DualContouringEdgeEntry (uint32_t cell_idx) noexcept :
		cell (cell_idx) {}
	DualContouringEdgeEntry (uint32_t cell_idx, uint32_t edge_idx, uint32_t axis) noexcept :
		cell (cell_idx), edge (edge_idx), axis (axis) {}
	// Index (in traversal order) of cell this edge belongs to
	uint32_t cell;
	// Index (in compressed storage) of edge
	uint32_t edge : 30;
	// Axis of this edge (X - 0, Y - 1, Z - 2)
	uint32_t axis : 2;
	// Sorting array of these structs will group them by cell id
	bool operator < (const DualContouringEdgeEntry &e) const noexcept { return cell < e.cell; }
};

static uint32_t generateVertex
	(const UniformGrid &G, const MaterialFilter &filter, QefSolver3D &solver,
	 uint32_t cellId, std::vector<DualContouringEdgeEntry> &edgeEntries, Mesh &out) {
	DualContouringEdgeEntry sample (cellId);
	auto entry_begin = std::lower_bound (edgeEntries.begin (), edgeEntries.end (), sample);
	auto entry_end = std::upper_bound (edgeEntries.begin (), edgeEntries.end (), sample);
	auto x_begin = G.xEdgesBegin ();
	auto y_begin = G.yEdgesBegin ();
	auto z_begin = G.zEdgesBegin ();
	solver.reset ();
	glm::vec3 avg_normal (0);
	for (auto iter = entry_begin; iter != entry_end; ++iter) {
		decltype (x_begin) edge_iter;
		glm::vec3 point;
		if (iter->axis == 0) {
			edge_iter = x_begin + iter->edge;
			point = glm::vec3 (edge_iter.localCoords ());
			point.x += edge_iter->offset;
		}
		else if (iter->axis == 1) {
			edge_iter = y_begin + iter->edge;
			point = glm::vec3 (edge_iter.localCoords ());
			point.y += edge_iter->offset;
		}
		else if (iter->axis == 2) {
			edge_iter = z_begin + iter->edge;
			point = glm::vec3 (edge_iter.localCoords ());
			point.z += edge_iter->offset;
		}
		else assert (false);
		glm::vec3 normal = edge_iter->normal;
		solver.addPlane (point, normal);
		avg_normal += normal;
	}
	glm::vec3 lowerBound = G.rawIndexToPoint (cellId);
	glm::vec3 upperBound = lowerBound + 1.0f;
	glm::vec3 pos = solver.solve (lowerBound, upperBound);
	avg_normal = glm::normalize (avg_normal);
	Material mat = filter.select (G, G.rawIndexToPoint (cellId), 0xFF);
	return out.addVertex (pos, avg_normal, mat);
}

Mesh dualContouring (const UniformGrid &G, const MaterialFilter &filter, QefSolver3D &solver) {
	std::vector<DualContouringEdgeEntry> edges;
	const int32_t max_sz = G.maxCoord ();
	const int32_t min_sz = G.minCoord ();
	// Collect X edges
	{
		auto first = G.xEdgesBegin ();
		auto last = G.xEdgesEnd ();
		for (auto iter = first; iter != last; ++iter) {
			glm::ivec3 lc = iter.localCoords ();
			uint32_t my_id = uint32_t (iter - first);
			if (lc.y < max_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc), my_id, 0);
			if (lc.y < max_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 0, 1)), my_id, 0);
			if (lc.y > min_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 1, 0)), my_id, 0);
			if (lc.y > min_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 1, 1)), my_id, 0);
		}
	}
	// Collect Y edges
	{
		auto first = G.yEdgesBegin ();
		auto last = G.yEdgesEnd ();
		for (auto iter = first; iter != last; ++iter) {
			glm::ivec3 lc = iter.localCoords ();
			uint32_t my_id = uint32_t (iter - first);
			if (lc.x < max_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc), my_id, 1);
			if (lc.x < max_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 0, 1)), my_id, 1);
			if (lc.x > min_sz && lc.z < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 0, 0)), my_id, 1);
			if (lc.x > min_sz && lc.z > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 0, 1)), my_id, 1);
		}
	}
	// Collect Z edges
	{
		auto first = G.zEdgesBegin ();
		auto last = G.zEdgesEnd ();
		for (auto iter = first; iter != last; ++iter) {
			glm::ivec3 lc = iter.localCoords ();
			uint32_t my_id = uint32_t (iter - first);
			if (lc.x < max_sz && lc.y < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc), my_id, 2);
			if (lc.x < max_sz && lc.y > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (0, 1, 0)), my_id, 2);
			if (lc.x > min_sz && lc.y < max_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 0, 0)), my_id, 2);
			if (lc.x > min_sz && lc.y > min_sz)
				edges.emplace_back (G.pointToRawIndex (lc - glm::ivec3 (1, 1, 0)), my_id, 2);
		}
	}
	// Group edges by cell ids
	std::sort (edges.begin (), edges.end ());
	const uint32_t invalid_id = ~uint32_t (0);
	std::vector<uint32_t> vertex_ids (G.dataSize (), invalid_id);
	Mesh mesh;
	// Make quads around X-edges
	{
		auto first = G.xEdgesBegin ();
		auto last = G.xEdgesEnd ();
		for (auto iter = first; iter != last; ++iter) {
			glm::ivec3 lc = iter.localCoords ();
			// Skip border edges
			if (lc.y == min_sz || lc.y == max_sz)
				continue;
			if (lc.z == min_sz || lc.z == max_sz)
				continue;
			// Adjacent cell ids
			uint32_t cell0 = G.pointToRawIndex (lc);
			uint32_t cell1 = G.pointToRawIndex (lc - glm::ivec3 (0, 0, 1));
			uint32_t cell2 = G.pointToRawIndex (lc - glm::ivec3 (0, 1, 0));
			uint32_t cell3 = G.pointToRawIndex (lc - glm::ivec3 (0, 1, 1));
			// Cells' vertex ids
			uint32_t vtx0 = vertex_ids[cell0];
			if (vtx0 == invalid_id)
				vtx0 = vertex_ids[cell0] = generateVertex (G, filter, solver, cell0, edges, mesh);
			uint32_t vtx1 = vertex_ids[cell1];
			if (vtx1 == invalid_id)
				vtx1 = vertex_ids[cell1] = generateVertex (G, filter, solver, cell1, edges, mesh);
			uint32_t vtx2 = vertex_ids[cell2];
			if (vtx2 == invalid_id)
				vtx2 = vertex_ids[cell2] = generateVertex (G, filter, solver, cell2, edges, mesh);
			uint32_t vtx3 = vertex_ids[cell3];
			if (vtx3 == invalid_id)
				vtx3 = vertex_ids[cell3] = generateVertex (G, filter, solver, cell3, edges, mesh);
			bool flip = (G[lc] == Material::Empty);
			if (!flip) {
				mesh.addTriangle (vtx0, vtx2, vtx1);
				mesh.addTriangle (vtx1, vtx2, vtx3);
			}
			else {
				mesh.addTriangle (vtx0, vtx1, vtx2);
				mesh.addTriangle (vtx1, vtx3, vtx2);
			}
		}
	}
	// Make quads around Y-edges
	{
		auto first = G.yEdgesBegin ();
		auto last = G.yEdgesEnd ();
		for (auto iter = first; iter != last; ++iter) {
			glm::ivec3 lc = iter.localCoords ();
			// Skip border edges
			if (lc.x == min_sz || lc.x == max_sz)
				continue;
			if (lc.z == min_sz || lc.z == max_sz)
				continue;
			// Adjacent cell ids
			uint32_t cell0 = G.pointToRawIndex (lc);
			uint32_t cell1 = G.pointToRawIndex (lc - glm::ivec3 (0, 0, 1));
			uint32_t cell2 = G.pointToRawIndex (lc - glm::ivec3 (1, 0, 0));
			uint32_t cell3 = G.pointToRawIndex (lc - glm::ivec3 (1, 0, 1));
			// Cells' vertex ids
			uint32_t vtx0 = vertex_ids[cell0];
			if (vtx0 == invalid_id)
				vtx0 = vertex_ids[cell0] = generateVertex (G, filter, solver, cell0, edges, mesh);
			uint32_t vtx1 = vertex_ids[cell1];
			if (vtx1 == invalid_id)
				vtx1 = vertex_ids[cell1] = generateVertex (G, filter, solver, cell1, edges, mesh);
			uint32_t vtx2 = vertex_ids[cell2];
			if (vtx2 == invalid_id)
				vtx2 = vertex_ids[cell2] = generateVertex (G, filter, solver, cell2, edges, mesh);
			uint32_t vtx3 = vertex_ids[cell3];
			if (vtx3 == invalid_id)
				vtx3 = vertex_ids[cell3] = generateVertex (G, filter, solver, cell3, edges, mesh);
			bool flip = (G[lc] == Material::Empty);
			if (!flip) {
				mesh.addTriangle (vtx0, vtx1, vtx2);
				mesh.addTriangle (vtx1, vtx3, vtx2);
			}
			else {
				mesh.addTriangle (vtx0, vtx2, vtx1);
				mesh.addTriangle (vtx1, vtx2, vtx3);
			}
		}
	}
	// Make quads around Z-edges
	{
		auto first = G.zEdgesBegin ();
		auto last = G.zEdgesEnd ();
		for (auto iter = first; iter != last; ++iter) {
			glm::ivec3 lc = iter.localCoords ();
			// Skip border edges
			if (lc.x == min_sz || lc.x == max_sz)
				continue;
			if (lc.y == min_sz || lc.y == max_sz)
				continue;
			// Adjacent cell ids
			uint32_t cell0 = G.pointToRawIndex (lc);
			uint32_t cell1 = G.pointToRawIndex (lc - glm::ivec3 (0, 1, 0));
			uint32_t cell2 = G.pointToRawIndex (lc - glm::ivec3 (1, 0, 0));
			uint32_t cell3 = G.pointToRawIndex (lc - glm::ivec3 (1, 1, 0));
			// Cells' vertex ids
			uint32_t vtx0 = vertex_ids[cell0];
			if (vtx0 == invalid_id)
				vtx0 = vertex_ids[cell0] = generateVertex (G, filter, solver, cell0, edges, mesh);
			uint32_t vtx1 = vertex_ids[cell1];
			if (vtx1 == invalid_id)
				vtx1 = vertex_ids[cell1] = generateVertex (G, filter, solver, cell1, edges, mesh);
			uint32_t vtx2 = vertex_ids[cell2];
			if (vtx2 == invalid_id)
				vtx2 = vertex_ids[cell2] = generateVertex (G, filter, solver, cell2, edges, mesh);
			uint32_t vtx3 = vertex_ids[cell3];
			if (vtx3 == invalid_id)
				vtx3 = vertex_ids[cell3] = generateVertex (G, filter, solver, cell3, edges, mesh);
			bool flip = (G[lc] == Material::Empty);
			if (!flip) {
				mesh.addTriangle (vtx0, vtx2, vtx1);
				mesh.addTriangle (vtx1, vtx2, vtx3);
			}
			else {
				mesh.addTriangle (vtx0, vtx1, vtx2);
				mesh.addTriangle (vtx1, vtx3, vtx2);
			}
		}
	}
	return mesh;
}

}
