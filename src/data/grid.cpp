/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <algorithm>
#include <cassert>
#include <stdexcept>

#include <isomesh/util/material_selector.hpp>
#include <isomesh/util/zero_finder.hpp>
#include <isomesh/data/grid.hpp>

namespace isomesh
{

UniformGrid::UniformGrid (uint32_t size, const glm::dvec3 &globalPos, double scale) :
	m_size (size), m_halfSize (int32_t (size) / 2), m_globalPos (globalPos), m_scale (scale) {
	if (size < 2)
		throw std::invalid_argument ("Grid size should be at least two");
	if (size & (size - 1))
		throw std::invalid_argument ("Grid size is not a power of two");
	if (size > 128)
		throw std::length_error ("Too large grid size (> 128)");

	m_mat.reset (new Material[(size + 1) * (size + 1) * (size + 1)]);
}

void UniformGrid::fill (const SurfaceFunction &f,
                        const ZeroFinder &solver,
                        const MaterialSelector &material) {
	std::vector<double> values (dataSize ());
	// Compute function values over the whole grid
	uint32_t idx = 0;
	glm::dvec3 lowest_point = localToGlobal (glm::dvec3 (-m_halfSize));
	glm::dvec3 call_pos = lowest_point;
	for (int32_t y = -m_halfSize; y <= m_halfSize; y++) {
		call_pos.x = lowest_point.x;
		for (int32_t x = -m_halfSize; x <= m_halfSize; x++) {
			call_pos.z = lowest_point.z;
			for (int32_t z = -m_halfSize; z <= m_halfSize; z++) {
				values[idx] = f (call_pos);
				if (values[idx] > 0)
					m_mat[idx] = Material::Empty;
				else
					m_mat[idx] = material.select (call_pos, values[idx]);
				idx++;
				call_pos.z += m_scale;
			}
			call_pos.x += m_scale;
		}
		call_pos.y += m_scale;
	}
	/* Find zero intersections on grid edges.
	 Different signs on edge endpoints means there is at least
	 one zero intersection on this edge. We assume that there is
	 exactly one and find it using supplied solver. */
	// Along X
	m_edgeX.clear ();
	uint32_t idx1 = 0;
	uint32_t idx2 = m_size + 1;
	for (int32_t y = -m_halfSize; y <= m_halfSize; y++) {
		for (int32_t x = -m_halfSize; x < m_halfSize; x++) { // x < -m_halfSize, this is intended
			for (int32_t z = -m_halfSize; z <= m_halfSize; z++) {
				bool sign1 = (values[idx1] <= 0.0);
				bool sign2 = (values[idx2] <= 0.0);
				if (sign1 != sign2) {
					glm::dvec3 p = localToGlobal (glm::dvec3 (x, y, z));
					double x0 = p.x;
					double x1 = p.x + m_scale;
					p.x = solver.findAlongX (x0, p.y, p.z, x1, values[idx1], values[idx2], f);
					glm::dvec3 grad = f.grad (p);
					double offset = (p.x - x0) / m_scale;
					Material mat = sign1 ? m_mat[idx1] : m_mat[idx2];
					m_edgeX.addEdge (x, y, z, grad, offset, 0, sign1, mat);
				}
				idx1++;
				idx2++;
			}
		}
		idx1 += m_size + 1;
		idx2 += m_size + 1;
	}
	// Along Y
	m_edgeY.clear ();
	idx1 = 0;
	idx2 = (m_size + 1) * (m_size + 1);
	for (int32_t y = -m_halfSize; y < m_halfSize; y++) { // y < -m_halfSize, this is intended
		for (int32_t x = -m_halfSize; x <= m_halfSize; x++) {
			for (int32_t z = -m_halfSize; z <= m_halfSize; z++) {
				bool sign1 = (values[idx1] <= 0.0);
				bool sign2 = (values[idx2] <= 0.0);
				if (sign1 != sign2) {
					glm::dvec3 p = localToGlobal (glm::dvec3 (x, y, z));
					double y0 = p.y;
					double y1 = p.y + m_scale;
					p.y = solver.findAlongY (p.x, y0, p.z, y1, values[idx1], values[idx2], f);
					glm::dvec3 grad = f.grad (p);
					double offset = (p.y - y0) / m_scale;
					Material mat = sign1 ? m_mat[idx1] : m_mat[idx2];
					m_edgeY.addEdge (x, y, z, grad, offset, 1, sign1, mat);
				}
				idx1++;
				idx2++;
			}
		}
	}
	// Along Z
	m_edgeZ.clear ();
	idx1 = 0;
	idx2 = 1;
	for (int32_t y = -m_halfSize; y <= m_halfSize; y++) {
		for (int32_t x = -m_halfSize; x <= m_halfSize; x++) {
			for (int32_t z = -m_halfSize; z < m_halfSize; z++) { // z < -m_halfSize, this is intended
				bool sign1 = (values[idx1] <= 0.0);
				bool sign2 = (values[idx2] <= 0.0);
				if (sign1 != sign2) {
					glm::dvec3 p = localToGlobal (glm::dvec3 (x, y, z));
					double z0 = p.z;
					double z1 = p.z + m_scale;
					p.z = solver.findAlongZ (p.x, p.y, z0, z1, values[idx1], values[idx2], f);
					glm::dvec3 grad = f.grad (p);
					double offset = (p.z - z0) / m_scale;
					Material mat = sign1 ? m_mat[idx1] : m_mat[idx2];
					m_edgeZ.addEdge (x, y, z, grad, offset, 2, sign1, mat);
				}
				idx1++;
				idx2++;
			}
			idx1++;
			idx2++;
		}
	}
}

uint32_t UniformGrid::pointToRawIndex (int32_t x, int32_t y, int32_t z) const {
	checkPoint (x, y, z);
	uint32_t idx = uint32_t (y + m_halfSize) * (m_size + 1);
	idx = (idx + uint32_t (x + m_halfSize)) * (m_size + 1);
	idx += uint32_t (z + m_halfSize);
	return idx;
}

uint32_t UniformGrid::pointToRawIndex (const glm::ivec3 &v) const {
	return pointToRawIndex (v.x, v.y, v.z);
}

glm::ivec3 UniformGrid::rawIndexToPoint (uint32_t idx) const noexcept {
	int32_t z = int32_t (idx % (m_size + 1)) - m_halfSize;
	idx /= (m_size + 1);
	int32_t x = int32_t (idx % (m_size + 1)) - m_halfSize;
	idx /= (m_size + 1);
	int32_t y = int32_t (idx) - m_halfSize;
	return glm::ivec3 (x, y, z);
}

Material UniformGrid::at (int32_t x, int32_t y, int32_t z) const {
	checkPoint (x, y, z);
	return m_mat[pointToRawIndex (x, y, z)];
}

Material UniformGrid::operator [] (const glm::ivec3 &v) const {
	return at (v.x, v.y, v.z);
}

void UniformGrid::checkPoint (int32_t x, int32_t y, int32_t z) const {
	assert (x >= -m_halfSize && y >= -m_halfSize && z >= -m_halfSize);
	assert (x <= m_halfSize && y <= m_halfSize && z <= m_halfSize);
}

}
