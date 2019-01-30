/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */

#include <cassert>

#include <isomesh/data/mesh.hpp>

namespace isomesh
{

Isomesh::Isomesh (size_t reserveVertices, size_t reserveIndices) {
	m_vertices.reserve (reserveVertices);
	m_indices.reserve (reserveIndices);
}

uint32_t Isomesh::addVertex (const glm::vec3 &pos, const glm::vec3 &normal, Material mat) {
	// This shouldn't happen in practice (> 2^32 vertices)
	assert (m_vertices.size () <= std::numeric_limits<uint32_t>::max ());
	// Construct vertex at the end of array
	m_vertices.emplace_back (pos, normal, mat);
	// Index of newly created vertex
	size_t index = m_vertices.size () - 1;
	return uint32_t (index);
}

void Isomesh::addTriangle (uint32_t i1, uint32_t i2, uint32_t i3) {
	m_indices.emplace_back (i1);
	m_indices.emplace_back (i2);
	m_indices.emplace_back (i3);
}

void Isomesh::clear () noexcept {
	m_vertices.clear ();
	m_indices.clear ();
}

// TODO: implement switching to 16-bit indices when there are <= 2^16 vertices

}
