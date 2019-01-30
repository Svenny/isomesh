/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Storage for algorithms output
*/
#pragma once

#include <vector>

#include "../common.hpp"

namespace isomesh
{

/** \brief Simple triangle mesh storage

	This is essentially a thin wrapper around a pair of std::vectors
	(one for vertices and one for indices). It keeps its contents in
	a form suitable for direct sending to GPU (using graphics APIs).
*/
class Isomesh {
public:
	struct alignas (4) Vertex {
		Vertex (const glm::vec3 &p, const glm::vec3 &n, Material m) noexcept :
			position (p), normal (n), material (float (m)) {}
		glm::vec3 position;
		glm::vec3 normal;
		/** Material number is intended to be used in shaders as index into texture
		 array. Array texture sampling such as texture3D() in GLSL uses floating
		 point even for array index, so material needs to be converted to float. */
		float material;
	};

	Isomesh () = default;

	explicit Isomesh (size_t reserveVertices, size_t reserveIndices = 0);

	Isomesh (Isomesh &&) = default;
	Isomesh &operator = (Isomesh &&) = default;

	Isomesh (const Isomesh &) = default;
	Isomesh &operator = (const Isomesh &) = default;

	/** \brief Insert a new vertex into the mesh
		\param[in] pos Vertex position
		\param[in] normal Vertex normal
		\param[in] mat Vertex material
		\return Index of newly created vertex
	*/
	uint32_t addVertex (const glm::vec3 &pos, const glm::vec3 &normal, Material mat);

	/** \brief Insert a new triangle into the mesh
		\param[in] i1 Index of the first vertex
		\param[in] i2 Index of the second vertex
		\paran[in] i3 Index of the third vertex
		\note Don't forget about winding order
	*/
	void addTriangle (uint32_t i1, uint32_t i2, uint32_t i3);

	void clear () noexcept;

	Vertex &operator [] (uint32_t index) { return m_vertices[index]; }
	const Vertex &operator [] (uint32_t index) const { return m_vertices[index]; }

	const void *vertices () const noexcept { return m_vertices.data (); }
	const void *indices () const noexcept { return m_indices.data (); }

	size_t verticesCount () const noexcept { return m_vertices.size (); }
	size_t verticesSize () const noexcept { return m_vertices.size () * sizeof (Vertex); }
	size_t indicesCount () const noexcept { return m_indices.size (); }
	size_t indicesSize () const noexcept { return m_indices.size () * sizeof (uint32_t); }
private:
	std::vector<Vertex> m_vertices; ///< Vertices array
	std::vector<uint32_t> m_indices; ///< Indices array
};

}
