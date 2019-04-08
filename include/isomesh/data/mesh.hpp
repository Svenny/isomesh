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
class Mesh {
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

	Mesh () = default;

	explicit Mesh (size_t reserveVertices, size_t reserveIndices = 0);

	Mesh (Mesh &&) = default;
	Mesh &operator = (Mesh &&) = default;

	Mesh (const Mesh &) = default;
	Mesh &operator = (const Mesh &) = default;

	/** \brief Insert a new vertex into the mesh
		\param[in] pos Vertex position
		\param[in] normal Vertex normal
		\param[in] mat Vertex material
		\return Index of newly created vertex, use it in \ref addTriangle
	*/
	uint32_t addVertex (const glm::vec3 &pos, const glm::vec3 &normal, Material mat);

	/** \brief Insert a new triangle into the mesh
		\param[in] i1 Index of the first vertex
		\param[in] i2 Index of the second vertex
		\paran[in] i3 Index of the third vertex
		\note Vertex indices are returned from \ref addVertex
		\note Don't forget about winding order
	*/
	void addTriangle (uint32_t i1, uint32_t i2, uint32_t i3);

	/** \brief Clears mesh data

		This removes all added vertices and indices.
	*/
	void clear () noexcept;

	Vertex &operator [] (uint32_t index) { return m_vertices[index]; }
	const Vertex &operator [] (uint32_t index) const { return m_vertices[index]; }

	const void *vertexData () const noexcept { return m_vertices.data (); }
	const void *indexData () const noexcept { return m_indices.data (); }

	/// Number of vertices in mesh
	size_t vertexCount () const noexcept { return m_vertices.size (); }
	/// Size of vertex data in bytes
	size_t vertexBytes () const noexcept { return m_vertices.size () * sizeof (Vertex); }
	/// Number of indices in mesh
	size_t indexCount () const noexcept { return m_indices.size (); }
	/// Size of index data in bytes
	size_t indexBytes () const noexcept { return m_indices.size () * sizeof (uint32_t); }
	
	glm::dvec3 globalPos () const noexcept { return m_globalPos; }
	void setGlobalPos (const glm::dvec3 &value) noexcept { m_globalPos = value; }
	double globalScale () const noexcept { return m_globalScale; }
	void setGlobalScale (double value) noexcept { m_globalScale = value; }
private:
	std::vector<Vertex> m_vertices; ///< Vertices array
	std::vector<uint32_t> m_indices; ///< Indices array
	glm::dvec3 m_globalPos { 0 }; ///< Position of local space origin in global space
	double m_globalScale = 1; ///< Scaling needed to transform mesh to global space
};

}
