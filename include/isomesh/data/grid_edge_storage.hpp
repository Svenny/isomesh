/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Compressed storage of uniform grid edges
*/
#pragma once

#include "../common.hpp"

#include <iterator>

namespace isomesh
{

/** \brief Surface-crossing edge of uniform grid

	The edge is directed along one of the coordinate axes (X, Y or Z) and has two endpoints.
	These endpoints have integer coordinates (in local space) which differ by one in coordinate
	along which the edge is directed. The lesser endpoint is the one with the smaller such
	coordinate. The bigger endpoint is, consequently, the one with the larger coordinate. Each
	of the endpoints is a vertex in the uniform grid with associated material. As this edge crosses
	the surface, one of the endpoints has 'Empty' material and the other has some non-'Empty'
	material. The latter is called the solid endpoint of the edge.
*/
struct UniformGridEdge {
	UniformGridEdge () noexcept {}
	/** \brief Main constructor

		\param[in] lesserX,lesserY,lesserZ Local coordinates of the lesser endpoint
		\param[in] gradient Gradient of the surface function in surface-crossing point
		\param[in] offset Surface-crossing point offset from lesser endpoint (in local coordinates)
		\param[in] axis Axis of this edge in GLM order (X=0, Y=1, Z=2)
		\param[in] isLesserEndpointSolid Self-descriptive
		\param[in] solidMaterial Material of the solid endpoint
	*/
	UniformGridEdge (int32_t lesserX, int32_t lesserY, int32_t lesserZ,
	                 const glm::dvec3 &gradient, double offset,
	                 int axis, bool isLesserEndpointSolid, Material solidMaterial) noexcept;
	/// Returns surface normal in the surface-crossing point on this edge
	glm::vec3 surfaceNormal () const noexcept;
	/// Returns local coordinates of the surface-crossing point on this edge
	glm::vec3 surfacePoint () const noexcept;
	/// Returns the material of the solid endpoint
	Material solidEndpointMaterial () const noexcept { return material; }
	/// Returns local coordinates of the lesser endpoint
	glm::ivec3 lesserEndpoint () const noexcept;
	/// Returns local coordinates of the bigger endpoint
	glm::ivec3 biggerEndpoint () const noexcept;
	/// Returns true if the lesser endpoint is solid, false otherwise
	bool isLesserEndpointSolid () const noexcept { return !solidEndpoint; }

private:
	/** \brief Surface normal in zero-crossing point.
	
		Only X and Z components (and sign of Y) are stored because absolute value of Y
		may be restored using the unit length condition. This saves 4 bytes of size,
		making the grid occupy ~11% less space on average (an empirical estimation).
	*/
	float normalX, normalZ;
	/// Offset from lesser endpoint (in local coordinates)
	float offset;
	/// 0 if Y is positive, 1 if negative
	uint8_t normalYSign : 1;
	/// 0 if solid endpoint is the lesser one, 1 otherwise
	uint8_t solidEndpoint : 1;
	/// Edge axis in GLM order (X=0, Y=1, Z=2)
	uint8_t axis : 2;
	/// Material of solid endpoint of the surface
	Material material;
	/// Local coordinates of lesser endpoint
	int16_t lesserX, lesserY, lesserZ;

	friend class UniformGridEdgeStorage;
};

/** \brief Compressed storage of uniform grid edges
*/
class UniformGridEdgeStorage {
public:
	using iterator = std::vector<UniformGridEdge>::iterator;
	using const_iterator = std::vector<UniformGridEdge>::const_iterator;
	using size_type = std::vector<UniformGridEdge>::size_type;

	template<typename... Args>
	void addEdge (Args &&... args) { m_edges.emplace_back (std::forward<Args> (args)...); }
	/** \brief Sorts stored edges by lesser endpoints (in YXZ order)
	
		Using \ref findEdge is possible only when edges are sorted. Instead of calling this
		function you may enforce adding edges in given order (if possible).
	*/
	void sortEdges () noexcept;
	void clear () noexcept { m_edges.clear (); }
	iterator begin () noexcept { return m_edges.begin (); }
	iterator end () noexcept { return m_edges.end (); }
	/** \brief Finds an edge with given lesser endpoint coordinates

		\param[in] x,y,z Local coordinates of the lesser endpoint
		\return Iterator to the found edge or \ref end in case no edge was found
		\attention This function runs binary search. Make sure edges are sorted before calling
	*/
	iterator findEdge (int32_t x, int32_t y, int32_t z) noexcept;
	/// Returns the number of edges in storage
	size_type size () const noexcept { return m_edges.size (); }
	const_iterator begin () const noexcept { return m_edges.begin (); }
	const_iterator end () const noexcept { return m_edges.end (); }
	const_iterator cbegin () const noexcept { return m_edges.cbegin (); }
	const_iterator cend () const noexcept { return m_edges.cend (); }
	/// \copydoc findEdge
	const_iterator findEdge (int32_t x, int32_t y, int32_t z) const noexcept;
private:
	/// Wrapped container
	std::vector<UniformGridEdge> m_edges;
	/// 'Less' comparator for edges, orders them as (Y, X, Z) tuples
	static bool edgeLess (const UniformGridEdge &a, const UniformGridEdge &b) noexcept;
};

}
