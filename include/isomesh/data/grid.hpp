/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
  \brief Defines uniform grid data storage for algorithms
*/
#pragma once

#include <iterator>
#include <memory>

#include "../common.hpp"

namespace isomesh
{

struct UniformGridEdge {
   /// Surface normal in zero-crossing point
   glm::vec3 normal;
   /// Normalized offset from lesser-coordinate endpoint (range 0..1)
   float offset;
};

// Immutable edge storage
class UniformGridEdgeStorage {
private:
	struct Edge {
		//Edge (int8_t xx, int8_t yy, int8_t zz, const UniformGridEdge &d) noexcept :
		//	x (xx), y (yy), z (zz), data (d) {}
		int8_t x, y, z;
		UniformGridEdge data;
		
		bool operator < (const Edge &b) const noexcept;
	};
public:
	struct const_iterator {
	public:
		using difference_type = ptrdiff_t;
		using value_type = UniformGridEdge;
		using pointer = const UniformGridEdge *;
		using reference = const UniformGridEdge &;
		using iterator_category = std::random_access_iterator_tag;
		
		const_iterator () = default;
		const_iterator (std::vector<Edge>::const_iterator iter) noexcept : m_iter (iter) {}
		
		const_iterator &operator ++ () noexcept { m_iter++; return *this; }
		const_iterator operator ++ (int) noexcept { auto res = *this; ++(*this); return res; }
		const_iterator &operator -- () noexcept { m_iter--; return *this; }
		const_iterator operator -- (int) noexcept { auto res = *this; --(*this); return res; }
		
		difference_type operator - (const_iterator b) const noexcept { return m_iter - b.m_iter; }
		
		const_iterator operator + (difference_type d) const noexcept { return const_iterator (m_iter + d); }
		const_iterator operator - (difference_type d) const noexcept { return const_iterator (m_iter - d); }
		const_iterator &operator += (difference_type d) noexcept { m_iter += d; return *this; }
		const_iterator &operator -= (difference_type d) noexcept { m_iter -= d; return *this; }
		
		bool operator < (const_iterator b) const noexcept { return m_iter < b.m_iter; }
		bool operator <= (const_iterator b) const noexcept { return m_iter <= b.m_iter; }
		bool operator > (const_iterator b) const noexcept { return m_iter > b.m_iter; }
		bool operator >= (const_iterator b) const noexcept { return m_iter >= b.m_iter; }
		bool operator == (const_iterator b) const noexcept { return m_iter == b.m_iter; }
		bool operator != (const_iterator b) const noexcept { return m_iter != b.m_iter; }
		
		reference operator [] (difference_type d) const { return m_iter[d].data; }
		reference operator * () const { return m_iter->data; }
		pointer operator -> () const { return &m_iter->data; }
		
		glm::ivec3 localCoords () const { return glm::ivec3 (m_iter->x, m_iter->y, m_iter->z); }
	private:
		std::vector<Edge>::const_iterator m_iter;
	};

	size_t size () const noexcept { return m_edges.size (); }
	const_iterator cbegin () const noexcept { return const_iterator (m_edges.begin ()); }
	const_iterator cend () const noexcept { return const_iterator (m_edges.end ()); }
private:
	std::vector<Edge> m_edges;
	void clear () noexcept { m_edges.clear (); }
	void sort () noexcept;
	void addEdge (int32_t x, int32_t y, int32_t z, const glm::dvec3 &normal, double offset);
	
	friend class UniformGrid;
};

// +/- overloads to complete the definition of const_iterator
inline auto operator + (UniformGridEdgeStorage::const_iterator::difference_type d,
	UniformGridEdgeStorage::const_iterator iter) noexcept { return iter + d; }
inline auto operator - (UniformGridEdgeStorage::const_iterator::difference_type d,
	UniformGridEdgeStorage::const_iterator iter) noexcept { return iter - d; }

class ZeroFinder;
class MaterialSelector;

// YXZ traversal order (to match Voxen's layout)
// Local coordinates are [-size/2; size/2]
// Global coordinates define point position in the world
// Size must be a power of two (size >= 2 and size <= 128), this is needed
// to make adaptive compression algorithms (like octree) work on grids
/* Size can't be made more than 128, it's a technical limitation. I use 24-bit vertex
 indexing in some places, so the grid shouldn't have more than 2^24 vertices. As the
 grid has (size + 1)^3 vertices, the maximal size meeting this limit is 255. Therefore
 the largest suitable power of two is 128. Compressed edge storage uses 8-bit indexing
 which has a range of [-128;127], which doesn't fit 256 (which is [-128;128]). */
class UniformGrid {
public:
	explicit UniformGrid (uint32_t size, const glm::dvec3 &globalPos = glm::dvec3 (0.0), double scale = 1.0);
	// Fills grid using given surface function & material selection method
	void fill (const SurfaceFunction &f, const ZeroFinder &solver, const MaterialSelector &material);
	// Converts local-coordinates point to YXZ-order traversal (raw) index
	uint32_t pointToRawIndex (int32_t x, int32_t y, int32_t z) const;
	uint32_t pointToRawIndex (const glm::ivec3 &v) const;
	// Converts YXZ-order traversal (raw) index to local-coordinates point
	glm::ivec3 rawIndexToPoint (uint32_t idx) const noexcept;
	// Local-coordinates indexing
	Material at (int32_t x, int32_t y, int32_t z) const;
	Material operator [] (const glm::ivec3 &v) const;
	// Raw indexing
	const Material *data () const noexcept { return m_mat.get (); }
	// Iterators to edge storages
	// TODO: add search by key (edge position)
	auto xEdgesBegin () const noexcept { return m_edgeX.cbegin (); }
	auto xEdgesEnd () const noexcept { return m_edgeX.cend (); }
	auto yEdgesBegin () const noexcept { return m_edgeY.cbegin (); }
	auto yEdgesEnd () const noexcept { return m_edgeY.cend (); }
	auto zEdgesBegin () const noexcept { return m_edgeZ.cbegin (); }
	auto zEdgesEnd () const noexcept { return m_edgeZ.cend (); }
	// Properties
	uint32_t dataSize () const noexcept { return (m_size + 1) * (m_size + 1) * (m_size + 1); }
	uint32_t gridSize () const noexcept { return m_size; }
	int32_t maxCoord () const noexcept { return m_halfSize; }
	int32_t minCoord () const noexcept { return -m_halfSize; }
	glm::dvec3 globalPosition () const noexcept { return m_globalPos; }
	double scale () const noexcept { return m_scale; }
	// Coordinate system conversions
	glm::dvec3 localToGlobal (const glm::dvec3 &L) const noexcept { return L * m_scale + m_globalPos; }
	glm::dvec3 globalToLocal (const glm::dvec3 &G) const noexcept { return (G - m_globalPos) / m_scale; }
private:
	const uint32_t m_size;
	const int32_t m_halfSize;

	std::unique_ptr<Material[]> m_mat;
	
	UniformGridEdgeStorage m_edgeX, m_edgeY, m_edgeZ;

	glm::dvec3 m_globalPos;
	double m_scale;
	
	// Checks whether point (x,y,z) lies in valid local coordinates
	void checkPoint (int32_t x, int32_t y, int32_t z) const;
};

}
