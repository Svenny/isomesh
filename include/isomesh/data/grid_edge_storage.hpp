/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Compressed storage for uniform grid edges
*/
#pragma once

#include "../common.hpp"

#include <iterator>

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
	template<bool isConst>
	struct base_iterator {
	public:
		using difference_type = ptrdiff_t;
		using value_type = UniformGridEdge;
		using pointer = std::conditional_t<isConst, const UniformGridEdge *, UniformGridEdge *>;
		using reference = std::conditional_t<isConst, const UniformGridEdge &, UniformGridEdge &>;
		using iterator_category = std::random_access_iterator_tag;
		using wrapped_type = std::conditional_t<isConst,
			std::vector<Edge>::const_iterator, std::vector<Edge>::iterator>;

		base_iterator () noexcept {};
		base_iterator (const wrapped_type &iter) noexcept : m_iter (iter) {}

		base_iterator &operator ++ () noexcept { m_iter++; return *this; }
		base_iterator operator ++ (int) noexcept { auto res = *this; ++(*this); return res; }
		base_iterator &operator -- () noexcept { m_iter--; return *this; }
		base_iterator operator -- (int) noexcept { auto res = *this; --(*this); return res; }

		difference_type operator - (const base_iterator &b) const noexcept { return m_iter - b.m_iter; }

		base_iterator operator + (difference_type d) const noexcept { return base_iterator (m_iter + d); }
		base_iterator operator - (difference_type d) const noexcept { return base_iterator (m_iter - d); }
		base_iterator &operator += (difference_type d) noexcept { m_iter += d; return *this; }
		base_iterator &operator -= (difference_type d) noexcept { m_iter -= d; return *this; }

		bool operator < (const base_iterator &b) const noexcept { return m_iter < b.m_iter; }
		bool operator <= (const base_iterator &b) const noexcept { return m_iter <= b.m_iter; }
		bool operator > (const base_iterator &b) const noexcept { return m_iter > b.m_iter; }
		bool operator >= (const base_iterator &b) const noexcept { return m_iter >= b.m_iter; }
		bool operator == (const base_iterator &b) const noexcept { return m_iter == b.m_iter; }
		bool operator != (const base_iterator &b) const noexcept { return m_iter != b.m_iter; }

		reference operator [] (difference_type d) const { return m_iter[d].data; }
		reference operator * () const { return m_iter->data; }
		pointer operator -> () const { return &m_iter->data; }

		glm::ivec3 localCoords () const { return glm::ivec3 (m_iter->x, m_iter->y, m_iter->z); }
	private:
		wrapped_type m_iter;
	};

	using iterator = base_iterator<false>;
	using const_iterator = base_iterator<true>;

	size_t size () const noexcept { return m_edges.size (); }
	iterator begin () noexcept { return iterator (m_edges.begin ()); }
	iterator end () noexcept { return iterator (m_edges.end ()); }
	const_iterator begin () const noexcept { return const_iterator (m_edges.begin ()); }
	const_iterator end () const noexcept { return const_iterator (m_edges.end ()); }
	const_iterator cbegin () const noexcept { return const_iterator (m_edges.cbegin ()); }
	const_iterator cend () const noexcept { return const_iterator (m_edges.cend ()); }
private:
	std::vector<Edge> m_edges;
	void clear () noexcept { m_edges.clear (); }
	void sort () noexcept;
	void addEdge (int32_t x, int32_t y, int32_t z, const glm::dvec3 &normal, double offset);

	friend class UniformGrid;
};

// (d + iter) operators to complete iterator definition
inline auto operator + (UniformGridEdgeStorage::iterator::difference_type d,
								const UniformGridEdgeStorage::iterator &iter) noexcept {
	return iter + d;
}
inline auto operator + (UniformGridEdgeStorage::const_iterator::difference_type d,
                        const UniformGridEdgeStorage::const_iterator &iter) noexcept {
	return iter + d;
}

}
