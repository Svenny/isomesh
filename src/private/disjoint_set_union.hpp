/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include <isomesh/common.hpp>

#include <vector>

namespace isomesh
{

template<typename T>
class DisjointSetUnion {
public:
	explicit DisjointSetUnion (T size = 0) : m_parent (size), m_size (size) {
		for (T i = 0; i < size; i++) {
			m_parent[i] = i;
			m_size[i] = 1;
		}
	}

	T size () const noexcept { return T (m_parent.size ()); }

	void expand (T new_size) {
		T old_size = size ();
		if (new_size <= old_size)
			return;
		// Resizes are preceeded with reserves to keep exception guarantee (if reserve throws
		// the length of vectors does not change. resize cannot throw after reserving)
		m_parent.reserve (new_size);
		m_size.reserve (new_size);
		m_parent.resize (new_size);
		m_size.resize (new_size);
		for (T i = old_size; i < new_size; i++) {
			m_parent[i] = i;
			m_size[i] = 1;
		}
	}

	T addSet () {
		expand (size () + 1);
		return size () - 1;
	}
	
	T getSetLeader (T item) {
		assert (item < size ());
		if (m_parent[item] == item)
			return item;
		return m_parent[item] = getSetLeader (m_parent[item]);
	}

	void mergeSets (T a, T b) {
		a = getSetLeader (a);
		b = getSetLeader (b);
		if (a == b)
			return;
		if (m_size[a] < m_size[b]) {
			m_parent[a] = b;
			m_size[b] += m_size[a];
		}
		else {
			m_parent[b] = a;
			m_size[a] += m_size[b];
		}
	}
	
	bool areInSameSet (T a, T b) { return getSetLeader (a) == getSetLeader (b); }
private:
	std::vector<T> m_parent;
	std::vector<T> m_size;
};

}
