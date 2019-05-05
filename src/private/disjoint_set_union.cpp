/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include "disjoint_set_union.hpp"

#include <cassert>
#include <stdexcept>

namespace isomesh
{

DisjointSetUnion::DisjointSetUnion (item_t size) : m_parent (size), m_size (size) {
	for (item_t i = 0; i < size; i++) {
		m_parent[i] = i;
		m_size[i] = 1;
	}
}

void DisjointSetUnion::expand (item_t new_size) {
	item_t old_size = size ();
	if (new_size <= old_size)
		return;
	// Resizes are preceeded with reserves to keep exception guarantee (if reserve throws
	// the length of vectors does not change. resize cannot throw after reserving)
	m_parent.reserve (new_size);
	m_size.reserve (new_size);
	m_parent.resize (new_size);
	m_size.resize (new_size);
	for (item_t i = old_size; i < new_size; i++) {
		m_parent[i] = i;
		m_size[i] = 1;
	}
}

DisjointSetUnion::item_t DisjointSetUnion::getSetLeader (item_t item) {
	assert (item < size ());
	if (m_parent[item] == item)
		return item;
	return m_parent[item] = getSetLeader (m_parent[item]);
}

void DisjointSetUnion::mergeSets (item_t a, item_t b) {
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

}
