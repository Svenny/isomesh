/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include <isomesh/common.hpp>

#include <vector>

namespace isomesh
{

class DisjointSetUnion {
public:
	using item_t = uint32_t;

	DisjointSetUnion (item_t size);

	item_t size () const noexcept { return item_t (m_parent.size ()); }
	void expand (item_t new_size);
	
	item_t getSetLeader (item_t item);
	void mergeSets (item_t a, item_t b);
	
	bool areInSameSet (item_t a, item_t b) { return getSetLeader (a) == getSetLeader (b); }
private:
	std::vector<item_t> m_parent;
	std::vector<item_t> m_size;
};

}
