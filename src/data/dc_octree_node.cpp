/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/dc_octree_node.hpp>

#include <stdexcept>

namespace isomesh
{

DC_OctreeNode::DC_OctreeNode () noexcept : is_leaf (true) {
	for (int i = 0; i < 8; i++)
		children[i] = nullptr;
}

DC_OctreeNode::~DC_OctreeNode () noexcept {
	if (!is_leaf)
		collapse ();
}

void DC_OctreeNode::subdivide () {
	if (!is_leaf)
		throw std::logic_error ("Octree node is already subdivided");
	is_leaf = false;
	for (int i = 0; i < 8; i++) {
		children[i] = new DC_OctreeNode;
		children[i]->m_depth = m_depth + 1;
	}
}

void DC_OctreeNode::collapse () {
	if (is_leaf)
		throw std::logic_error ("Octree node is already collapsed");
	is_leaf = true;
	for (int i = 0; i < 8; i++) {
		delete children[i];
		children[i] = nullptr;
	}
}

bool DC_OctreeNode::isHomogenous () const noexcept {
	if (!is_leaf)
		return false;
	bool all_air = true;
	bool all_solid = true;
	for (int i = 0; i < 8; i++) {
		if (leaf_data.corners[i] == Material::Empty)
			all_solid = false;
		else all_air = false;
	}
	return all_air || all_solid;
}

}
