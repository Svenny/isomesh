/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/octree_node.hpp>

#include <stdexcept>

namespace isomesh
{

DMC_OctreeNode::DMC_OctreeNode (glm::ivec3 min_corner, int32_t size) noexcept :
	minCorner (min_corner), size (size) {
	for (int i = 0; i < 8; i++)
		children[i] = nullptr;
	hasChildren = false;
}

DMC_OctreeNode::~DMC_OctreeNode () noexcept {
	for (int i = 0; i < 8; i++)
		delete children[i];
}

void DMC_OctreeNode::subdivide () {
	if (size == 1)
		throw std::logic_error ("Cannot subdivide octree node of size 1");
	if (hasChildren)
		throw std::logic_error ("Octree node is already subdivided");
	int32_t new_size = size / 2;
	try {
		for (int i = 0; i < 8; i++)
			children[i] = new DMC_OctreeNode (minCorner + kCornerOffset[i] * new_size, new_size);
		hasChildren = true;
	}
	catch (std::bad_alloc &e) {
		// Roll back
		for (int i = 0; i < 8; i++) {
			delete children[i];
			children[i] = nullptr;
		}
		throw e;
	}
}

void DMC_OctreeNode::collapse () noexcept {
	for (int i = 0; i < 8; i++) {
		delete children[i];
		children[i] = nullptr;
	}
	hasChildren = false;
}

}
