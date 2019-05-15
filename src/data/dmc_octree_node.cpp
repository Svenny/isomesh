/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/dmc_octree_node.hpp>

#include <stdexcept>

namespace isomesh
{

DMC_OctreeNode::DMC_OctreeNode () noexcept : children (nullptr)
{}

DMC_OctreeNode::~DMC_OctreeNode () noexcept {
	collapse ();
}

void DMC_OctreeNode::subdivide () {
	if (children)
		throw std::logic_error ("Octree node is already subdivided");
	children = new DMC_OctreeNode[8];
}

void DMC_OctreeNode::collapse () noexcept {
	delete[] children;
	children = nullptr;
}

}
