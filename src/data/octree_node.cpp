/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/octree_node.hpp>

namespace isomesh
{

DMC_OctreeNode::DMC_OctreeNode () noexcept {
	for (int i = 0; i < 8; i++)
		children[i] = nullptr;
}

DMC_OctreeNode::~DMC_OctreeNode () noexcept {
	for (int i = 0; i < 8; i++)
		delete children[i];
}

}
