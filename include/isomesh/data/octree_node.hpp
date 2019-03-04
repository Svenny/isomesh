/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Octree node
*/
#pragma once

#include "../common.hpp"

namespace isomesh
{

struct DMC_OctreeNode {
	DMC_OctreeNode (glm::ivec3 min_corner, int32_t size) noexcept;
	~DMC_OctreeNode () noexcept;

	void subdivide ();
	void collapse ();
	bool isSubdivided () const noexcept { return hasChildren; }

	DMC_OctreeNode *operator[] (int num) const noexcept { return children[num]; }

	glm::vec4 dualVertex;
	glm::vec3 normal;
	Material material;
	const glm::ivec3 minCorner;
	const int32_t size;
private:
	bool hasChildren;
	DMC_OctreeNode *children[8];
};

}
