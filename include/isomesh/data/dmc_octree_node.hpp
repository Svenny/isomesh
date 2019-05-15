/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Node of DMC Octree
*/
#pragma once

#include "../common.hpp"

namespace isomesh
{

struct DMC_OctreeNode {
	DMC_OctreeNode () noexcept;
	~DMC_OctreeNode () noexcept;

	void subdivide ();
	void collapse () noexcept;
	bool isSubdivided () const noexcept { return children != nullptr; }

	DMC_OctreeNode *operator[] (int num) const noexcept { return children + num; }

	glm::vec4 dualVertex;
	glm::vec3 normal;
	Material material;

private:
	DMC_OctreeNode *children;
};

}
