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
	DMC_OctreeNode () noexcept;
	~DMC_OctreeNode () noexcept;
	
	DMC_OctreeNode *children[8];
	glm::vec4 dualVertex;
};

}
