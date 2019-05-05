/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Look-up tables for use in algorithms
*/
#pragma once

#include "../common.hpp"

namespace isomesh
{

extern const uint8_t kCellEdgeEndpoint[12][2];
extern const uint8_t kCellEdgeDirection[12];

extern const glm::ivec3 kCellCornerOffset[8];

// Octree traversal tables

extern const uint8_t kEdgeProcCallTable[3][2][4];
extern const uint8_t kEdgeProcChildTable[3][8][2];
extern const uint8_t kEdgeProcSharedEdge[3][4];

extern const uint8_t kFaceProcCallTable[3][4][2];
extern const uint8_t kFaceProcChildTable[3][8][2];

// Marching Cubes tables

extern const uint16_t kMcVertexMaskToEdgeMask[256];
extern const int8_t kMcTriangleTable[256][13];

// Manifold Dual Contouring tables

extern const int8_t kMdcEdgeSetIndex[256][12];

}
