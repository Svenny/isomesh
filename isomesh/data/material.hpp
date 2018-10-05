/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
  \brief Voxel material constants
*/
#pragma once

#include <cstdint>

namespace isomesh
{

enum class VoxelMaterial : uint8_t {
   // No material (i.e. air)
   Empty = 0,
   Stone,
   Dirt,
   
   // Not a material
   Count
};

}
