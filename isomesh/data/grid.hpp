/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
  \brief Defines uniform grid data storage for algorithms
*/
#pragma once

#include <cstring>
#include <memory>

#include <glm/glm.hpp>

#include "material.hpp"

namespace isomesh
{

namespace detail
{

struct UniformGridEdge {
   /// Surface normal in zero-crossing point
   glm::vec3 normal;
   /// Offset from lesser-coordinate endpoint (range 0..1)
   float offset;
};

template<size_t Size = 32>
struct UniformGridStorage {
   /// Grid voxels. Using YXZ traversal order to match Voxen's layout
   VoxelMaterial grid[Size + 1][Size + 1][Size + 1];
   /// X axis-oriented edges
   UniformGridEdge xEdge[Size][Size][Size];
   /// Y axis-oriented edges
   UniformGridEdge yEdge[Size][Size][Size];
   /// Z axis-oriented edges
   UniformGridEdge zEdge[Size][Size][Size];
};

}

template<size_t Size = 32>
struct UniformGridChunk {
   static_assert (Size > 1, "Size should be at least 2");
   static_assert ((Size & (Size - 1)) == 0, "Size should be a power of 2");

   using Edge = detail::UniformGridEdge;
   using Storage = detail::UniformGridStorage<Size>;

   UniformGridChunk () {
      data = std::make_unique<Storage> ();
   }

   UniformGridChunk (UniformGridChunk &&) = default;
   UniformGridChunk &operator = (UniformGridChunk &&) = default;

   UniformGridChunk (const UniformGridChunk &chunk) {
      data = std::make_unique<Storage> ();
      std::memcpy (data.get (), chunk.data.get (), sizeof (Storage));
   }

   UniformGridChunk &operator = (const UniformGridChunk &chunk) {
      std::memcpy (data.get (), chunk.data.get (), sizeof (Storage));
      return *this;
   }

   VoxelMaterial  matAt (size_t x, size_t y, size_t z) const noexcept { return data->grid[y][x][z]; }
   VoxelMaterial& matAt (size_t x, size_t y, size_t z) noexcept { return data->grid[y][x][z]; }

   Edge  xEdgeAt (size_t x, size_t y, size_t z) const noexcept { return data->xEdge[y][x][z]; }
   Edge& xEdgeAt (size_t x, size_t y, size_t z) noexcept { return data->xEdge[y][x][z]; }

   Edge  yEdgeAt (size_t x, size_t y, size_t z) const noexcept { return data->yEdge[y][x][z]; }
   Edge& yEdgeAt (size_t x, size_t y, size_t z) noexcept { return data->yEdge[y][x][z]; }

   Edge  zEdgeAt (size_t x, size_t y, size_t z) const noexcept { return data->zEdge[y][x][z]; }
   Edge& zEdgeAt (size_t x, size_t y, size_t z) noexcept { return data->zEdge[y][x][z]; }

private:
   std::unique_ptr<Storage> data;
};

}
