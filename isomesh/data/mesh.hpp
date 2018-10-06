/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
  \brief Storage for algorithms output
*/
#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "material.hpp"

namespace isomesh
{

namespace detail
{

struct alignas (4) IsomeshVertex {
   IsomeshVertex (const glm::vec3 &pos, const glm::vec3 &norm, VoxelMaterial mat) noexcept :
      position (pos), normal (norm), material (float (mat)) {}
   glm::vec3 position;
   glm::vec3 normal;
   /// Stored as float because GPU uses floating-point indexing
   float material;
};

}

/** \brief Simple triangle mesh storage

  This is essentially a thin wrapper around a pair of
  std::vectors (vertices and indices), suitable to
  send its contents directly to GPU.
*/
struct Isomesh {
   using Vertex = detail::IsomeshVertex;

   Isomesh () = default;

   explicit Isomesh (size_t reserve_vertices = 0, size_t reserve_indices = 0) {
      m_vertices.reserve (reserve_vertices);
      m_indices.reserve (reserve_indices);
   }

   Isomesh (Isomesh &&) = default;
   Isomesh &operator = (Isomesh &&) = default;

   Isomesh (const Isomesh &) = delete;
   Isomesh &operator = (const Isomesh &) = delete;

   /** \brief Insert a new vertex into the mesh
     \param[in] args Arguments passed to vertex constructor (position, normal and material)
     \return Index of newly created vertex
   */
   template<typename... Args> uint32_t addVertex (Args&&... args) {
      // Construct vertex at the end of array
      m_vertices.emplace_back (std::forward<Args> (args)...);
      // Index of newly created vertex
      size_t index = m_vertices.size () - 1;
      // This shouldn't happen in practice (> 2^32 vertices)
      assert (index <= std::numeric_limits<uint32_t>::max ());
      return uint32_t (index);
   }

   /** \brief Insert a new triangle into the mesh
     \param[in] i1 Index of the first vertex
     \param[in] i2 Index of the second vertex
     \paran[in] i3 Index of the third vertex
   */
   void addTriangle (uint32_t i1, uint32_t i2, uint32_t i3) {
      m_indices.emplace_back (i1);
      m_indices.emplace_back (i2);
      m_indices.emplace_back (i3);
   }

   Vertex &operator[] (size_t index) { return m_vertices[index]; }
   const Vertex &operator[] (size_t index) const { return m_vertices[index]; }

   Vertex *vertices () noexcept { return m_vertices.data (); }
   const Vertex *vertices () const noexcept { return m_vertices.data (); }

   uint32_t *indices () noexcept { return m_indices.data (); }
   const uint32_t *indices () const noexcept { return m_indices.data (); }

   size_t verticesCount () const noexcept { return m_vertices.size (); }
   size_t indicesCount () const noexcept { return m_indices.size (); }

private:
   std::vector<Vertex> m_vertices; ///< Vertices array
   std::vector<uint32_t> m_indices; ///< Indices array
};

}
