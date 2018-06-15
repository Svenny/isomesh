/*
  Isomesh - a collection of isosurface extraction algorithms
  
  Copyright (c) 2018 Pavel Asyutchenko
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
/** \file
  \brief Definiton of output mesh format */
#pragma once

#include <cassert>
#include <cstdint>

#include <limits>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

namespace isomesh::data
{

struct MeshVertex
{
    MeshVertex (const glm::vec3 &pos, const glm::vec3 &norm, float mat) noexcept :
        position (pos), normal (norm), material (mat) {}
    glm::vec3 position;
    glm::vec3 normal;
    float material;
};

/** \brief Simple triangle mesh storage

  This is essentially a thin wrapper around a pair of
  std::vectors (vertices and indices), suitable to
  send its contents directly to GPU.
*/
class Mesh
{
public:
    Mesh () = default;
    
    explicit Mesh (size_t reserve_vertices, size_t reserve_indices = 0) {
        m_vertices.reserve (reserve_vertices);
        m_indices.reserve (reserve_indices);
    }
    
    Mesh (Mesh &&rv) = default;
    Mesh &operator = (Mesh &&rv) = default;

    Mesh (const Mesh &) = delete;
    Mesh &operator = (const Mesh &) = delete;

    /** \brief Insert a new vertex in the mesh
      \param[in] args Arguments passed to vertex constructor
      \return Index of newly created vertex
    */
    template<typename... Args> uint32_t addVertex (Args&&... args) {
        // Construct vertex at the end of array
        m_vertices.emplace_back (std::forward<Args> (args)...);
        // Index of newly created vertex
        size_t index = --m_vertices.size ();
        // This shouldn't happen in practice (> 2^32 vertices)
        assert (index <= std::numeric_limits<uint32_t>::max ());
        return uint32_t (index);
    }

    /** \brief Insert a new triangle in the mesh
      \param[in] i1 Index of the first vertex
      \param[in] i2 Index of the second vertex
      \paran[in] i3 Index of third vertex
    */
    void addTriangle (uint32_t i1, uint32_t i2, uint32_t i3) {
        m_indices.emplace_back (i1);
        m_indices.emplace_back (i2);
        m_indices.emplace_back (i3);
    }

    MeshVertex &operator[] (size_t index) { return m_vertices[index]; }
    const MeshVertex &operator[] (size_t index) const { return m_vertices[index]; }

    MeshVertex *vertices () noexcept { return m_vertices.data (); }
    const MeshVertex *vertices () const noexcept { return m_vertices.data (); }

    uint32_t *indices () noexcept { return m_indices.data (); }
    const uint32_t *indices () const noexcept { return m_indices.data (); }

    size_t verticesCount () const noexcept { return m_vertices.size (); }
    size_t indicesCount () const noexcept { return m_indices.size (); }

private:
    std::vector<MeshVertex> m_vertices; ///< Vertices array
    std::vector<uint32_t> m_indices; ///< Indices array
};

}
