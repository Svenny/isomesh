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
  \brief Contains examples of some simple signed distance functions */
#pragma once

#include "sdf.hpp"

namespace isomesh::sdf
{

/** \brief Signed distance function of a sphere

  
*/
class Sphere : public ISignedDistance
{
public:
    /** Basic constructor
      \param[in] radius Radius of the sphere
      \param[in] center Center of the sphere, defaults to origin (0; 0; 0)
    */
    explicit Sphere (double radius, const glm::dvec3 &center = glm::dvec3 (0.0)) noexcept :
        m_radius (radius), m_center (center) {}
    virtual ~Sphere () = default;

    virtual double calcValue (const glm::dvec3 &p) const noexcept override;
    virtual glm::dvec3 calcGradient (const glm::dvec3 &p) const noexcept override;
    virtual hermite_data calcHermiteData (const glm::dvec3 &p) const noexcept override;
protected:
    double m_radius; ///< Radius of the sphere
    glm::dvec3 m_center; ///< Center of the sphere
};

}
