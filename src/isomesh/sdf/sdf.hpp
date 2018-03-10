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
  \brief Definition file for Signed Distance Function */
#pragma once

#include <utility>

#include <glm/glm.hpp>

namespace isomesh::sdf
{

/** \brief Abstract base class for Signed Distance Function (SDF)
  
*/
class ISignedDistance
{
public:
    virtual ~ISignedDistance () = default;

    /** \brief Calculates value of SDF in given point \c p
      \param[in] p Point where to compute value
      \return Value of the function in point \c p
      \attention Properly defined signed distance function is
      continuous over the whole \f$ R^3 \f$. This property is
      crucial to produce a valid isosurface
    */
    virtual double calcValue (const glm::dvec3 &p) const noexcept = 0;

    /** \brief Calculates gradient of SDF in given point \c p
      \param[in] p Point where to compute gradient
      \return Gradient of the function in point \c p
      \attention Function should not return NaNs or
      infinities. Prefer to return arbitrary unit vector instead
      \attention Properly defined signed distance function always
      has gradient of unit length (where the gradient is defined).
      Normalize it if this property does not hold in your function
    */
    virtual glm::dvec3 calcGradient (const glm::dvec3 &p) const noexcept;

    /** \brief A shorthand for Hermite data pair
      
      For a given function \f$ f(x,y,z) \f$ its
      Hermite data at point \f$ p \f$ is a pair
      \f$ (f(p); \nabla f(p)) \f$, where
      \f$ \nabla f(p) = \{
      \frac{\partial f(p)}{\partial x};
      \frac{\partial f(p)}{\partial y};
      \frac{\partial f(p)}{\partial z}
      \} \f$
    */
    using hermite_data = std::pair<double, glm::dvec3>;

    /** \brief Calculates Hermite data of SDF in given point \c p

      Hermite data consists of value and gradient in given point, so
      default implementation simply calls the respective functions
      and packs their result into a pair. Override this function
      if it is possible to calculate value and gradient together
      faster than separately (i.e. they have some common subexpressions)
      \param[in] p Point where to compute these values
      \return Hermite data pair in point \c p
      \see calcValue, calcGradient, hermite_data
    */
    virtual hermite_data calcHermiteData (const glm::dvec3 &p) const noexcept;
};

}
