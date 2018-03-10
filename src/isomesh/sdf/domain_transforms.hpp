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
  \brief Contains examples of some simple domain transformations of distance functions
  \see http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm - an excellent list of distance functions */
#pragma once

#include "sdf.hpp"

namespace isomesh::sdf
{

/** \brief Translates the origin of SDF domain to point \f$ p \f$.

  Translation is done simply by subtracting \f$ p \f$ from any point. This
  operation does not change the behavior of underlying SDF in any way.
*/
class Translation : public ISignedDistance
{
public:
    /** \brief Basic constructor
      \param[in] fun Function which to translate
      \param[in] p New point of origin
    */
    explicit Translation (const ISignedDistance &fun, const glm::dvec3 &p) noexcept :
        m_fun (fun), m_offset (p) {}

    virtual double calcValue (const glm::dvec3 &p) const noexcept override {
        return m_fun.calcValue (p - m_offset);
    }
    virtual glm::dvec3 calcGradient (const glm::dvec3 &p) const noexcept override {
        m_fun.calcGradient (p - m_offset);
    }
    virtual hermite_data calcHermiteData (const glm::dvec3 &p) const noexcept override {
        return m_fun.calcHermiteData (p - m_offset);
    }
protected:
    const ISignedDistance &m_fun; ///< Base function
    const glm::dvec3 m_offset; ///< Offset vector
};

}
