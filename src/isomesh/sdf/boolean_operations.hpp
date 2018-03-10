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
  \brief Contains examples of some boolean operations on distance functions
  \see http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm - an excellent list of distance functions */
#pragma once

#include "sdf.hpp"

namespace isomesh::sdf
{

/** \brief Union of two SDFS

  Union of two distance functions \f$ a = F1(p) \f$ and \f$ b = F2(p) \f$
  is defined as their minimum \f$ U(a,b,p) = min(a(p),b(p)) \f$. Note that
  when \f$ a(p) \neq b(p) \f$ this function behaves exactly like the minimal
  of base functions, but in case where \f$ a(p)=b(p) \f$ its gradient is undefined.
*/
class Union : public ISignedDistance
{
public:
    /** \brief Basic constructor
      \param[in] a Function \f$ a(p) \f$
      \param[in] b Function \f$ b(p) \f$
    */
    explicit Union (const ISignedDistance &a, const ISignedDistance &b) noexcept;

    virtual double calcValue (const glm::dvec3 &p) const noexcept override;
    virtual glm::dvec3 calcGradient (const glm::dvec3 &p) const noexcept override;
    virtual hermite_data calcHermiteData (const glm::dvec3 &p) const noexcept override;
protected:
    const ISignedDistance &m_funA; ///< Function \f$ a(p) \f$
    const ISignedDistance &m_funB; ///< Function \f$ b(p) \f$
};

/** \brief Difference of two SDFs

  Difference of two distance functions \f$ a = F1(p) \f$ and \f$ b = F2(p) \f$
  is defined as \f$ D(a,b,p) = max(-a(p),b(p)) \f$. Note that, as with Union
  and Intersection this function has undefined gradient where \f$ a(p)=b(p) \f$.
*/
class Difference : public ISignedDistance
{
public:
    /** \brief Basic constructor
      \param[in] a Function \f$ a(p) \f$
      \param[in] b Function \f$ b(p) \f$
    */
    explicit Difference (const ISignedDistance &a, const ISignedDistance &b) noexcept;

    virtual double calcValue (const glm::dvec3 &p) const noexcept override;
    virtual glm::dvec3 calcGradient (const glm::dvec3 &p) const noexcept override;
    virtual hermite_data calcHermiteData (const glm::dvec3 &p) const noexcept override;
protected:
    const ISignedDistance &m_funA; ///< Function \f$ a(p) \f$
    const ISignedDistance &m_funB; ///< Function \f$ b(p) \f$
};

/** \brief Intersection of two SDFS

  Intersection of two distance functions \f$ a = F1(p) \f$ and \f$ b = F2(p) \f$
  is defined as their maximum \f$ I(a,b,p) = max(a(p),b(p)) \f$. Note that
  when \f$ a(p) \neq b(p) \f$ this function behaves exactly like the maximal
  of base functions, but in case where \f$ a(p)=b(p) \f$ its gradient is undefined.
*/
class Intersection : public ISignedDistance
{
public:
    /** \brief Basic constructor
      \param[in] a Function \f$ a(p) \f$
      \param[in] b Function \f$ b(p) \f$
    */
    explicit Intersection (const ISignedDistance &a, const ISignedDistance &b) noexcept;

    virtual double calcValue (const glm::dvec3 &p) const noexcept override;
    virtual glm::dvec3 calcGradient (const glm::dvec3 &p) const noexcept override;
    virtual hermite_data calcHermiteData (const glm::dvec3 &p) const noexcept override;
protected:
    const ISignedDistance &m_funA; ///< Function \f$ a(p) \f$
    const ISignedDistance &m_funB; ///< Function \f$ b(p) \f$
};

}
