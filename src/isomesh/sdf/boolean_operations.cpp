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
#include "pch.hpp"

#include "boolean_operations.hpp"

namespace isomesh::sdf
{

// ---- Union ----

Union::Union (const ISignedDistance &a, const ISignedDistance &b) noexcept :
    m_funA (a), m_funB (b) {}

double Union::calcValue (const glm::dvec3 &p) const noexcept
{
    return glm::min (m_funA.calcValue (p), m_funB.calcValue (p));
}

glm::dvec3 Union::calcGradient (const glm::dvec3 &p) const noexcept
{
    hermite_data dataA = m_funA.calcHermiteData (p);
    hermite_data dataB = m_funB.calcHermiteData (p);
    // When a < b, gradient is the same as a's
    // When a > b, gradient is the same as b's
    // Strictly speaking, in case of a == b gradient is undefined,
    // but we still need to return something in this case
    if (dataA.first <= dataB.first)
        return dataA.second;
    else
        return dataB.second;
}

hermite_data Union::calcHermiteData (const glm::dvec3 &p) const noexcept
{
    hermite_data dataA = m_funA.calcHermiteData (p);
    hermite_data dataB = m_funB.calcHermiteData (p);
    if (dataA.first <= dataB.first)
        return dataA;
    else
        return dataB;
}

// ---- Difference ----

Difference::Difference (const ISignedDistance &a, const ISignedDistance &b) noexcept :
    m_funA (a), m_funB (b) {}

double Difference::calcValue (const glm::dvec3 &p) const noexcept
{
    return glm::max (-m_funA.calcValue (p), m_funB.calcValue (p));
}

glm::dvec3 Difference::calcGradient (const glm::dvec3 &p) const noexcept
{
    hermite_data dataA = m_funA.calcHermiteData (p);
    // Negating value of function also negates its gradient
    dataA.first = -dataA.first;
    dataA.second = -dataA.second;
    hermite_data dataB = m_funB.calcHermiteData (p);
    // When -a > b, gradient is the same as -a's
    // When -a < b, gradient is the same as b's
    // Strictly speaking, in case of -a == b gradient is undefined,
    // but we still need to return something in this case
    if (dataA.first >= dataB.first)
        return dataA.second;
    // min (a, b) = b, return gradient of b
    else
        return dataB.second;
}

hermite_data Difference::calcHermiteData (const glm::dvec3 &p) const noexcept
{
    hermite_data dataA = m_funA.calcHermiteData (p);
    dataA.first = -dataA.first;
    dataA.second = -dataA.second;
    hermite_data dataB = m_funB.calcHermiteData (p);
    if (dataA.first >= dataB.first)
        return dataA;
    else
        return dataB;
}

// ---- Intersection ----

Intersection::Intersection (const ISignedDistance &a, const ISignedDistance &b) noexcept :
    m_funA (a), m_funB (b) {}

double Intersection::calcValue (const glm::dvec3 &p) const noexcept
{
    return glm::max (m_funA.calcValue (p), m_funB.calcValue (p));
}

glm::dvec3 Intersection::calcGradient (const glm::dvec3 &p) const noexcept
{
    hermite_data dataA = m_funA.calcHermiteData (p);
    hermite_data dataB = m_funB.calcHermiteData (p);
    // When a > b, gradient is the same as a's
    // When a < b, gradient is the same as b's
    // Strictly speaking, in case of a == b gradient is undefined,
    // but we still need to return something in this case
    if (dataA.first >= dataB.first)
        return dataA.second;
    else
        return dataB.second;
}

hermite_data Intersection::calcHermiteData (const glm::dvec3 &p) const noexcept
{
    hermite_data dataA = m_funA.calcHermiteData (p);
    hermite_data dataB = m_funB.calcHermiteData (p);
    if (dataA.first >= dataB.first)
        return dataA;
    else
        return dataB;
}

}
