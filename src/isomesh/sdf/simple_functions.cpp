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

#include <limits>

#include "simple_functions.hpp"

namespace isomesh::sdf
{

double Sphere::calcValue (const glm::dvec3 &p) const noexcept
{
    // For a sphere of radius R with center in (x0; y0; z0) its SDF
    // is defined as f(x,y,z) = sqrt ((x-x0)^2 + (y-y0)^2 + (z-z0)^2) - R
    return glm::distance (p, m_center) - m_radius;
}

glm::dvec3 Sphere::calcGradient (const glm::dvec3 &p) const noexcept
{
    constexpr double eps = std::numeric_limits<double>::min ();
    // Gradient of sphere SDF is simply its normalized offset vector
    // Add very small value to avoid division by zero
    glm::dvec3 offset = p - m_center + eps;
    double len = glm::length (offset);
    return offset / len;
}

std::pair<double, glm::dvec3> Sphere::calcHermiteData (const glm::dvec3 &p) const noexcept
{
    // See calcValue and calcGradient
    constexpr double eps = std::numeric_limits<double>::min ();
    glm::dvec3 offset = p - m_center + eps;
    // Also len = glm::distance (p, m_center)
    double len = glm::length (offset);
    return { len - m_radius, offset / len };
}

}
