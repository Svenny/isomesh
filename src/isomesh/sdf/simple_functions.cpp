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

// For details on these distance functions see
// http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm

// ---- Sphere ----

double Sphere::calcValue (const glm::dvec3 &p) const noexcept
{
    // For a sphere of radius R its SDF
    // is defined as f(x,y,z) = sqrt (x^2 + y^2 + z^2) - R
    return glm::length (p) - m_radius;
}

glm::dvec3 Sphere::calcGradient (const glm::dvec3 &p) const noexcept
{
    constexpr double eps = std::numeric_limits<double>::min ();
    // Gradient of sphere SDF is simply its normalized offset vector
    // Add very small value to avoid division by zero
    double len = glm::length (p) + eps;
    return p / len;
}

std::pair<double, glm::dvec3> Sphere::calcHermiteData (const glm::dvec3 &p) const noexcept
{
    // See calcValue and calcGradient
    constexpr double eps = std::numeric_limits<double>::min ();
    double len = glm::length (p);
    return { len - m_radius, p / (len + eps) };
}

// ---- Cube ----

double Cube::calcValue (const glm::dvec3 &p) const noexcept
{
    // See the link above, Box-signed-exact
    glm::dvec3 d = glm::abs (p) - m_halfside;
    double maxd = glm::max (d.x, glm::max (d.y, d.z));
    return glm::min (maxd, 0.0) + glm::length (glm::max (d, 0.0));
}

}
