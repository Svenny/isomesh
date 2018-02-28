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

#include "sdf.hpp"

namespace isomesh::sdf
{

glm::dvec3 ISignedDistance::calcGradient (const glm::dvec3 &p) const noexcept
{
    constexpr double step = 1.0 / 1024.0;
    glm::dvec3 result;
    // Points for function sampling
    glm::dvec3 p1 (p), p2 (p);
    // Calculate finite difference on each axis
    for (int i = 0; i < 3; i++)
    {
        /* Shift points slightly along i-th axis.
          Due to the limited precision shifting by a small step will not
          change any bit in huge numbers (on the order of 2^42 or more),
          so if p[i] is so large then p1[i] == p2[i] and
          result[i] == 0 regardless of function used. */
        p1[i] += step;
        p2[i] -= step;
        result[i] = calcValue (p1) - calcValue (p2);
        // Divide by interval length, which is 2 * step
        // as we shifted both points by step in opposite sides
        result[i] /= 2.0 * step;
        // Restore original (unshifted) value
        p1[i] = p2[i] = p[i];
    }
    return result;
}

std::pair<double, glm::dvec3> ISignedDistance::calcHermiteData (const glm::dvec3 &p) const noexcept
{
    return { calcValue (p), calcGradient (p) };
}

}
