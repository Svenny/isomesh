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

glm::dvec3 SDF::grad (const glm::dvec3 &p) const noexcept
{
    constexpr double Step = 1.0 / 1024.0;
    double d[3];
    for (int i = 0; i < 2; i++)
    {
        glm::dvec3 shift; shift[i] = Step;
        d[i] = value (p + shift) - value (p - shift);
        d[i] /= 2.0 * Step;
    }
    return glm::dvec3 (d[0], d[1], d[2]);
}

}
