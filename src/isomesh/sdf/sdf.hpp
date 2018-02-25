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
#pragma once

#include <utility>

#include <glm/glm.hpp>

namespace isomesh::sdf
{

/* Signed Distance Function (SDF) abstract class.
*/
class SDF
{
public:
    virtual ~SDF () = default;
    /* Computes value of SDF in given point p.
    */
    virtual double value (const glm::dvec3 &p) const noexcept = 0;
    /* Computes gradient of SDF in given point p.
    */
    virtual glm::dvec3 grad (const glm::dvec3 &p) const noexcept;
    /* Computes both value and gradient of SDF in given point p.
    */
    virtual std::pair<double, glm::dvec3> value_and_grad (const glm::dvec3 &p) const noexcept { return { value (p), grad (p) }; }
};

}
