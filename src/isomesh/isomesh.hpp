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
  \brief Main include file for isomesh library

  This file includes all other needed headers automatically. The only
  dependency is GLM, a header-only vector/matrix math library */
/** \mainpage Library overview
  This library contains implementations of several isosurface extraction
  algorithms. There are also some utilities, such as signed distance functions which
  define isosurfaces and QEF solvers which find the optimal
  locations for vertex placement. The library can be extended
  with new algorithms. A isosurface viewer is present as a separate application.
  \author Pavel Asyutchenko
  \version 0.1
  \date 2018
  \copyright MIT License
*/
#pragma once

#include <glm/glm.hpp>

#include "qef/qef_solver.hpp"

#include "sdf/sdf.hpp"
#include "sdf/simple_functions.hpp"
