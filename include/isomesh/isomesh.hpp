/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Master header file for Isomesh library
*/
/** \mainpage Library overview
  This library provides implementations of several isosurface extraction algorithms. It is designed
  to be easily extendable and customizable by providing your own data sources, solvers and other entities
  used by algorithms. There is an <a href="https://github.com/Svenny/isomesh-viewer">isosurface viewer</a>
  which demonstrates the usage of this library.
  \author Pavel Asyutchenko
  \version 0.1
  \date 2018
  \copyright MIT License
*/
#pragma once

#include "common.hpp"

#include "data/grid.hpp"
#include "data/mesh.hpp"

#include "qef/qef_solver_3d.hpp"

#include "util/material_filter.hpp"
#include "util/material_selector.hpp"
#include "util/zero_finder.hpp"

#include "algo/marching_cubes.hpp"
#include "algo/uniform_dual_contouring.hpp"
