/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Marching cubes isosurface contouring algorithm
*/
#pragma once

#include "../data/grid.hpp"
#include "../data/mesh.hpp"

namespace isomesh
{

/** \brief Marching cubes isosurface algorithm

	TBD
*/
Isomesh marchingCubes (const UniformGrid &G);

}

