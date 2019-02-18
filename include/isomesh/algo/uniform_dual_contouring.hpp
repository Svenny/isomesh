/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Dual contouring algorithm, implemented over uniform grids
*/
#pragma once

#include "../util/material_filter.hpp"
#include "../data/grid.hpp"
#include "../data/mesh.hpp"

namespace isomesh
{

/** \brief Dual contouring isosurface algorithm

	TBD
*/
Mesh dualContouring (const UniformGrid &G, const MaterialFilter &filter);

}
