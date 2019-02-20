/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Dual contouring algorithm, implemented over uniform grids
*/
#pragma once

#include "../data/grid.hpp"
#include "../data/mesh.hpp"
#include "../qef/qef_solver_3d.hpp"
#include "../util/material_filter.hpp"

namespace isomesh
{

/** \brief Dual contouring isosurface algorithm

	TBD
*/
Mesh dualContouring (const UniformGrid &G, const MaterialFilter &filter, QefSolver3D &solver);

}
