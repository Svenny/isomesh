/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
/** @file
 * @brief Convert from 3D model to Mesh
 */
#pragma once

#include <isomesh/data/mesh.hpp>
#include "../../../src/private/ply_data.hpp"

namespace isomesh {
	/**
	 * @brief Function for convert 3d model from .ply file to Mesh
	 * @param filename path to .ply file with 3d model
	 */
	Mesh* ply2mesh(std::string filename);
}
