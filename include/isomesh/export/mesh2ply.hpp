/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once
/** @file */

namespace isomesh {
	class Mesh;

	/**
	 * @brief Export function for save @p mesh to .ply file
	 * @param mesh exported mesh
	 * @param filename path to result file
	 */
	void mesh2ply(const Mesh* mesh, std::string filename);
}
