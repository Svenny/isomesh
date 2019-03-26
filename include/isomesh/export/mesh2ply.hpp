/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

namespace isomesh {
	class Mesh;

	void mesh2ply(const Mesh* mesh, std::string filename);
}
