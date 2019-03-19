/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <isomesh/data/mesh.hpp>
#include "../../../src/private/ply_data.hpp"

namespace isomesh {
	class PlyMesh {
	public:
		PlyMesh();

		void load(std::string filename);

		float scale() noexcept;
		void setScale(float scale) noexcept;

		Mesh* mesh();

		bool loaded() noexcept;
	private:
		void calculateNormals();

	private:
		PlyData m_data;
		std::vector<glm::vec3> m_normals;
		float m_scale;
	};
}
