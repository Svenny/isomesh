/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <isomesh/data/mesh.hpp>
#include "../../../src/private/ply_data.hpp"

namespace isomesh {
	/**
	 * Converter, which load 3D model from .ply file and generate isomesh::Mesh from it with some @p scale.
	 */
	class PlyMesh {
	public:
		PlyMesh();

		/**
		 * @brief Load 3D model from file
		 * @param filename path to model's file
		 */
		void load(std::string filename);

		float scale() noexcept;
		void setScale(float scale) noexcept;

		/**
		 * @brief Generate mesh from model
		 * @return mesh corresponding .ply data
		 */
		Mesh* mesh();

		/// Checks, if 3D model data loaded from file
		bool loaded() noexcept;
	private:
		void calculateNormals();

	private:
		PlyData m_data;
		std::vector<glm::vec3> m_normals;
		float m_scale;
	};
}
