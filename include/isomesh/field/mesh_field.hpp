/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
/** @file
 * @brief Scalar field from 3D model
 */
#pragma once

#include <glm/common.hpp>
#include "../../../src/private/ply_data.hpp"
#include <isomesh/field/scalar_field.hpp>

class TriangleOctree;

namespace isomesh {
	/**
	 * @brief Scalar field from 3D model .ply file
	 *
	 * Value of scalar field treates as distance to near model triangle, with sign, depends on normal of the triangle
	 */
	class MeshField : public ScalarField {
	public:
		MeshField();

		/**
		@brief Load 3d model
		@param filename path to .ply file with 3d model
		*/
		void load(std::string filename);

		double value (double x, double y, double z) const noexcept override;
		glm::dvec3 grad (double x, double y, double z) const noexcept override;
	private:
		void fillOctree();
	private:
		PlyData m_data;
		TriangleOctree* m_root;
	};
}
