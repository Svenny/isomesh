/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <glm/common.hpp>
#include "../../../src/private/ply_data.hpp"
#include <isomesh/field/scalar_field.hpp>

namespace isomesh {
	class MeshField : public ScalarField {
	public:
		MeshField() = default;

		void load(std::string filename);

		double value (double x, double y, double z) const noexcept override;
		glm::dvec3 grad (double x, double y, double z) const noexcept override;
	private:
		void calculateNormalsAndCenters();
		// returns index of this triangle and distance from p to it
		size_t nearTriangle(glm::dvec3 p) const noexcept;

	private:
		PlyData m_data;
		std::vector<glm::vec3> m_normals;
		std::vector<glm::vec3> m_centers;
	};
}
