/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <glm/vec3.hpp>
#include "../3dparty/tinyply.h"

class PlyData {
public:
	PlyData();

	void load(std::string filename);

	glm::vec3 vertex(const size_t idx) const;
	size_t verticesCount() const noexcept;

	std::array<glm::vec3, 3> triangle(const size_t idx) const;
	glm::ivec3 triangleIndexs(const size_t idx) const;
	size_t trianglesCount() const noexcept;

	bool loaded() const noexcept;
private:
	void calcCenter() noexcept;

private:
	bool m_loaded;
	std::shared_ptr<tinyply::PlyData> m_vertices, m_faces;
	glm::vec3 m_center;
};
