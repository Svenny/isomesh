/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <glm/common.hpp>
#include <glm/ext.hpp>

struct Triangle {
	Triangle() = default;

	Triangle(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
		this->a = a;
		this->b = b;
		this->c = c;
		this->normal = glm::normalize(glm::cross(a - b, a - c));
	}

	glm::vec3 a, b, c;
	glm::vec3 normal;
};
