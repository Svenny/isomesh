/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
// Simple test to check GLM installation correctness
#include <glm/glm.hpp>
#include <isomesh/common.hpp>

int main () {
	glm::mat3 I (1.0f);
	glm::vec3 a (1.0f, 2.0f, 3.0f), b (5.0f, -2.0f, 0.25f);
	if (I * a != a || I * b != b)
		return 1;
	
	auto x = std::get<0> (a);
	auto y = std::get<1> (a);
	auto z = std::get<2> (a);
	if (x != a.x || y != a.y || z != a.z)
		return 2;
	
	std::get<0> (b) += 2.0f;
	std::get<1> (b) += 2.0f;
	std::get<2> (b) += 2.0f;
	if (b != glm::vec3 (7.0f, 0.0f, 2.25f))
		return 3;
	
	auto x2 = std::get<0> (glm::dvec3 (1.0, 2.0, 3.0));
	auto y2 = std::get<1> (glm::dvec3 (1.0, 2.0, 3.0));
	auto z2 = std::get<2> (glm::dvec3 (1.0, 2.0, 3.0));
	if (x2 != 1.0 || y2 != 2.0 || z2 != 3.0)
		return 4;
	
	return 0;
}

