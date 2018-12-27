/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
// Simple test to check GLM installation correctness
#include <glm/glm.hpp>

int main () {
	glm::mat3 I (1.0f);
	glm::vec3 a (1.0f, 2.0f, 3.0f), b (5.0f, -2.0f, 0.25f);
	if (I * a != a || I * b != b)
		return 1;
	return 0;
}

