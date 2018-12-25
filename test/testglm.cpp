#include <glm/glm.hpp>

int main () {
	glm::mat3 I (1.0f);
	glm::vec3 a (1.0f, 2.0f, 3.0f), b (5.0f, -2.0f, 0.25f);
	if (I * a != a || I * b != b)
		return 1;
	return 0;
}

