/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <vector>

#include <glm/vec3.hpp>
#include "triangle.hpp"

class TriangleOctree {
public:
	TriangleOctree(float halfSize, uint8_t level = 0, glm::vec3 pos = glm::vec3(0));
	~TriangleOctree();

	void insert(Triangle triangle);

	std::tuple<Triangle, float, int> nearTriangle(glm::vec3 p) const;

	size_t printInfoRecursify(int indent = 0, int level = 1);

	float halfSize() const noexcept {return m_halfSize;}
private:
	int index(glm::vec3 point) const;
	void insertByIndex(int idx, Triangle triangle);
	void insertAsNode(Triangle triangle);
	// Returns founded Triangle, squared distance and sign
	void nearTriangleImpl(glm::vec3 p, std::shared_ptr<float> bound, std::shared_ptr<Triangle> ans, std::shared_ptr<int> ansi) const noexcept;
	bool inside(glm::vec3 point) const noexcept;
	float distanceToNode(glm::vec3 p) const noexcept;

	static std::pair<float, int> distance(glm::vec3 p, Triangle tri);
private:
	const static int X_SIGN_MASK = 1;
	const static int Y_SIGN_MASK = 2;
	const static int Z_SIGN_MASK = 4;
	const static int SPLIT_TRIANGLES_LIMIT = 20;
	const static int MAX_DEPTH = 14;
private:
	glm::vec3 m_pos;
	float m_halfSize;
	bool m_isLeaf;
	uint8_t m_level;
	TriangleOctree* m_children[8];
	std::vector<Triangle> m_triangles;
};
