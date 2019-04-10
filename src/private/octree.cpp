/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#include "octree.hpp"
#include <iostream>
#include <string>
#include <memory>
#include <cmath>

using namespace std;

float distance2(glm::vec3 a, glm::vec3 b) {
	return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}

TriangleOctree::TriangleOctree(float halfSize, uint8_t level, glm::vec3 pos):
	m_pos(pos),
	m_halfSize(halfSize),
	m_isLeaf(true),
	m_level(level)
{
	//std::cout << "TriangleOctree: " << halfSize << " " << pos << std::endl;;
	for (int i = 0; i < 8; i++)
		m_children[i] = nullptr;
}

TriangleOctree::~TriangleOctree() {
	if (!m_isLeaf)
		for (int i = 0; i < 8; i++)
			if (m_children[i])
				delete m_children[i];
}

int TriangleOctree::index(glm::vec3 point) const {
	int index = 0;
	if (point.x >= m_pos.x + m_halfSize)
		index |= X_SIGN_MASK;
	if (point.y >= m_pos.y + m_halfSize)
		index |= Y_SIGN_MASK;
	if (point.z >= m_pos.z + m_halfSize)
		index |= Z_SIGN_MASK;

	return index;
}

std::tuple<Triangle, float, int> TriangleOctree::nearTriangle(glm::vec3 point) const {
	shared_ptr<float> bound = make_shared<float>(std::numeric_limits<float>::max());
	shared_ptr<Triangle> ans(new Triangle());
	shared_ptr<int> ansi(new int(0));
	nearTriangleImpl(point, bound, ans, ansi);
	return { *ans, sqrt(*bound), *ansi };
}

void TriangleOctree::nearTriangleImpl(glm::vec3 p, std::shared_ptr<float> bound, std::shared_ptr<Triangle> ans, std::shared_ptr<int> ansi) const noexcept {
	float d = distanceToNode (p);
	if (d > *bound)
		return;
	if (m_isLeaf) {
		std::pair<float, int> calc;
		for (size_t i = 0; i < m_triangles.size(); i++) {
			calc = TriangleOctree::distance(p, m_triangles[i]);
			if (calc.first < *bound) {
				*ans = m_triangles[i];
				*bound = calc.first;
				*ansi = calc.second;
			}
		}
	} else {
		int idx = index(p);
		if (m_children[idx])
			m_children[idx]->nearTriangleImpl(p, bound, ans, ansi);

		for (int i = 0; i < 8; i++)
			if (i != idx && m_children[i])
				m_children[i]->nearTriangleImpl(p, bound, ans, ansi);
	}
}

void TriangleOctree::insert(Triangle triangle) {
	if (m_isLeaf) {
		if (m_triangles.size() < SPLIT_TRIANGLES_LIMIT || m_level >= MAX_DEPTH) {
			m_triangles.push_back(triangle);
		} else {
			m_isLeaf = false;

			for (size_t i = 0; i < m_triangles.size(); i++)
				insertAsNode(m_triangles[i]);

			m_triangles.clear();
		}
	} else {
		insertAsNode(triangle);
	}
}

bool TriangleOctree::inside(glm::vec3 point) const noexcept {
	return
		point.x >= m_pos.x && point.y >= m_pos.y && point.z >= m_pos.z &&
		point.x < (m_pos.x + 2*m_halfSize) && point.y < (m_pos.y + 2*m_halfSize) && point.z < (m_pos.z + 2*m_halfSize);
}

void TriangleOctree::insertAsNode(Triangle triangle) {
	int i1 = index(triangle.a);
	int i2 = index(triangle.b);
	int i3 = index(triangle.c);

	insertByIndex(i1, triangle);

	if (i2 != i1)
		insertByIndex(i2, triangle);

	if (i3 != i2 && i3 != i1)
		insertByIndex(i3, triangle);
}

void TriangleOctree::insertByIndex(int idx, Triangle triangle) {
	if (m_children[idx] == nullptr) {
		glm::vec3 subpos = m_pos;
		subpos.x += (idx & X_SIGN_MASK ? 1 : 0) * m_halfSize;
		subpos.y += (idx & Y_SIGN_MASK ? 1 : 0) * m_halfSize;
		subpos.z += (idx & Z_SIGN_MASK ? 1 : 0) * m_halfSize;

		m_children[idx] = new TriangleOctree(m_halfSize / 2, m_level+1, subpos);
	}
	m_children[idx]->insert(triangle);
}

void TriangleOctree::printInfoRecursify(int indent) const {
	if (m_isLeaf) {
		cout << string(indent, '-') << (int)m_level << endl;
	} else {
		for (int i = 0; i < 8; i++)
			if (m_children[i])
				m_children[i]->printInfoRecursify(indent+2);
	}
}

pair<float, int> TriangleOctree::distance (glm::vec3 P, Triangle tri) {
	glm::vec3 AB = tri.b - tri.a;
	glm::vec3 CA = tri.a - tri.c;
	float d_ab = glm::dot (AB, P - tri.a) / glm::dot (AB, AB);
	float d_ca = glm::dot (CA, P - tri.c) / glm::dot (CA, CA);
	// Check vertices
	if (d_ca >= 1 && d_ab <= 0)
		return { distance2 (tri.a, P), glm::sign(glm::dot(tri.a-P, tri.normal)) };
	glm::vec3 BC = tri.c - tri.b;
	float d_bc = glm::dot (BC, P - tri.b) / glm::dot (BC, BC);
	if (d_ab >= 1 && d_bc <= 0)
		return { distance2 (tri.b, P), glm::sign(glm::dot(tri.b-P, tri.normal)) };
	if (d_bc >= 1 && d_ca <= 0)
		return { distance2 (tri.c, P), glm::sign(glm::dot(tri.c-P, tri.normal)) };
	// Check edges
	if (d_ab >= 0 && d_ab <= 1) {
		glm::vec3 normalAB = glm::cross (AB, tri.normal);
		if (glm::dot (normalAB, P - tri.a) >= 0) {
			glm::vec3 K = tri.a + d_ab * AB;
			return { distance2 (K, P), glm::sign(glm::dot(K-P, tri.normal)) };
		}
	}
	if (d_bc >= 0 && d_bc <= 1) {
		glm::vec3 normalBC = glm::cross (BC, tri.normal);
		if (glm::dot (normalBC, P - tri.b) >= 0) {
			glm::vec3 K = tri.b + d_bc * BC;
			return { distance2 (K, P), glm::sign(glm::dot(K-P, tri.normal)) };
		}
	}
	if (d_ca >= 0 && d_ca <= 1) {
		glm::vec3 normalCA = glm::cross (CA, tri.normal);
		if (glm::dot (normalCA, P - tri.c) >= 0) {
			glm::vec3 K = tri.c + d_ca * CA;
			return { distance2(K, P), glm::sign(glm::dot(K-P, tri.normal)) };
		}
	}
	// Point is inside triangle, direct answer
	glm::vec3 K = P - glm::dot (P - tri.a, tri.normal) * tri.normal;
	return { distance2 (K, P), glm::sign(glm::dot (K - P, tri.normal)) };
}

float TriangleOctree::distanceToNode(glm::vec3 p) const noexcept {
	float x = glm::clamp(m_pos.x, p.x, m_pos.x + 2.0f * m_halfSize);
	float y = glm::clamp(m_pos.y, p.y, m_pos.y + 2.0f * m_halfSize);
	float z = glm::clamp(m_pos.z, p.z, m_pos.z + 2.0f * m_halfSize);
	glm::vec3 near(x,y,z);

	return distance2(near, p);
}
