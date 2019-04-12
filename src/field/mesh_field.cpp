/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include <isomesh/field/mesh_field.hpp>

#include <limits>
#include <iostream>
#include <cmath>
#include <limits>

#include "../private/octree.hpp"

using namespace std;

isomesh::MeshField::MeshField():
	m_root(nullptr)
{
}

void isomesh::MeshField::load(std::string filename)
{
	m_data.load(filename);
	calculateNormalsAndCenters();
	fillOctree();
}

double isomesh::MeshField::value (double x, double y, double z) const noexcept
{
	glm::vec3 p(x, y, z);
	std::tuple<Triangle, float, int> ans = m_root->nearTriangle(p);
	return get<1>(ans) * get<2>(ans);
}

glm::dvec3 isomesh::MeshField::grad (double x, double y, double z) const noexcept
{
	const double h = m_root->halfSize() / 500;

	const double x1 = value(x - h, y, z);
	const double x2 = value(x + h, y, z);
	const double y1 = value(x, y - h, z);
	const double y2 = value(x, y + h, z);
	const double z1 = value(x, y, z - h);
	const double z2 = value(x, y, z + h);

	return {-(x2 - x1)/2/h, -(y2 - y1)/2/h, -(z2 - z1)/2/h};
	/*
	glm::vec3 p(x, y, z);
	std::tuple<Triangle, float, int> ans = m_root->nearTriangle(p);
	return get<0>(ans).normal;
	*/
}

void isomesh::MeshField::calculateNormalsAndCenters()
{
	const size_t fcount = m_data.trianglesCount();

	m_normals.resize(fcount);
	m_centers.resize(fcount);

	for (size_t i = 0; i < fcount; i++) {
		glm::ivec3 indexs = m_data.triangleIndexs(i);
		glm::vec3 p1 = m_data.vertex(indexs.x);
		glm::vec3 p2 = m_data.vertex(indexs.y);
		glm::vec3 p3 = m_data.vertex(indexs.z);

		m_normals[i] = glm::normalize(glm::cross(p1 - p2, p1 - p3));
		m_centers[i] = (p1 + p2 + p3) / 3.0f;
	}
}

size_t isomesh::MeshField::nearTriangle(glm::dvec3 p) const noexcept
{
	glm::vec3 s = p;
	const size_t fcount = m_data.trianglesCount();

	float dist = std::numeric_limits<float>::max();
	size_t idx = 0;
	for (size_t i = 0; i < fcount; i++) {
		float l = glm::distance(s, m_centers[i]);
		if (l < dist) {
			dist = l;
			idx = i;
		}
	}

	return idx;
}

void isomesh::MeshField::fillOctree() {
	const size_t vcount = m_data.verticesCount();

	float minX = std::numeric_limits<float>::max();
	float maxX = std::numeric_limits<float>::min();
	float minY = std::numeric_limits<float>::max();
	float maxY = std::numeric_limits<float>::min();
	float minZ = std::numeric_limits<float>::max();
	float maxZ = std::numeric_limits<float>::min();
	for (size_t i = 0; i < vcount; i++) {
		glm::vec3 p = m_data.vertex(i);
		//std::cout << "v: " << p.x << " " << p.y << " " << p.z << std::endl;
		if (p.x < minX)
			minX = p.x;
		if (p.x > maxX)
			maxX = p.x;

		if (p.y < minY)
			minY = p.y;
		if (p.y > maxY)
			maxY = p.y;

		if (p.z < minZ)
			minZ = p.z;
		if (p.z > maxZ)
			maxZ = p.z;
	}

	//std::cout << "mins: " << minX << " " << minY << " " << minZ << std::endl;
	//std::cout << "maxs: " << maxX << " " << maxY << " " << maxZ << std::endl;
	float treeSize = max(maxX - minX, max(maxY - minY, maxZ - minZ));
	//std::cout << "treeSize: " << treeSize << std::endl;
	//std::cout << "minpos: " << minX << " " << minY << " " << minZ << std::endl;
	if (m_root)
		delete m_root;
	m_root = new TriangleOctree(treeSize/2, 0, glm::vec3(minX, minY, minZ));

	const size_t fcount = m_data.trianglesCount();
	std::cout << "load model with " << fcount << " triangles" << std::endl;
	for (size_t i = 0; i < fcount; i++) {
		m_root->insert(m_data.triangle(i));
	}

	//m_root->printInfoRecursify();
}
