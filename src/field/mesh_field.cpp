/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include <isomesh/field/mesh_field.hpp>

#include <limits>
#include <iostream>
#include <cmath>

using namespace std;

void isomesh::MeshField::load(std::string filename)
{
	m_data.load(filename);
	calculateNormalsAndCenters();
}

double isomesh::MeshField::value (double x, double y, double z) const noexcept
{
	glm::vec3 p(x, y, z);

	size_t idx = nearTriangle(p);

	glm::ivec3 indexs = m_data.triangleIndexs(idx);
	glm::vec3 p1 = m_data.vertex(indexs.x);
	glm::vec3 p2 = m_data.vertex(indexs.y);
	glm::vec3 p3 = m_data.vertex(indexs.z);

	glm::vec3 center = m_centers[idx];
	glm::vec3 normal = m_normals[idx];

	float h = glm::dot(normal, p - center);
	glm::vec3 projP = p - normal*h;

	float d1 = glm::distance(projP, p1);
	float d2 = glm::distance(projP, p2);
	float d3 = glm::distance(projP, p3);
	float dmax = max(d1, max(d2, d3));

	glm::vec3 edgeP;
	if (dmax == d1) {
		edgeP = glm::dot(p, p2 - p3) / glm::dot(p2 - p3, p2 - p3) * (p2-p3);
	} else if (dmax == d2) {
		edgeP = glm::dot(p, p1 - p3) / glm::dot(p1 - p3, p1 - p3) * (p1-p3);
	} else {
		edgeP = glm::dot(p, p2 - p1) / glm::dot(p2 - p1, p2 - p1) * (p2-p1);
	}

	float dproj = glm::distance(edgeP, center);
	if (glm::distance(center, projP) > dproj)
		return glm::sign(h) * glm::distance(edgeP, p);
	else
		return h;
}

glm::dvec3 isomesh::MeshField::grad (double x, double y, double z) const noexcept
{
	glm::vec3 p(x, y, z);
	size_t idx = nearTriangle(p);
	return m_normals[idx];
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
