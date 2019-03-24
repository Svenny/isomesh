/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include <isomesh/util/ply_mesh.hpp>
#include <fstream>
#include <iostream>
#include <memory>

using namespace isomesh;
using namespace std;

isomesh::PlyMesh::PlyMesh():
	m_scale(1.0)
{
}

float isomesh::PlyMesh::scale() noexcept
{
	return m_scale;
}

void isomesh::PlyMesh::setScale(float scale) noexcept
{
	m_scale = scale;
}

void isomesh::PlyMesh::load(string filename)
{
	m_data.load(filename);
	calculateNormals();
}

void isomesh::PlyMesh::calculateNormals()
{
	const size_t vcount = m_data.verticesCount();
	const size_t fcount = m_data.trianglesCount();

	fill(m_normals.begin(), m_normals.end(), glm::vec3(0,0,0));
	m_normals.resize(vcount, glm::vec3(0,0,0));

	for (size_t i = 0; i < fcount; i++) {
		glm::ivec3 indexs = m_data.triangleIndexs(i);
		glm::vec3 p1 = m_data.vertex(indexs.x);
		glm::vec3 p2 = m_data.vertex(indexs.y);
		glm::vec3 p3 = m_data.vertex(indexs.z);

		glm::vec3 normal = glm::cross(p1 - p2, p1 - p3);
		m_normals[indexs.x] += normal;
		m_normals[indexs.y] += normal;
		m_normals[indexs.z] += normal;
	}

	for (size_t i = 0; i < m_normals.size(); i++) {
		m_normals[i] = glm::normalize(m_normals[i]);
	}
}

isomesh::Mesh* isomesh::PlyMesh::mesh()
{
	const size_t vcount = m_data.verticesCount();
	const size_t fcount = m_data.trianglesCount();

	Mesh* ptr = new Mesh(vcount, fcount);

	for (size_t i = 0; i < vcount; i++) {
		ptr->addVertex(m_data.vertex(i) * m_scale, m_normals[i], isomesh::Material::Stone);
	}

	for (size_t i = 0; i < fcount; i++) {
		glm::ivec3 idx = m_data.triangleIndexs(i);
		ptr->addTriangle(idx.x, idx.y, idx.z);
	}

	return ptr;
}

bool isomesh::PlyMesh::loaded() noexcept
{
	return m_data.loaded();
}
