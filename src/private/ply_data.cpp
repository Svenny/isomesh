/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include "ply_data.hpp"

#include <fstream>
#include <array>
#include <string>

PlyData::PlyData():
	m_loaded(false),
	m_vertices(nullptr),
	m_faces(nullptr)
{

}

void PlyData::load(std::string filename)
{
	m_loaded = false;
	tinyply::PlyFile file;
	std::ifstream fin(filename, std::ios::binary);
	if (!fin.is_open())
		throw std::runtime_error("file" + filename + "not found");
	file.parse_header(fin);

	m_vertices = file.request_properties_from_element("vertex", { "x", "y", "z"}, 3);
	std::cerr << "vertices: " << tinyply::PropertyTable[m_vertices->t].str << std::endl;
	m_faces = file.request_properties_from_element("face", { "vertex_indices" }, 3);
	std::cerr << "faces: " << tinyply::PropertyTable[m_faces->t].str << std::endl;

	file.read(fin);
	m_loaded = true;
}

glm::vec3 PlyData::vertex(const size_t idx)
{
	float* vdata = reinterpret_cast<float*>(m_vertices->buffer.get());
	return glm::vec3(vdata[3*idx], vdata[3*idx+1], vdata[3*idx+2]);
}
size_t PlyData::verticesCount() noexcept
{
	if (m_vertices)
		return m_vertices->count;
	else
		return 0;
}

std::array<glm::vec3, 3> PlyData::triangle(const size_t idx)
{
	const glm::ivec3& indexs = triangleIndexs(idx);

	return {vertex(indexs.x), vertex(indexs.y), vertex(indexs.z)};
}

glm::ivec3 PlyData::triangleIndexs(const size_t idx)
{
	int* fdata = reinterpret_cast<int*>(m_faces->buffer.get());
	int i1 = fdata[3*idx];
	int i2 = fdata[3*idx+1];
	int i3 = fdata[3*idx+2];

	return {i1, i2, i3};
}

size_t PlyData::trianglesCount() noexcept
{
	if (m_faces)
		return m_faces->count;
	else
		return 0;
}

bool PlyData::loaded() noexcept
{
	return m_loaded;
}
