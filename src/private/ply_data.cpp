/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include "ply_data.hpp"

#include <fstream>
#include <array>
#include <string>
#include <iostream>
#include <cmath>

PlyData::PlyData():
	m_loaded(false),
	m_directWindingOrder(true),
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
		throw std::runtime_error("file " + filename + " not found");
	file.parse_header(fin);

	m_vertices = file.request_properties_from_element("vertex", { "x", "y", "z"}, 3);
	//std::cerr << "vertices: " << tinyply::PropertyTable[m_vertices->t].str << std::endl;
	m_faces = file.request_properties_from_element("face", { "vertex_indices" }, 3);
	//std::cerr << "faces: " << tinyply::PropertyTable[m_faces->t].str << std::endl;

	file.read(fin);
	normalization();
	m_loaded = true;
}

glm::vec3 PlyData::vertex(const size_t idx) const
{
	float* vdata = reinterpret_cast<float*>(m_vertices->buffer.get());
	return (glm::vec3(vdata[3*idx], vdata[3*idx+1], vdata[3*idx+2]) - m_center) / m_multiplier;
}

size_t PlyData::verticesCount() const noexcept
{
	if (m_vertices)
		return m_vertices->count;
	else
		return 0;
}

Triangle PlyData::triangle(const size_t idx) const
{
	const glm::ivec3& indexs = triangleIndexs(idx);

	return Triangle(vertex(indexs.x), vertex(indexs.y), vertex(indexs.z));
}

glm::ivec3 PlyData::triangleIndexs(const size_t idx) const
{
	int* fdata = reinterpret_cast<int*>(m_faces->buffer.get());
	int i1 = fdata[3*idx];
	int i2, i3;
	if (m_directWindingOrder) {
		i2 = fdata[3*idx+1];
		i3 = fdata[3*idx+2];
	} else {
		i2 = fdata[3*idx+2];
		i3 = fdata[3*idx+1];
	}

	return {i1, i2, i3};
}

size_t PlyData::trianglesCount() const noexcept
{
	if (m_faces)
		return m_faces->count;
	else
		return 0;
}

bool PlyData::loaded() const noexcept
{
	return m_loaded;
}

void PlyData::normalization() noexcept
{
	float minX = std::numeric_limits<float>::max();
	float maxX = std::numeric_limits<float>::min();
	float minY = std::numeric_limits<float>::max();
	float maxY = std::numeric_limits<float>::min();
	float minZ = std::numeric_limits<float>::max();
	float maxZ = std::numeric_limits<float>::min();

	float* vdata = reinterpret_cast<float*>(m_vertices->buffer.get());
	size_t vcount = m_vertices->count;

	for (size_t i = 0; i < vcount; i++) {
		float x = vdata[3*i];
		float y = vdata[3*i+1];
		float z = vdata[3*i+2];

		if (x < minX)
			minX = x;
		if (x > maxX)
			maxX = x;

		if (y < minY)
			minY = y;
		if (y > maxY)
			maxY = y;

		if (z < minZ)
			minZ = z;
		if (z > maxZ)
			maxZ = z;
	}

	m_multiplier = std::max(maxX - minX, std::max(maxY - minY, maxZ - minZ));
	m_multiplier /= 32;
	m_center = glm::vec3((maxX + minX)/2, (maxY + minY)/2, (maxZ + minZ)/2);
}

void PlyData::setWindingOrder(WindingOrder order) noexcept {
	switch (order) {
		case WindingOrder::Direct:
			m_directWindingOrder = true;
			break;

		case WindingOrder::Inverted:
			m_directWindingOrder = false;
			break;
	}
}

WindingOrder PlyData::windingOrder() const noexcept {
	return m_directWindingOrder ? WindingOrder::Direct : WindingOrder::Inverted;
}
