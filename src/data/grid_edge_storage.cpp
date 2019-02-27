/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/grid_edge_storage.hpp>

#include <algorithm>
#include <cassert>
#include <limits>

namespace isomesh
{

UniformGridEdge::UniformGridEdge (int32_t lesserX, int32_t lesserY, int32_t lesserZ,
                                  const glm::dvec3 &gradient, double offset,
                                  int axis, bool isLesserEndpointSolid, Material solidMaterial) noexcept {
	constexpr int32_t max16 = std::numeric_limits<int16_t>::max ();
	constexpr int32_t min16 = std::numeric_limits<int16_t>::min ();
	assert (lesserX <= max16 && lesserY <= max16 && lesserZ <= max16);
	assert (lesserX >= min16 && lesserY >= min16 && lesserZ >= min16);
	assert (axis >= 0 && axis <= 2);
	this->lesserX = int16_t (lesserX);
	this->lesserY = int16_t (lesserY);
	this->lesserZ = int16_t (lesserZ);
	// Normalize gradient to get surface normal. Adding epsilon to avoid
	// possible division by zero in case of zero gradient.
	double grad_len = glm::length (gradient) + std::numeric_limits<double>::epsilon ();
	this->normalX = float (gradient.x / grad_len);
	this->normalZ = float (gradient.z / grad_len);
	this->normalYSign = (gradient.y < 0);
	this->offset = float (offset);
	this->axis = uint8_t (axis);
	this->solidEndpoint = uint8_t (!isLesserEndpointSolid);
	this->material = solidMaterial;
}

glm::vec3 UniformGridEdge::surfaceNormal () const noexcept {
	float y_squared = 1.0f - normalX * normalX - normalZ * normalZ;
	float normalY = glm::sqrt (glm::max (0.0f, y_squared));
	if (normalYSign)
		normalY = -normalY;
	return { normalX, normalY, normalZ };
}

glm::vec3 UniformGridEdge::surfacePoint () const noexcept {
	glm::vec3 point (lesserX, lesserY, lesserZ);
	point[axis] += offset;
	return point;
}

glm::ivec3 UniformGridEdge::lesserEndpoint () const noexcept {
	return { lesserX, lesserY, lesserZ };
}

glm::ivec3 UniformGridEdge::biggerEndpoint () const noexcept {
	glm::ivec3 point (lesserX, lesserY, lesserZ);
	point[axis]++;
	return point;
}

bool UniformGridEdgeStorage::edgeLess (const UniformGridEdge &a, const UniformGridEdge &b) noexcept {
	// Compare as (Y, X, Z) tuples
	if (a.lesserY < b.lesserY)
		return true;
	if (a.lesserY == b.lesserY) {
		if (a.lesserX < b.lesserX)
			return true;
		if (a.lesserX == b.lesserX)
			return a.lesserZ < b.lesserZ;
	}
	return false;
}

void UniformGridEdgeStorage::sortEdges () noexcept {
	std::sort (m_edges.begin (), m_edges.end (), edgeLess);
}

UniformGridEdgeStorage::iterator UniformGridEdgeStorage::findEdge
	(int32_t x, int32_t y, int32_t z) noexcept {
	constexpr int32_t max16 = std::numeric_limits<int16_t>::max ();
	constexpr int32_t min16 = std::numeric_limits<int16_t>::min ();
	if (x > max16 || y > max16 || z > max16)
		return end ();
	if (x < min16 || y < min16 || z < min16)
		return end ();
	UniformGridEdge sample;
	sample.lesserX = int16_t (x);
	sample.lesserY = int16_t (y);
	sample.lesserZ = int16_t (z);
	auto iter = std::lower_bound (m_edges.begin (), m_edges.end (), sample, edgeLess);
	if (iter == m_edges.end ())
		return iter;
	if (iter->lesserX != x || iter->lesserY != y || iter->lesserZ != z)
		return end ();
	return iter;
}

UniformGridEdgeStorage::const_iterator UniformGridEdgeStorage::findEdge
	(int32_t x, int32_t y, int32_t z) const noexcept {
	constexpr int32_t max16 = std::numeric_limits<int16_t>::max ();
	constexpr int32_t min16 = std::numeric_limits<int16_t>::min ();
	if (x > max16 || y > max16 || z > max16)
		return end ();
	if (x < min16 || y < min16 || z < min16)
		return end ();
	UniformGridEdge sample;
	sample.lesserX = int16_t (x);
	sample.lesserY = int16_t (y);
	sample.lesserZ = int16_t (z);
	auto iter = std::lower_bound (m_edges.begin (), m_edges.end (), sample, edgeLess);
	if (iter == m_edges.end ())
		return iter;
	if (iter->lesserX != x || iter->lesserY != y || iter->lesserZ != z)
		return end ();
	return iter;
}

}
