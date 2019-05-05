/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Node of MDC octree
*/
#pragma once

#include "../common.hpp"
#include "../qef/qef_solver_3d.hpp"

#include <vector>

namespace isomesh
{

struct MDC_Vertex {
	glm::vec3 m_position = { 0.0f, 0.0f, 0.0f };
	glm::vec3 m_normal = { 0.0f, 0.0f, 0.0f };
	Material m_material = Material::Empty;
	bool m_collapsible = true;
	float m_error = -1.0f;
	MDC_Vertex *m_parent = nullptr;
	uint32_t m_vertexIdx = kBadIndex;
	uint32_t m_surfaceIdx = kBadIndex;
	QefSolver3D::State m_qef;

	MDC_Vertex *highestAncestor () noexcept;
	MDC_Vertex *highestCollapsibleAncestor () noexcept;
	const MDC_Vertex *highestAncestor () const noexcept;
	const MDC_Vertex *highestCollapsibleAncestor () const noexcept;
	bool hasCollapsibleAncestor () const noexcept;
};

struct MDC_OctreeNode {
	~MDC_OctreeNode () noexcept;
	
	void subdivide ();
	void collapse () noexcept;
	void setVertexMask (uint8_t value) noexcept { m_vertexMask = value; }
	void setCorners (std::array<Material, 8> value) noexcept { m_corners = value; }

	bool isSubdivided () const noexcept { return !m_isLeaf; }
	bool isLeaf () const noexcept { return m_isLeaf; }
	uint8_t depth () const noexcept { return m_depth; }
	uint8_t vertexMask () const noexcept { return m_vertexMask; }

	const MDC_OctreeNode *child (int num) const noexcept { return m_children + num; }
	Material corner (int num) const noexcept { return m_corners[num]; }
	const std::array<Material, 8> corners () const noexcept { return m_corners; }

	MDC_OctreeNode *child (int num) noexcept { return m_children + num; }
	Material &corner (int num) noexcept { return m_corners[num]; }
	std::array<Material, 8> &corners () noexcept { return m_corners; }

private:
	union {
		MDC_OctreeNode *m_children = nullptr;
		std::array<Material, 8> m_corners;
	};
	bool m_isLeaf = true;
	bool m_isSubtreeCollapsed = true;
	uint8_t m_depth = 0;
	uint8_t m_vertexMask = 0;
public:
	std::vector<MDC_Vertex> m_vertices;
};

}
