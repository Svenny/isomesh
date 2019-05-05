/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/data/mdc_octree_node.hpp>

#include <cassert>

namespace isomesh
{

MDC_Vertex *MDC_Vertex::highestAncestor () noexcept {
	MDC_Vertex *v = this;
	while (v->m_parent)
		v = v->m_parent;
	return v;
}

MDC_Vertex *MDC_Vertex::highestCollapsibleAncestor () noexcept {
	MDC_Vertex *result = (m_collapsible ? this : nullptr);
	MDC_Vertex *v = m_parent;
	while (v) {
		if (v->m_collapsible)
			result = v;
		v = v->m_parent;
	}
	return result;
}

const MDC_Vertex *MDC_Vertex::highestAncestor () const noexcept {
	const MDC_Vertex *v = this;
	while (v->m_parent)
		v = v->m_parent;
	return v;
}

const MDC_Vertex *MDC_Vertex::highestCollapsibleAncestor () const noexcept {
	const MDC_Vertex *result = (m_collapsible ? this : nullptr);
	const MDC_Vertex *v = m_parent;
	while (v) {
		if (v->m_collapsible)
			result = v;
		v = v->m_parent;
	}
	return result;
}

bool MDC_Vertex::hasCollapsibleAncestor () const noexcept {
	const MDC_Vertex *v = m_parent;
	while (v) {
		if (v->m_collapsible)
			return true;
		v = v->m_parent;
	}
	return false;
}

MDC_OctreeNode::~MDC_OctreeNode () noexcept {
	if (m_isLeaf)
		delete[] m_children;
}

void MDC_OctreeNode::subdivide () {
	if (m_isLeaf) {
		assert (m_depth != UINT8_MAX);
		m_children = new MDC_OctreeNode[8];
		for (int i = 0; i < 8; i++)
			m_children[i].m_depth = m_depth + 1;
		m_isLeaf = false;
	}
}

void MDC_OctreeNode::collapse () noexcept {
	if (!m_isLeaf) {
		delete[] m_children;
		m_children = nullptr;
		m_isLeaf = true;
	}
}

}
