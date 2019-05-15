/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/util/material_filter.hpp>

#include <algorithm>

namespace isomesh
{

namespace matfilter_detail
{

Material MaterialFilterPrecise::select () const noexcept {
	// Add 1 to begin () to exclude empty material which should
	// never be returned when there is at least one nonempty material
	auto iter = std::max_element (m_counts.begin () + 1, m_counts.end ());
	if (*iter == 0)
		return Material::Empty;
	return Material (iter - m_counts.begin ());
}

void MaterialFilterFast::add (Material mat) noexcept {
	if (mat == Material::Empty)
		return;
	if (mat == m_candidate)
		m_count++;
	else if (m_count > 1)
		m_count--;
	else {
		m_candidate = mat;
		m_count = 1;
	}
}

void MaterialFilterFast::add (const MaterialFilterFast &flt) noexcept {
	if (m_candidate == flt.m_candidate)
		m_count += flt.m_count;
	else if (m_count > flt.m_count)
		m_count -= flt.m_count;
	else {
		m_candidate = flt.m_candidate;
		m_count = std::max (uint32_t (1), flt.m_count - m_count);
	}
}

}

}
