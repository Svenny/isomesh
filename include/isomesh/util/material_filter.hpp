/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Defines material choosing functions
*/
#pragma once

#include "../common.hpp"

#include <array>
#include <type_traits>

namespace isomesh
{

namespace matfilter_detail
{

constexpr bool kUsePreciseFilter = false;

class MaterialFilterPrecise {
public:
	MaterialFilterPrecise () noexcept { m_counts.fill (0); }

	void add (Material mat) noexcept { m_counts[size_t (mat)]++; }
	void add (const std::array<Material, 8> &mats) noexcept {
		for (auto mat : mats)
			m_counts[size_t (mat)]++;
	}
	void add (const MaterialFilterPrecise &flt) noexcept {
		for (size_t i = 0; i < m_counts.size (); i++)
			m_counts[i] += flt.m_counts[i];
	}

	void reset () noexcept { m_counts.fill (0); }
	Material select () const noexcept;

private:
	std::array<uint32_t, size_t (Material::Count)> m_counts;
};

class MaterialFilterFast {
public:
	MaterialFilterFast () noexcept : m_candidate (Material::Empty), m_count (0) {}

	void add (Material mat) noexcept;
	void add (const std::array<Material, 8> &mats) noexcept {
		for (auto mat : mats)
			add (mat);
	}
	void add (const MaterialFilterFast &flt) noexcept;

	void reset () noexcept {
		m_candidate = Material::Empty;
		m_count = 0;
	}
	Material select () const noexcept { return m_candidate; }

private:
	Material m_candidate;
	uint32_t m_count;
};

}

using MaterialFilter = std::conditional_t<matfilter_detail::kUsePreciseFilter,
	matfilter_detail::MaterialFilterPrecise, matfilter_detail::MaterialFilterFast>;

}
