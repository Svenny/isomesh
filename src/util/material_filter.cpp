/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/util/material_filter.hpp>

#include <algorithm>

namespace isomesh
{

Material AnyNonemptyMaterialFilter::select (const std::array<Material, 8> &corners, uint8_t mask) const {
	for (uint32_t i = 0; i < 8; i++)
		if ((mask & (1 << i)) && corners[i] != Material::Empty)
			return corners[i];
	return Material::Empty;
}

Material AnyNonemptyMaterialFilter::select (const UniformGrid &G, glm::ivec3 pos, uint8_t mask) const {
	uint32_t idx = 0;
	for (int32_t y = 0; y <= 1; y++) {
		for (int32_t x = 0; x <= 1; x++) {
			for (int32_t z = 0; z <= 1; z++) {
				if (mask & (1 << idx)) {
					auto mat = G.at (pos.x + x, pos.y + y, pos.z + z);
					if (mat != Material::Empty)
						return mat;
				}
				idx++;
			}
		}
	}
	return Material::Empty;
}

Material HistogramMaterialFilter::select (const std::array<Material, 8> &corners, uint8_t mask) const {
	constexpr size_t n = size_t (Material::Count);
	uint32_t cnt[n];
	std::fill (cnt, cnt + n, 0);

	for (uint32_t i = 0; i < 8; i++) {
		if (mask & (1 << i)) {
			auto mat = corners[i];
			if (mat != Material::Empty)
				cnt[size_t (mat)]++;
		}
	}

	Material mat = Material (std::max_element (cnt, cnt + n) - cnt);
	return mat;
}

Material HistogramMaterialFilter::select (const UniformGrid &G, glm::ivec3 pos, uint8_t mask) const {
	constexpr size_t n = size_t (Material::Count);
	uint32_t cnt[n];
	std::fill (cnt, cnt + n, 0);

	uint32_t idx = 0;
	for (int32_t y = 0; y <= 1; y++) {
		for (int32_t x = 0; x <= 1; x++) {
			for (int32_t z = 0; z <= 1; z++) {
				if (mask & (1 << idx)) {
					auto mat = G.at (pos.x + x, pos.y + y, pos.z + z);
					if (mat != Material::Empty)
						cnt[size_t (mat)]++;
				}
				idx++;
			}
		}
	}

	Material mat = Material (std::max_element (cnt, cnt + n) - cnt);
	return mat;
}

}
