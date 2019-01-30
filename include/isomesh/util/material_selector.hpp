/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Defines material selection functions
*/
#pragma once

#include "../common.hpp"

namespace isomesh
{

/** \brief Selects material for a given space point

	Based on point coordinates and surface function value in that
	point. It is guaranteed that value is non-positive, as Empty
	material is automatically selected for positive values.
*/
class MaterialSelector {
public:
	virtual Material select (glm::dvec3 pos, double val) const = 0;
};

// Always selects stone
class TrivialMaterialSelector : public MaterialSelector {
public:
	virtual Material select (glm::dvec3 pos, double val) const override {
		return Material::Stone;
	}
};

// Selects stone or soil based on function value
class SimpleMaterialSelector : public MaterialSelector {
public:
	virtual Material select (glm::dvec3 pos, double val) const override {
		if (val >= -2)
			return Material::Soil;
		return Material::Stone;
	}
};

}
