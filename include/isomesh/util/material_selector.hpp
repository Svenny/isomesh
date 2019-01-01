/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
/** \file
	\brief Defines material selection functions
*/
#pragma once

#include "../common.hpp"

namespace isomesh
{

class MaterialSelector {
public:
	virtual Material select (glm::dvec3 pos, double val) const noexcept = 0;
};

class TrivialMaterialSelector : public MaterialSelector {
public:
	virtual Material select (glm::dvec3 pos, double val) const noexcept override {
		if (val > 0.0)
			return Material::Empty;
		return Material::Stone;
	}
};

}
