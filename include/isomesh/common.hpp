/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
#pragma once

#include <functional>

#include <glm/glm.hpp>

namespace isomesh
{

/** \brief Isosurface function wrapper

	This is a thin wrapper structure that packs together an isosurface defining
	function and its gradient. An isosurface function should have these properties:
	- Be continuous over R^3
	- Have its gradient (or at least some approximation) defined everywhere
	- Have negative sign inside the surface and positive outside
*/
struct SurfaceFunction {
	std::function<double (glm::dvec3)> f;
	std::function<glm::dvec3 (glm::dvec3)> grad;
	double operator () (const glm::dvec3 &p) const noexcept { return f (p); }
};

}

namespace std
{

template<size_t I, int D, typename T>
constexpr T &get (glm::vec<D, T, glm::defaultp> &v) noexcept {
	static_assert (I < D, "Index should be less than vector dimension");
	static_assert (D <= 4, "Dimension should be not higher than 4");
	if constexpr (I == 0)
		return v.x;
	else if constexpr (I == 1)
		return v.y;
	else if constexpr (I == 2)
		return v.z;
	else
		return v.w;
}

template<size_t I, int D, typename T>
constexpr const T &get (const glm::vec<D, T, glm::defaultp> &v) noexcept {
	static_assert (I < D, "Index should be less than vector dimension");
	static_assert (D <= 4, "Dimension should be not higher than 4");
	if constexpr (I == 0)
		return v.x;
	else if constexpr (I == 1)
		return v.y;
	else if constexpr (I == 2)
		return v.z;
	else
		return v.w;
}

}
