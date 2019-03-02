/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include <isomesh/fun/heightmap.hpp>

#include <cmath>
#include <algorithm>
#include <exception>
#include <experimental/filesystem>

#include "../private/stbi_data.hpp"

using namespace std;
using namespace std::experimental::filesystem;

namespace isomesh {
	Heightmap::Heightmap(pair<double, double> height_range, glm::dvec3 center, double pixel_size):
		m_data(nullptr),
		m_heightRange(height_range),
		m_pixelSize(pixel_size),
		m_center(center) {
	}

	Heightmap::~Heightmap() {
		m_data = nullptr;
	}

	void Heightmap::loadGrayscaleMap(string filename){
		m_data = make_shared<STBImageData>(filename);

		if (m_data->bpp() != 1) {
			m_data = nullptr;
			throw std::runtime_error("file " + filename + " not grayscale image file");
		}
	}

	double Heightmap::value(double px, double py, double pz) const noexcept {
		double h = 0;

		if (m_data) {
			double x = (px - m_center.x) / m_pixelSize + m_data->width()/2;
			double y = (pz - m_center.z) / m_pixelSize + m_data->height()/2;
			glm::dvec2 p(floor(x), floor(y));

			double q11 = heightVal(p.x, p.y);
			double q12 = heightVal(p.x, p.y + 1);
			double q21 = heightVal(p.x + 1, p.y);
			double q22 = heightVal(p.x + 1, p.y + 1);

			// Bilinear interpolation
			double h =
				q11 * (p.x+1 - x) * (p.y+1 - y)
				+ q21 * (x - p.x) * (p.y+1 - y)
				+ q12 * (p.x+1 - x) * (y - p.y)
				+ q22 * (x - p.x) * (y - p.y);
		}

		h += m_center.y;

		return py - h;
	}

	glm::dvec3 Heightmap::grad(double x, double y, double z) const noexcept {
		const double h = m_pixelSize / 4;

		const double x1 = value(x - h, y, z);
		const double x2 = value(x + h, y, z);
		const double z1 = value(x, y, z - h);
		const double z2 = value(x, y, z + h);

		return {(x2 - x1)/2/h, 1, (z2 - z1)/2/h};
	}

	double Heightmap::heightVal(int x, int y) const {
		stbi_us color = m_data->operator[](clamp(x, 0, m_data->height()-1) * m_data->width() + clamp(y, 0, m_data->width()-1));
		return m_heightRange.first + color * (m_heightRange.second - m_heightRange.first) / 65536.0;
	}
}

