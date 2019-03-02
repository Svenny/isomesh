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
	HeightMapImporter::HeightMapImporter(pair<double, double> height_range, glm::dvec3 center, double pixel_size):
		m_data(nullptr),
		m_heightRange(height_range),
		m_pixelSize(pixel_size),
		m_center(center) {
	}

	HeightMapImporter::~HeightMapImporter() {
		m_data = nullptr;
	}

	SurfaceFunction HeightMapImporter::buildSurfaceFunction() {
		if (!m_data)
			throw std::logic_error("use surface function without data loading");

		SurfaceFunction surf;
		surf.f = std::bind(HeightMapImporter::f, m_data, m_heightRange, m_pixelSize, m_center, placeholders::_1);
		surf.grad = std::bind(HeightMapImporter::grad, m_data, m_heightRange, m_pixelSize, m_center, placeholders::_1);
		return surf;
	}

	void HeightMapImporter::loadGrayscaleMap(string filename){
		m_data = make_shared<STBImageData>(filename);

		if (m_data->bpp() != 1) {
			m_data = nullptr;
			throw std::runtime_error("file " + filename + " not grayscale image file");
		}
	}

	double HeightMapImporter::f(std::shared_ptr<STBImageData> data, std::pair<double, double> heightRange, double pixelSize, glm::dvec3 center, glm::dvec3 P) {
		double x = (P.x - center.x) / pixelSize + data->width()/2;
		double y = (P.z - center.z) / pixelSize + data->height()/2;
		glm::dvec2 p(floor(x), floor(y));

		double q11 = HeightMapImporter::heightVal(data, heightRange, p.x, p.y);
		double q12 = HeightMapImporter::heightVal(data, heightRange, p.x, p.y + 1);
		double q21 = HeightMapImporter::heightVal(data, heightRange, p.x + 1, p.y);
		double q22 = HeightMapImporter::heightVal(data, heightRange, p.x + 1, p.y + 1);

		// Bilinear interpolation
		double h =
			q11 * (p.x+1 - x) * (p.y+1 - y)
			+ q21 * (x - p.x) * (p.y+1 - y)
			+ q12 * (p.x+1 - x) * (y - p.y)
			+ q22 * (x - p.x) * (y - p.y);

		h += center.y;

		return P.y - h;
	}

	glm::dvec3 HeightMapImporter::grad(std::shared_ptr<STBImageData> data, std::pair<double, double> heightRange, double pixelSize, glm::dvec3 center, glm::dvec3 P) {
		const double h = pixelSize / 4;

		const double x1 = HeightMapImporter::f(data, heightRange, pixelSize, center, {P.x - h, P.y, P.z});
		const double x2 = HeightMapImporter::f(data, heightRange, pixelSize, center, {P.x + h, P.y, P.z});
		const double z1 = HeightMapImporter::f(data, heightRange, pixelSize, center, {P.x, P.y, P.z - h});
		const double z2 = HeightMapImporter::f(data, heightRange, pixelSize, center, {P.x, P.y, P.z + h});

		return {(x2 - x1)/2/h, 1, (z2 - z1)/2/h};
	}

	double HeightMapImporter::heightVal(shared_ptr<STBImageData> data, pair<double, double> heightRange, int x, int y) {
		stbi_us color = data->operator[](clamp(x, 0, data->height()-1) * data->width() + clamp(y, 0, data->width()-1));
		return heightRange.first + color * (heightRange.second - heightRange.first) / 65536.0;
	}
}

