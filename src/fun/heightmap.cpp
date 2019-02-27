/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include <isomesh/fun/heightmap.hpp>

#include <cmath>
#include <algorithm>
#include <exception>
#include <experimental/filesystem>

#define STB_IMAGE_IMPLEMENTATION
#include "../3dparty/stb_image.h"

using namespace std;
using namespace std::experimental::filesystem;

namespace isomesh {
	HeightMapImporter::HeightMapImporter(pair<double, double> height_range, glm::dvec3 center, double pixel_size):
		m_heightRange(height_range),
		m_center(center),
		m_pixelSize(pixel_size)
	{
	}

	HeightMapImporter::~HeightMapImporter() {
		if (m_data) {
			stbi_image_free(m_data);
			m_data = nullptr;
		}
	}

	SurfaceFunction HeightMapImporter::buildSurfaceFunction(shared_ptr<HeightMapImporter> heightmap) {
		return SurfaceFunction{bind(&HeightMapImporter::f, heightmap, placeholders::_1), bind(&HeightMapImporter::grad, heightmap, placeholders::_1)};
	}

	void HeightMapImporter::loadGrayscale8bitMap(string image_filename){
		path image_path(image_filename);
		if (m_data) {
			stbi_image_free(m_data);
			m_data = nullptr;
		}

		if (exists(image_path)) {
			m_data = stbi_load(image_path.string().c_str(), &m_width, &m_height, &m_bpp, 1);
			if (!m_data)
				throw std::runtime_error("file " + image_path.string() + " not image file");
			else if (m_bpp != 1)
				throw std::runtime_error("file " + image_path.string() + " not 8bit grayscale image file");
		}
		else {
			m_data = nullptr;
			throw std::runtime_error("file " + image_path.string() + " not found");
		}
	}

	double isomesh::HeightMapImporter::f(glm::dvec3 P) const {
		glm::dvec2 p(toMinPixCoords(P));
		double x = (P.x - m_center.x) / m_pixelSize + m_width/2;
		double y = (P.z - m_center.z) / m_pixelSize + m_height/2;
		
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
		
		h += m_center.y;
		
		return P.y - h;
	}
	
	glm::dvec3 isomesh::HeightMapImporter::grad(glm::dvec3 P) const {
		const double h = m_pixelSize / 4;
		double dx = (f({P.x + h, P.y, P.z}) - f({P.x - h, P.y, P.z})) / 2 / h;
		double dz = (f({P.x, P.y, P.z + h}) - f({P.x, P.y, P.z - h})) / 2 / h;
		return{dx,1,dz};
	}
	
	glm::dvec2 isomesh::HeightMapImporter::toMinPixCoords(glm::dvec3 P) const {
		return glm::dvec2(floor((P.x - m_center.x) / m_pixelSize) + m_width/2, floor((P.z - m_center.z) / m_pixelSize) + m_height/2);
	}
	
	double isomesh::HeightMapImporter::heightVal(int x, int y) const {
		stbi_uc color = m_data[clamp(x, 0, m_height-1) * m_width + clamp(y, 0, m_width-1)];
		return m_heightRange.first + color * (m_heightRange.second - m_heightRange.first) / 256.0;
	}
}
