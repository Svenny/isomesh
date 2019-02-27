/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <experimental/filesystem>
#include <memory>

#include "../common.hpp"

typedef unsigned char stbi_uc; // from stb_image

namespace isomesh {
	class HeightMapImporter {
	public:
		// Simple grayscale heightmap image with 3 bpp(black - height minimum, white - height maximum).
		HeightMapImporter(
			std::pair<double, double> height_range,
			glm::dvec3 center = glm::dvec3(0,0,0),
			double pixel_size = 1.0
		);
		~HeightMapImporter();


		// Getters, setters
		void setHeightRange(std::pair<double, double> range) {m_heightRange = range;};
		std::pair<double, double> heightRange() {return m_heightRange;};

		void setPixelSize(double pixelSize) {m_pixelSize = pixelSize;};
		double pixelSize() {return m_pixelSize;};

		void setCenter(glm::dvec3 center) {m_center = center;};
		glm::dvec3 center() {return m_center;};
		
		// Loads heightmap
		void loadGrayscale8bitMap(std::experimental::filesystem::path image_path);


		static SurfaceFunction buildSurfaceFunction(std::shared_ptr<HeightMapImporter> heightmap);
	private:
		// For surface function
		double f(glm::dvec3 P) const;
		glm::dvec3 grad(glm::dvec3 P) const;

		glm::dvec2 toMinPixCoords(glm::dvec3 P) const;
		glm::dvec2 toGlobalCoords(glm::dvec2 P) const;

		double heightVal(int x, int z) const;
	private:
		stbi_uc* m_data;
		int m_width, m_height, m_bpp;
		std::pair<double, double> m_heightRange;
		double m_pixelSize;
		glm::dvec3 m_center;
	};
}
