/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <memory>

#include "../common.hpp"

class STBImageData;

namespace isomesh {
	class HeightMapImporter {
	public:
		HeightMapImporter(
			std::pair<double, double> height_range,
			glm::dvec3 center = glm::dvec3(0,0,0),
			double pixel_size = 1.0
		);
		~HeightMapImporter();

		// Getters, setters
		void setHeightRange(std::pair<double, double> range) noexcept {m_heightRange = range;};
		std::pair<double, double> heightRange()  const noexcept {return m_heightRange;};

		void setPixelSize(double pixelSize) noexcept {m_pixelSize = pixelSize;};
		double pixelSize() const noexcept {return m_pixelSize;};

		void setCenter(glm::dvec3 center) noexcept {m_center = center;};
		glm::dvec3 center() const noexcept {return m_center;};

		// Load grayscale heightmap with 8-bit or 16-bit on color channel
		void loadGrayscaleMap(std::string filename);

		bool isDataLoaded() const noexcept {return m_data != nullptr;};

		SurfaceFunction buildSurfaceFunction() const;
	private:
		static double f(std::shared_ptr<STBImageData> data, std::pair<double, double> heightRange, double pixelSize,
		glm::dvec3 center, glm::dvec3 P);
		static glm::dvec3 grad(std::shared_ptr<STBImageData> data, std::pair<double, double> heightRange, double pixelSize,
		glm::dvec3 center, glm::dvec3 P);
		static double heightVal(std::shared_ptr<STBImageData> data, std::pair<double, double> heightRange, int x, int z);

	private:
		std::shared_ptr<STBImageData> m_data;
		std::pair<double, double> m_heightRange;
		double m_pixelSize;
		glm::dvec3 m_center;
	};
}
