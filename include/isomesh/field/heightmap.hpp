/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <memory>
#include <string>

#include <isomesh/field/scalar_field.hpp>

class STBImageData;

namespace isomesh {
	class Heightmap: public ScalarField {
	public:
		Heightmap(
			std::pair<double, double> height_range,
			glm::dvec3 center = glm::dvec3(0,0,0),
			double pixel_size = 1.0
		);
		~Heightmap();

		// ScalarField overridings
		double value(double x, double y, double z) const noexcept override;
		glm::dvec3 grad(double x, double y, double z) const noexcept override;

		// getters, setters
		void setHeightRange(std::pair<double, double> range) noexcept;
		std::pair<double, double> heightRange()  const noexcept;
		void setPixelSize(double pixelSize) noexcept;
		double pixelSize() const noexcept;
		void setCenter(glm::dvec3 center) noexcept;
		glm::dvec3 center() const noexcept;

		/**
		@brief Load grayscale heightmap with 8-bit or 16-bit on color channel
		@param filename path to image file
		*/
		void loadGrayscaleMap(std::string filename);

		/// Checks, if heightmap data loaded
		bool isDataLoaded() const noexcept;

	private:
		double heightVal(int x, int z) const;

	private:
		std::shared_ptr<STBImageData> m_data;
		std::pair<double, double> m_heightRange;
		double m_pixelSize;
		glm::dvec3 m_center;
	};
}
