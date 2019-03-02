/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include "stbi_data.hpp"

#include <experimental/filesystem>

#define STB_IMAGE_IMPLEMENTATION
#include "../3dparty/stb_image.h"

using namespace std;
using namespace std::experimental::filesystem;

STBImageData::STBImageData(std::string filename): m_data(nullptr) {
	path image_path(filename);

	if (exists(image_path)) {
		m_data = stbi_load_16(image_path.string().c_str(), &m_width, &m_height, &m_bpp, 1);
		if (!m_data)
			throw std::runtime_error("file " + image_path.string() + " not image file");
	} else {
		throw std::runtime_error("file " + image_path.string() + " not found");
	}
}

STBImageData::~STBImageData() {
	stbi_image_free(m_data);
}

stbi_us STBImageData::operator[](int idx) const {
	return m_data[idx];
}
