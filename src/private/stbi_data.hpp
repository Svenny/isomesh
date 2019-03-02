/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */
#pragma once

#include <string>
#include <memory>

typedef unsigned short stbi_us; // from stb_image

class STBImageData {
public:
	STBImageData(std::string filename);
	~STBImageData();

	stbi_us operator[](int idx) const;

	int width() {return m_width;}
	int height() {return m_height;}
	int bpp() {return m_bpp;}
private:
	stbi_us* m_data;
	int m_width, m_height, m_bpp;
};
