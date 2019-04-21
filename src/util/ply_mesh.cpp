/* This file is part of Isomesh library, released under MIT license.
   Copyright (c) 2019 Nikita Sirgienko (warquark@gmail.com) */

#include <isomesh/util/ply_mesh.hpp>
#include <fstream>
#include <iostream>
#include <memory>

using namespace std;

isomesh::Mesh* isomesh::ply2mesh(const std::string filename)
{
	PlyData data;
	data.load(filename);

	const size_t vcount = data.verticesCount();
	const size_t fcount = data.trianglesCount();

	std::vector<glm::vec3> normals(vcount, glm::vec3(0,0,0));

	for (size_t i = 0; i < fcount; i++) {
		glm::ivec3 indexs = data.triangleIndexs(i);
		glm::vec3 p1 = data.vertex(indexs.x);
		glm::vec3 p2 = data.vertex(indexs.y);
		glm::vec3 p3 = data.vertex(indexs.z);

		glm::vec3 normal = glm::cross(p2 - p3, p2 - p1);
		normals[indexs.x] += normal;
		normals[indexs.y] += normal;
		normals[indexs.z] += normal;
	}

	for (size_t i = 0; i < normals.size(); i++) {
		normals[i] = glm::normalize(normals[i]);
	}

	isomesh::Mesh* ptr = new isomesh::Mesh(vcount, fcount);

	for (size_t i = 0; i < vcount; i++) {
		// TODO: Stone? Is it normal usage of material argument?
		ptr->addVertex(data.vertex(i), normals[i], isomesh::Material::Stone);
	}

	for (size_t i = 0; i < fcount; i++) {
		glm::ivec3 idx = data.triangleIndexs(i);
		ptr->addTriangle(idx.x, idx.y, idx.z);
	}

	return ptr;
}
