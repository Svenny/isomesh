/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
// Tests for uniform grid
#include <iostream>

#include <isomesh/util/material_selector.hpp>
#include <isomesh/util/zero_finder.hpp>
#include <isomesh/data/grid.hpp>

using std::cerr;
using std::clog;
using std::endl;

int main () {
	isomesh::SurfaceFunction f;
	isomesh::BisectionZeroFinder solver;
	// Plane
	f.f = [] (glm::dvec3 p) { return p.x + p.y + p.z - 0.6; };
	f.grad = [] (glm::dvec3) { return glm::dvec3 (1.0, 1.0, 1.0); };
	const int sz = 4;
	const int hsz = sz / 2;
	isomesh::UniformGrid G (sz);
	G.fill (f, solver, isomesh::TrivialMaterialSelector ());
	clog << "Grid layout map:" << endl;
	for (int32_t y = hsz; y >= -hsz; y--) {
		for (int32_t z = -hsz; z <= hsz; z++) {
			for (int32_t x = -hsz; x <= hsz; x++) {
				auto m = G.at (x, y, z);
				if (m == isomesh::Material::Empty)
					clog << '0';
				else clog << '1';
			}
			clog << endl;
		}
		clog << endl;
	}
	// Print surface-crossing edges found by grid
	clog << "X edges:" << endl;
	auto it1 = G.xEdgesBegin ();
	auto it2 = G.xEdgesEnd ();
	auto x_cnt = it2 - it1;
	while (it1 != it2) {
		auto lc = G.localToGlobal (it1.localCoords ());
		lc.x += it1->offset * G.scale ();
		clog << "pos " << lc.x << ' ' << lc.y << ' ' << lc.z << " normal ";
		clog << it1->normal.x << ' ' << it1->normal.y << ' ' << it1->normal.z << endl;
		++it1;
	}
	clog << endl << "Y edges:" << endl;
	it1 = G.yEdgesBegin ();
	it2 = G.yEdgesEnd ();
	auto y_cnt = it2 - it1;
	while (it1 != it2) {
		auto lc = G.localToGlobal (it1.localCoords ());
		lc.y += it1->offset * G.scale ();
		clog << "pos " << lc.x << ' ' << lc.y << ' ' << lc.z << " normal ";
		clog << it1->normal.x << ' ' << it1->normal.y << ' ' << it1->normal.z << endl;
		++it1;
	}
	clog << endl << "Z edges:" << endl;
	it1 = G.zEdgesBegin ();
	it2 = G.zEdgesEnd ();
	auto z_cnt = it2 - it1;
	while (it1 != it2) {
		auto lc = G.localToGlobal (it1.localCoords ());
		lc.z += it1->offset * G.scale ();
		clog << "pos " << lc.x << ' ' << lc.y << ' ' << lc.z << " normal ";
		clog << it1->normal.x << ' ' << it1->normal.y << ' ' << it1->normal.z << endl;
		++it1;
	}
	// Very simplified check, but it is enough to catch trivial bugs
	if (x_cnt != 16 || y_cnt != 16 || z_cnt != 16) {
		cerr << "Incorrect number of found surface-crossing edges!" << endl;
		return 1;
	}
	return 0;
}