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

using isomesh::UniformGrid;
using edge_iter_t = isomesh::UniformGridEdgeStorage::const_iterator;

// Print grid signs layer by layer (along Y)
void printGridLayout (const UniformGrid &G) {
	const int32_t hsz = int32_t (G.gridSize () / 2);
	for (int32_t y = hsz; y >= -hsz; y--) { // From top to bottom
		for (int32_t z = -hsz; z <= hsz; z++) {
			for (int32_t x = -hsz; x <= hsz; x++) {
				auto mat = G.at (x, y, z);
				if (mat == isomesh::Material::Empty)
					clog << '0';
				else clog << '1';
			}
			clog << endl;
		}
		clog << endl;
	}
}

template<int D>
void printEdges (const UniformGrid &G, edge_iter_t iter, edge_iter_t last) {
	for (; iter != last; ++iter) {
		// Position of surface-crossing point on the edge
		glm::dvec3 pos_local = iter.localCoords ();
		std::get<D> (pos_local) += iter->offset;
		glm::dvec3 pos_global = G.localToGlobal (pos_local);
		clog << "pos " << pos_global.x << ' ' << pos_global.y << ' ' << pos_global.z;
		clog << " normal " << iter->normal.x << ' ' << iter->normal.y << ' ' << iter->normal.z << endl;
	}
}

// Check edge op
template<int D>
bool validateEdges (const UniformGrid &G, const isomesh::SurfaceFunction &f,
                    edge_iter_t iter, edge_iter_t last) {
	// Use a very precise solver to find actual roots
	isomesh::BisectionZeroFinder solver (50);
	for (; iter != last; ++iter) {
		// Lower-coordinate edge endpoint
		glm::dvec3 p0 = iter.localCoords ();
		// Higher-coordinate edge endpoint
		glm::dvec3 p1 = p0;
		std::get<D> (p1) += 1;
		// Surface-crossing point
		glm::dvec3 p = p0;
		std::get<D> (p) += iter->offset;

		p0 = G.localToGlobal (p0);
		p1 = G.localToGlobal (p1);
		p = G.localToGlobal (p);

		double f0 = f (p0);
		bool sign0 = (f0 <= 0.0);
		double f1 = f (p1);
		bool sign1 = (f1 <= 0.0);
		if (sign0 == sign1) {
			cerr << "Edge " << p.x << ' ' << p.y << ' ' << p.z << " failed the validation: ";
			cerr << "signs on endpoints are not different!" << endl;
			return false;
		}
		/* Check that found zero crossing point is near the actual root. This check may
		 look like a zero finder test, but in fact it did find a bug in grid code!
		 This is plane, so any edge will have exactly one zero crossing point, we
		 don't need to care about multiple roots failing this check. */
		double true_root;
		if (D == 0)
			true_root = solver.findAlongX (p0.x, p0.y, p0.z, p1.x, f0, f1, f);
		else if (D == 1)
			true_root = solver.findAlongY (p0.x, p0.y, p0.z, p1.y, f0, f1, f);
		else true_root = solver.findAlongZ (p0.x, p0.y, p0.z, p1.z, f0, f1, f);

		double edge_root = std::get<D> (p);
		if (fabs (true_root - edge_root) > 1e-2) {
			cerr << "Edge " << p.x << ' ' << p.y << ' ' << p.z << " failed the validation: ";
			cerr << "zero finder didn't converge to actual zero!" << endl;
			return false;
		}
	}
	return true;
}

int main () {
	/* Code below will create a grid and fill it using a simple
	 plane function. The grid then may be checked for correctness,
	 as the expected grid structure is known in advance. */
	isomesh::BisectionZeroFinder solver;
	// This function defines a plane
	isomesh::SurfaceFunction f;
	f.f = [] (glm::dvec3 p) { return p.x + p.y + p.z - 0.6; };
	f.grad = [] (glm::dvec3) { return glm::dvec3 (1.0, 1.0, 1.0); };
	// Create and fill uniform grid
	const int sz = 4;
	UniformGrid G (sz);
	G.fill (f, solver, isomesh::TrivialMaterialSelector ());

	clog << "Grid layout map:" << endl;
	printGridLayout (G);

	// Print surface-crossing edges found by grid
	bool fail = false;

	clog << "X edges:" << endl;
	auto it1 = G.xEdgesBegin ();
	auto it2 = G.xEdgesEnd ();
	auto x_cnt = it2 - it1;
	printEdges<0> (G, it1, it2);
	if (!validateEdges<0> (G, f, it1, it2))
		fail = true;

	clog << endl << "Y edges:" << endl;
	it1 = G.yEdgesBegin ();
	it2 = G.yEdgesEnd ();
	auto y_cnt = it2 - it1;
	printEdges<1> (G, it1, it2);
	if (!validateEdges<1> (G, f, it1, it2))
		fail = true;

	clog << endl << "Z edges:" << endl;
	it1 = G.zEdgesBegin ();
	it2 = G.zEdgesEnd ();
	auto z_cnt = it2 - it1;
	printEdges<2> (G, it1, it2);
	if (!validateEdges<2> (G, f, it1, it2))
		fail = true;

	// Very rough check, but this is enough to catch trivial bugs
	if (x_cnt != 16 || y_cnt != 16 || z_cnt != 16) {
		cerr << "Incorrect number of found surface-crossing edges!" << endl;
		return 1;
	}
	if (fail) {
		cerr << "Edges validation failed!" << endl;
		return 2;
	}

	return 0;
}
