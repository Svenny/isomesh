/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
// Tests for uniform grid
#include <isomesh/util/zero_finder.hpp>
#include <isomesh/data/grid.hpp>

#include <iostream>

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
void printEdges (const UniformGrid &G) {
	for (const auto &edge : G.edges<D> ()) {
		// Position of surface-crossing point on the edge
		glm::dvec3 pos_local = edge.surfacePoint ();
		glm::dvec3 pos_global = G.localToGlobal (pos_local);
		clog << "pos " << pos_global.x << ' ' << pos_global.y << ' ' << pos_global.z;
		glm::vec3 normal = edge.surfaceNormal ();
		clog << " normal " << normal.x << ' ' << normal.y << ' ' << normal.z << endl;
	}
}

// Check edge op
template<int D>
bool validateEdges (const UniformGrid &G, const isomesh::ScalarField &f) {
	// Use a very precise solver to find actual roots
	isomesh::BisectionZeroFinder solver (50);
	for (const auto &edge : G.edges<D> ()) {
		glm::dvec3 p0 = edge.lesserEndpoint ();
		glm::dvec3 p1 = edge.biggerEndpoint ();
		// Surface-crossing point
		glm::dvec3 p = edge.surfacePoint ();

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

class PlaneScalarField : public isomesh::ScalarField {
public:
	virtual double value (double x, double y, double z) const noexcept override {
		return x + y + z - 0.6;
	}
	virtual glm::dvec3 grad (double x, double y, double z) const noexcept override {
		return glm::dvec3 (1, 1, 1);
	}
};

int main () {
	/* Code below will create a grid and fill it using a simple
	 plane function. The grid then may be checked for correctness,
	 as the expected grid structure is known in advance. */
	isomesh::BisectionZeroFinder solver;
	PlaneScalarField F;
	// Create and fill uniform grid
	const int sz = 4;
	UniformGrid G (sz);
	G.fill (F, solver);

	clog << "Grid layout map:" << endl;
	printGridLayout (G);

	// Print surface-crossing edges found by grid
	bool fail = false;

	clog << "X edges:" << endl;
	auto x_cnt = G.edges<0> ().size ();
	printEdges<0> (G);
	if (!validateEdges<0> (G, F))
		fail = true;

	clog << endl << "Y edges:" << endl;
	auto y_cnt = G.edges<1> ().size ();
	printEdges<1> (G);
	if (!validateEdges<1> (G, F))
		fail = true;

	clog << endl << "Z edges:" << endl;
	auto z_cnt = G.edges<2> ().size ();
	printEdges<2> (G);
	if (!validateEdges<2> (G, F))
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
