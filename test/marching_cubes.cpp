/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
// Tests for marching cubes algorithm
#include <iostream>
#include <fstream>

#include <isomesh/util/material_selector.hpp>
#include <isomesh/util/zero_finder.hpp>
#include <isomesh/algo/marching_cubes.hpp>

using std::cerr;
using std::clog;
using std::endl;

int main () {
	isomesh::SurfaceFunction f;
	isomesh::BisectionZeroFinder solver;
	// Waves (or hills?)
	{
		const double freq_x = 0.3;
		const double freq_z1 = 0.4;
		const double freq_z2 = 0.2;
		f.f = [=](glm::dvec3 p) {
			return sin (freq_x * p.x) + p.y - sin (freq_z1 * p.z) * cos (freq_z2 * p.z);
		};
		f.grad = [=](glm::dvec3 p) {
			double dx = freq_x * cos (freq_x * p.x);
			double dy = 1;
			double dz = freq_z2 * sin (freq_z1 * p.z) * sin (freq_z2 * p.z)
			          - freq_z1 * cos (freq_z1 * p.z) * cos (freq_z2 * p.z);
			return glm::dvec3 (dx, dy, dz);
		};
	}
	// Generate a surface chunk from this function,
	// then check it against a manually validated result
	const int sz = 16;
	isomesh::UniformGrid G (sz);
	G.fill (f, solver, isomesh::TrivialMaterialSelector ());
	// Separate grid-related bugs from algorithm-related
	auto edge_count = G.edges<0> ().size ();
	if (edge_count != 39) {
		cerr << "Wrong uniform grid! X edges counts mismatch, "
		     << edge_count << " found, 39 expected" << endl;
		return 1;
	}
	edge_count = G.edges<1> ().size ();
	if (edge_count != 289) {
		cerr << "Wrong uniform grid! Y edges counts mismatch, "
		     << edge_count << " found, 289 expected" << endl;
		return 1;
	}
	edge_count = G.edges<2> ().size ();
	if (edge_count != 45) {
		cerr << "Wrong uniform grid! Z edges counts mismatch, "
		     << edge_count << " found, 45 expected" << endl;
		return 1;
	}
	auto mesh = isomesh::marchingCubes (G);
	// TODO: add testing for vertices & normals placement
	// For now do a simple sanity check
	if (mesh.vertexCount () != 373 || mesh.indexCount () != 2010) {
		cerr << "Marching cubes result differs from expected!" << endl;
		cerr << "Expected 373 vertices and 2010 indices" << endl;
		cerr << "Got " << mesh.vertexCount () << " vertices and "
		     << mesh.indexCount () << " indices" << endl;
		return 2;
	}

	return 0;
}
