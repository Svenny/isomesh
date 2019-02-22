/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018 Pavel Asyutchenko (sventeam@yandex.ru) */
// Tests for QEF solver interface and its example implementation
#include <iostream>

#include <isomesh/qef/qef_solver_3d.hpp>

using std::cerr;
using std::clog;
using std::endl;

std::ostream &operator << (std::ostream &s, const glm::vec3 &v) {
	return s << '(' << v.x << ", " << v.y << ", " << v.z << ')';
}

bool validate (isomesh::QefSolver3D &solver, glm::vec3 trueSolution) {
	glm::vec3 solution = solver.solve ({ 0, 0, 0 }, { 1, 1, 1 });
	clog << "  Point: " << solution << endl;
	clog << "  Error: " << solver.eval (solution) << endl;
	clog << "  True solution is " << trueSolution << endl;
	if (solution.x < 0 || solution.y < 0 || solution.z < 0 ||
		 solution.x > 1 || solution.y > 1 || solution.z > 1) {
		cerr << "Solver has breached constraints!" << endl;
		return false;
	}
	if (glm::distance (solution, trueSolution) > 0.05f) {
		cerr << "Solver is too far from true solution!" << endl;
		return false;
	}
	return true;
}

bool test1 (isomesh::QefSolver3D &solver) {
	glm::vec3 n1 (1, 0, 0);
	glm::vec3 n2 (0, 0, 1);
	glm::vec3 n3 (0, 1, 0);
	solver.addPlane ({ 0, 1, 0 }, n1);
	solver.addPlane ({ 1, 0, 0 }, n2);
	solver.addPlane ({ 0, 0, 1 }, n3);
	clog << "Test 1:" << endl;
	return validate (solver, { 0, 0, 0 });
}

bool test2 (isomesh::QefSolver3D &solver) {
	glm::vec2 base = glm::vec2 (0.01f, 1);
	glm::vec3 n1 (-base.x, base.y, -base.x);
	glm::vec3 n2 (base.x, base.y, -base.x);
	glm::vec3 n3 (-base.x, base.y, base.x);
	glm::vec3 n4 (base.x, base.y, base.x);
	solver.addPlane ({ 0, 0, 0 }, glm::normalize (n1));
	solver.addPlane ({ 1, 0, 0 }, glm::normalize (n2));
	solver.addPlane ({ 0, 0, 1 }, glm::normalize (n3));
	solver.addPlane ({ 1, 0, 1 }, glm::normalize (n4));
	clog << "Test 2:" << endl;
	return validate (solver, { 0.5f, 0.01f, 0.5f });
}

bool test3 (isomesh::QefSolver3D &solver) {
	glm::vec2 base = glm::vec2 (1.5f, 1);
	glm::vec3 n1 (-base.x, base.y, -base.x);
	glm::vec3 n2 (base.x, base.y, -base.x);
	glm::vec3 n3 (-base.x, base.y, base.x);
	glm::vec3 n4 (base.x, base.y, base.x);
	solver.addPlane ({ 0, 0, 0 }, glm::normalize (n1));
	solver.addPlane ({ 1, 0, 0 }, glm::normalize (n2));
	solver.addPlane ({ 0, 0, 1 }, glm::normalize (n3));
	solver.addPlane ({ 1, 0, 1 }, glm::normalize (n4));
	clog << "Test 3:" << endl;
	return validate (solver, { 0.5f, 1, 0.5f });
}

int main () {
	isomesh::GradientDescentQefSolver3D solver;
	solver.setStepCount (25);
	if (!test1 (solver))
		return 1;
	// Broken reset will probably make second test fail
	solver.reset ();
	if (!test2 (solver))
		return 2;
	solver.reset ();
	if (!test3 (solver))
		return 3;
	return 0;
}
