/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
// Tests for QEF solver interface and its example implementation
#include <isomesh/qef/qef_solver_4d.hpp>

#include <iostream>

using std::cerr;
using std::clog;
using std::endl;

std::ostream &operator << (std::ostream &s, const glm::vec4 &v) {
	return s << '(' << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ')';
}

bool validate (isomesh::QefSolver4D &solver, glm::vec4 trueSolution) {
	glm::vec4 solution = solver.solve ({ 0, 0, 0, 0 }, { 1, 1, 1, 1 });
	clog << "  Point: " << solution << endl;
	clog << "  Error: " << solver.eval (solution) << endl;
	clog << "  True solution is " << trueSolution << endl;
	if (solution.x < 0 || solution.y < 0 || solution.z < 0 || solution.w < 0 ||
		 solution.x > 1 || solution.y > 1 || solution.z > 1 || solution.w > 1) {
		cerr << "Solver has breached constraints!" << endl;
		return false;
	}
	if (glm::distance (solution, trueSolution) > 0.001f) {
		cerr << "Solver is too far from true solution!" << endl;
		return false;
	}
	return true;
}

bool test1 (isomesh::QefSolver4D &solver) {
	glm::vec4 n1 (1, 0, 0, 0);
	glm::vec4 n2 (0, 0, 1, 0);
	glm::vec4 n3 (0, 1, 0, 0);
	glm::vec4 n4 (0, 0, 0, 1);
	solver.addPlane ({ 0, 1, 0, 0 }, n1);
	solver.addPlane ({ 1, 0, 0, 0 }, n2);
	solver.addPlane ({ 0, 0, 0, 1 }, n3);
	solver.addPlane ({ 0, 0, 1, 0 }, n4);
	clog << "Test 1:" << endl;
	return validate (solver, { 0, 0, 0, 0 });
}

bool test2 (isomesh::QefSolver4D &solver) {
	glm::vec2 base = glm::vec2 (0.01f, 1);
	glm::vec4 n1 (-base.x, base.y, -base.x, 0.0f);
	glm::vec4 n2 (base.x, base.y, -base.x, 0.0f);
	glm::vec4 n3 (-base.x, base.y, base.x, 0.0f);
	glm::vec4 n4 (base.x, base.y, base.x, 0.0f);
	solver.addPlane ({ 0, 0, 0, 0 }, glm::normalize (n1));
	solver.addPlane ({ 1, 0, 0, 0 }, glm::normalize (n2));
	solver.addPlane ({ 0, 0, 1, 0 }, glm::normalize (n3));
	solver.addPlane ({ 1, 0, 1, 0 }, glm::normalize (n4));
	clog << "Test 2:" << endl;
	return validate (solver, { 0.5f, 0.01f, 0.5f, 0.0f });
}

bool test3 (isomesh::QefSolver4D &solver) {
	glm::vec2 base = glm::vec2 (1.5f, 1);
	glm::vec4 n1 (-base.x, base.y, -base.x, 0.0f);
	glm::vec4 n2 (base.x, base.y, -base.x, 0.0f);
	glm::vec4 n3 (-base.x, base.y, base.x, 0.0f);
	glm::vec4 n4 (base.x, base.y, base.x, 0.0f);
	solver.addPlane ({ 0, 0, 0, 0 }, glm::normalize (n1));
	solver.addPlane ({ 1, 0, 0, 0 }, glm::normalize (n2));
	solver.addPlane ({ 0, 0, 1, 0 }, glm::normalize (n3));
	solver.addPlane ({ 1, 0, 1, 0 }, glm::normalize (n4));
	clog << "Test 3:" << endl;
	return validate (solver, { 0.5f, 1, 0.5f, 0.0f });
}

bool test4 (isomesh::QefSolver4D &solver) {
	glm::vec4 n1 (1, 0, 0, 0);
	glm::vec4 n2 (0, 1, 0, 0);
	glm::vec4 n3 (0, 0, 1, 0);
	glm::vec4 n4 (0, 0, 0, 1);
	glm::vec4 n5 (1, 1, 1, 1);
	solver.addPlane ({ 0, 0, 0, 0 }, glm::normalize (n1));
	solver.addPlane ({ 0, 0, 0, 0 }, glm::normalize (n2));
	solver.addPlane ({ 0, 0, 0, 0 }, glm::normalize (n3));
	solver.addPlane ({ 0, 0, 0, 0 }, glm::normalize (n4));
	solver.addPlane ({ 1, 1, 1, 1 }, glm::normalize (n5));
	clog << "Test 4:" << endl;
	return validate (solver, { 0.5f, 0.5f, 0.5f, 0.5f });
}

int main () {
	isomesh::QefSolver4D solver;
	if (!test1 (solver))
		return 1;
	// Broken reset will probably make second test fail
	solver.reset ();
	if (!test2 (solver))
		return 2;
	solver.reset ();
	if (!test3 (solver))
		return 3;
	solver.reset ();
	if (!test4 (solver))
		return 4;
	return 0;
}
