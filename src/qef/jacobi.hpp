/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/common.hpp>

namespace isomesh
{

namespace jacobi_detail
{

// Faster but less accurate (Jacobi does not fully converge with this)
template<typename T>
std::pair<T, T> calcSinCosFast (T tan_twophi) {
	T inv = T (0.5) / hypot (T (1), tan_twophi);
	T c = sqrt (T (0.5) + inv);
	T s = copysign (sqrt (T (0.5) - inv), tan_twophi);
	return { s, c };
}

// More accurate but much slower
template<typename T>
std::pair<T, T> calcSinCosAccurate (T tan_twophi) {
	T phi = T (0.5) * atan (tan_twophi);
	return { sin (phi), cos (phi) };
}

template<typename T>
void rotate (glm::mat<3, 3, T> &A, glm::mat<3, 3, T> &H, T c, T s, int i, int j) {
#define jacobiRotate(i, j, k) { \
	float aik = A[glm::min (i, k)][glm::max (i, k)]; \
	float ajk = A[glm::min (j, k)][glm::max (j, k)]; \
	A[glm::min (i, k)][glm::max (i, k)] = c * aik + s * ajk; \
	A[glm::min (j, k)][glm::max (j, k)] = -s * aik + c * ajk; \
}
	if (i == 0 && j == 1) {
		jacobiRotate (0, 1, 2);
	}
	else if (i == 0 && j == 2) {
		jacobiRotate (0, 2, 1);
	}
	else { // (i == 1 && j == 2), other cases are impossible
		jacobiRotate (1, 2, 0);
	}
#undef jacobiRotate
	float aii = A[i][i];
	float ajj = A[j][j];
	float aij = A[i][j];
	A[i][i] = c * c * aii + s * s * ajj + c * s * (aij + aij);
	A[j][j] = s * s * aii + c * c * ajj - c * s * (aij + aij);
	A[i][j] = 0;
	for (int k = 0; k < 3; k++) {
		float hik = H[i][k];
		float hjk = H[j][k];
		H[i][k] = c * hik + s * hjk;
		H[j][k] = -s * hik + c * hjk;
	}
}

template<typename T>
void rotate (glm::mat<4, 4, T> &A, glm::mat<4, 4, T> &H, T c, T s, int i, int j) {
#define jacobiRotate(i, j, k) { \
	float aik = A[glm::min (i, k)][glm::max (i, k)]; \
	float ajk = A[glm::min (j, k)][glm::max (j, k)]; \
	A[glm::min (i, k)][glm::max (i, k)] = c * aik + s * ajk; \
	A[glm::min (j, k)][glm::max (j, k)] = -s * aik + c * ajk; \
}
	if (i == 0 && j == 1) {
		jacobiRotate (0, 1, 2);
		jacobiRotate (0, 1, 3);
	}
	else if (i == 0 && j == 2) {
		jacobiRotate (0, 2, 1);
		jacobiRotate (0, 2, 3);
	}
	else if (i == 0 && j == 3) {
		jacobiRotate (0, 3, 1);
		jacobiRotate (0, 3, 2);
	}
	else if (i == 1 && j == 2) {
		jacobiRotate (1, 2, 0);
		jacobiRotate (1, 2, 3);
	}
	else if (i == 1 && j == 3) {
		jacobiRotate (1, 3, 0);
		jacobiRotate (1, 3, 2);
	}
	else { // (i == 2 && j == 3), other cases are impossible
		jacobiRotate (2, 3, 0);
		jacobiRotate (2, 3, 1);
	}
#undef jacobiRotate
	float aii = A[i][i];
	float ajj = A[j][j];
	float aij = A[i][j];
	A[i][i] = c * c * aii + s * s * ajj + c * s * (aij + aij);
	A[j][j] = s * s * aii + c * c * ajj - c * s * (aij + aij);
	A[i][j] = 0;
	for (int k = 0; k < 4; k++) {
		float hik = H[i][k];
		float hjk = H[j][k];
		H[i][k] = c * hik + s * hjk;
		H[j][k] = -s * hik + c * hjk;
	}
}

}

template<typename T, int D>
std::pair<glm::vec<D, T>, glm::mat<D, D, T> > jacobi (glm::mat<D, D, T> A, T tolerance,
                                                      int max_iters, bool use_fast_sincos) {
	using namespace jacobi_detail;
	glm::mat<D, D, T> E { T (1) };
	for (int k = 0; k < max_iters; k++) {
		T max_el = abs (A[0][1]);
		int max_i = 0;
		int max_j = 1;
		for (int i = 0; i < D; i++) {
			for (int j = i + 1; j < D; j++) {
				T el = abs (A[i][j]);
				if (el > max_el) {
					max_el = el;
					max_i = i;
					max_j = j;
				}
			}
		}
		if (max_el <= tolerance)
			break;
		int i = max_i;
		int j = max_j;
		T tan_twophi = (A[i][j] + A[i][j]) / (A[i][i] - A[j][j]);
		T s, c;
		if (use_fast_sincos)
			std::tie (s, c) = calcSinCosFast (tan_twophi);
		else
			std::tie (s, c) = calcSinCosAccurate (tan_twophi);
		rotate<T> (A, E, c, s, i, j);
	}
	glm::vec<D, T> e;
	for (int i = 0; i < D; i++)
		e[i] = A[i][i];
	return { e, E };
}

}
