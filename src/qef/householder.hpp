/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/common.hpp>

namespace isomesh
{

/* Transform NxM matrix to upper triangular form using Householder
 reflections. The process is known as QR decomposition, this routine
 computes R matrix and stores it in place of A.
 A is assumed to be column-major with N columns and M rows.
 used_rows should not exceed M.
*/
template<typename T, int N, int M>
void householder (T A[N][M], int used_rows) {
	T v[M];
	// Zero out all subdiagonal elements in i-th column
	for (int i = 0; i < N; i++) {
		// Sum of squares of i-th subcolumn
		T norm { 0 };
		for (int j = i; j < used_rows; j++)
			norm += A[i][j] * A[i][j];
		// Make v - reflection vector
		memset (v, 0, sizeof (v));
		T invgamma { 0 };
		if (norm < std::numeric_limits<T>::min ()) {
			v[i] = T (1);
			invgamma = T (2);
		}
		else {
			T mult = T (1) / glm::sqrt (norm);
			for (int j = i; j < used_rows; j++)
				v[j] = A[i][j] * mult;
			if (v[i] >= T (0))
				v[i] += T (1);
			else v[i] -= T (1);
			invgamma = T (1) / glm::abs (v[i]);
		}
		// For each column do A[j] -= v * dot (A[j], v) / gamma
		for (int j = i; j < 4; j++) {
			T mult { 0 };
			for (int k = i; k < used_rows; k++)
				mult += A[j][k] * v[k];
			mult *= invgamma;
			for (int k = i; k < used_rows; k++)
				A[j][k] -= mult * v[k];
		}
	}
}

// Original code - remove when code above is properly tested
/*void QrQefSolver3D::compressMatrix () {
	float v[kRows];
	for (int i = 0; i < 4; i++) {
		// norm of i-th subcolumn
		float norm = 0;
		for (int j = i; j < kRows; j++)
			norm += A[i][j] * A[i][j];
		memset (v, 0, sizeof (v));
		float invgamma = 0;
		if (norm < std::numeric_limits<float>::min ()) {
			v[i] = 1.0f;
			invgamma = 2.0f;
		}
		else {
			float mult = 1.0f / glm::sqrt (norm);
			for (int j = i; j < kRows; j++)
				v[j] = A[i][j] * mult;
			if (v[i] >= 0)
				v[i] += 1.0f;
			else
				v[i] -= 1.0f;
			invgamma = 1.0f / glm::abs (v[i]);
		}
		for (int j = i; j < 4; j++) {
			float dot = 0;
			for (int k = i; k < kRows; k++)
				dot += A[j][k] * v[k];
			for (int k = 0; k < kRows; k++)
				A[j][k] -= dot * invgamma * v[k];
		}
	}
	m_usedRows = 4;
}*/

}
