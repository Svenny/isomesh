/* This file is part of Isomesh library, released under MIT license.
  Copyright (c) 2018-2019 Pavel Asyutchenko (sventeam@yandex.ru) */
#include <isomesh/common.hpp>

#include <cstring>

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
		for (int j = i; j < N; j++) {
			T mult { 0 };
			for (int k = i; k < used_rows; k++)
				mult += A[j][k] * v[k];
			mult *= invgamma;
			for (int k = i; k < used_rows; k++)
				A[j][k] -= mult * v[k];
		}
	}
}

}
