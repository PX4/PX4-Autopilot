/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file matrix_alg.cpp
 *
 * Matrix algebra on raw arrays
 */

#include "matrix_alg.h"
#include <px4_platform_common/defines.h>

/*
 *    Does matrix multiplication of two regular/square matrices
 *
 *    @param     A,           Matrix A
 *    @param     B,           Matrix B
 *    @param     n,           dimemsion of square matrices
 *    @returns                multiplied matrix i.e. A*B
 */

float *mat_mul(float *A, float *B, uint8_t n)
{
	float *ret = new float[n * n];
	memset(ret, 0.0f, n * n * sizeof(float));

	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
			for (uint8_t k = 0; k < n; k++) {
				ret[i * n + j] += A[i * n + k] * B[k * n + j];
			}
		}
	}

	return ret;
}

static inline void swap(float &a, float &b)
{
	float c;
	c = a;
	a = b;
	b = c;
}

/*
 *    calculates pivot matrix such that all the larger elements in the row are on diagonal
 *
 *    @param     A,           input matrix matrix
 *    @param     pivot
 *    @param     n,           dimenstion of square matrix
 *    @returns                false = matrix is Singular or non positive definite, true = matrix inversion successful
 */

static void mat_pivot(float *A, float *pivot, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
			pivot[i * n + j] = (i == j);
		}
	}

	for (uint8_t i = 0; i < n; i++) {
		uint8_t max_j = i;

		for (uint8_t j = i; j < n; j++) {
			if (fabsf(A[j * n + i]) > fabsf(A[max_j * n + i])) {
				max_j = j;
			}
		}

		if (max_j != i) {
			for (uint8_t k = 0; k < n; k++) {
				swap(pivot[i * n + k], pivot[max_j * n + k]);
			}
		}
	}
}

/*
 *    calculates matrix inverse of Lower trangular matrix using forward substitution
 *
 *    @param     L,           lower triangular matrix
 *    @param     out,         Output inverted lower triangular matrix
 *    @param     n,           dimension of matrix
 */

static void mat_forward_sub(float *L, float *out, uint8_t n)
{
	// Forward substitution solve LY = I
	for (int i = 0; i < n; i++) {
		out[i * n + i] = 1 / L[i * n + i];

		for (int j = i + 1; j < n; j++) {
			for (int k = i; k < j; k++) {
				out[j * n + i] -= L[j * n + k] * out[k * n + i];
			}

			out[j * n + i] /= L[j * n + j];
		}
	}
}

/*
 *    calculates matrix inverse of Upper trangular matrix using backward substitution
 *
 *    @param     U,           upper triangular matrix
 *    @param     out,         Output inverted upper triangular matrix
 *    @param     n,           dimension of matrix
 */

static void mat_back_sub(float *U, float *out, uint8_t n)
{
	// Backward Substitution solve UY = I
	for (int i = n - 1; i >= 0; i--) {
		out[i * n + i] = 1 / U[i * n + i];

		for (int j = i - 1; j >= 0; j--) {
			for (int k = i; k > j; k--) {
				out[j * n + i] -= U[j * n + k] * out[k * n + i];
			}

			out[j * n + i] /= U[j * n + j];
		}
	}
}

/*
 *    Decomposes square matrix into Lower and Upper triangular matrices such that
 *    A*P = L*U, where P is the pivot matrix
 *    ref: http://rosettacode.org/wiki/LU_decomposition
 *    @param     U,           upper triangular matrix
 *    @param     out,         Output inverted upper triangular matrix
 *    @param     n,           dimension of matrix
 */

static void mat_LU_decompose(float *A, float *L, float *U, float *P, uint8_t n)
{
	memset(L, 0, n * n * sizeof(float));
	memset(U, 0, n * n * sizeof(float));
	memset(P, 0, n * n * sizeof(float));
	mat_pivot(A, P, n);

	float *APrime = mat_mul(P, A, n);

	for (uint8_t i = 0; i < n; i++) {
		L[i * n + i] = 1;
	}

	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
			if (j <= i) {
				U[j * n + i] = APrime[j * n + i];

				for (uint8_t k = 0; k < j; k++) {
					U[j * n + i] -= L[j * n + k] * U[k * n + i];
				}
			}

			if (j >= i) {
				L[j * n + i] = APrime[j * n + i];

				for (uint8_t k = 0; k < i; k++) {
					L[j * n + i] -= L[j * n + k] * U[k * n + i];
				}

				L[j * n + i] /= U[i * n + i];
			}
		}
	}

	delete[] APrime;
}

/*
 *    matrix inverse code for any square matrix using LU decomposition
 *    inv = inv(U)*inv(L)*P, where L and U are triagular matrices and P the pivot matrix
 *    ref: http://www.cl.cam.ac.uk/teaching/1314/NumMethods/supporting/mcmaster-kiruba-ludecomp.pdf
 *    @param     m,           input 4x4 matrix
 *    @param     inv,      Output inverted 4x4 matrix
 *    @param     n,           dimension of square matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */
bool mat_inverse(float *A, float *inv, uint8_t n)
{
	float *L, *U, *P;
	bool ret = true;
	L = new float[n * n];
	U = new float[n * n];
	P = new float[n * n];
	mat_LU_decompose(A, L, U, P, n);

	float *L_inv = new float[n * n];
	float *U_inv = new float[n * n];

	memset(L_inv, 0, n * n * sizeof(float));
	mat_forward_sub(L, L_inv, n);

	memset(U_inv, 0, n * n * sizeof(float));
	mat_back_sub(U, U_inv, n);

	// decomposed matrices no longer required
	delete[] L;
	delete[] U;

	float *inv_unpivoted = mat_mul(U_inv, L_inv, n);
	float *inv_pivoted = mat_mul(inv_unpivoted, P, n);

	//check sanity of results
	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
			if (!PX4_ISFINITE(inv_pivoted[i * n + j])) {
				ret = false;
			}
		}
	}

	memcpy(inv, inv_pivoted, n * n * sizeof(float));

	//free memory
	delete[] inv_pivoted;
	delete[] inv_unpivoted;
	delete[] P;
	delete[] U_inv;
	delete[] L_inv;
	return ret;
}
