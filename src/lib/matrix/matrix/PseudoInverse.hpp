/**
 * @file PseudoInverse.hpp
 *
 * Implementation of matrix pseudo inverse
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 * @author Julian Kent <julian@auterion.com>
 */

#pragma once

#include "SquareMatrix.hpp"
#include "Vector.hpp"

namespace matrix
{

/**
 * Geninv
 * Fast pseudoinverse based on full rank cholesky factorisation
 *
 * Courrieu, P. (2008). Fast Computation of Moore-Penrose Inverse Matrices, 8(2), 25–29. http://arxiv.org/abs/0804.4809
 */
template<typename Type, size_t M, size_t N>
bool geninv(const Matrix<Type, M, N> &G, Matrix<Type, N, M> &res)
{
	size_t rank;

	if (M <= N) {
		SquareMatrix<Type, M> A = G * G.transpose();
		SquareMatrix<Type, M> L = fullRankCholesky(A, rank);

		A = L.transpose() * L;
		SquareMatrix<Type, M> X;

		if (!inv(A, X, rank)) {
			res = Matrix<Type, N, M>();
			return false; // LCOV_EXCL_LINE -- this can only be hit from numerical issues
		}

		// doing an intermediate assignment reduces stack usage
		A = X * X * L.transpose();
		res = G.transpose() * (L * A);

	} else {
		SquareMatrix<Type, N> A = G.transpose() * G;
		SquareMatrix<Type, N> L = fullRankCholesky(A, rank);

		A = L.transpose() * L;
		SquareMatrix<Type, N> X;

		if (!inv(A, X, rank)) {
			res = Matrix<Type, N, M>();
			return false; // LCOV_EXCL_LINE -- this can only be hit from numerical issues
		}

		// doing an intermediate assignment reduces stack usage
		A = X * X * L.transpose();
		res = (L * A) * G.transpose();
	}

	return true;
}

/**
 * Pseudo-inverse of a wide float matrix (M <= N) with only the M x M kernel in double.
 *
 * geninv inverts G * G', whose condition number is the square of G's: for cond(G) ~ 1e3 single
 * precision is lost entirely. Only the kernel needs double. G and the result stay float, every
 * product touching them is an explicit loop accumulated in double, and the M x M work matrices
 * are reused in place, so the stack holds three M x M doubles instead of the N x M double copies
 * a full double geninv needs (control allocation runs this on a 3 KB work queue stack).
 *
 * @param G wide input matrix
 * @param res pseudo-inverse of G; untouched if the kernel could not be inverted
 * @return false if the kernel could not be inverted
 */
template<size_t M, size_t N>
bool geninvMixedPrecision(const Matrix<float, M, N> &G, Matrix<float, N, M> &res)
{
	static_assert(M <= N, "geninvMixedPrecision requires a wide matrix (M <= N)");

	// A = G * G'
	SquareMatrix<double, M> A;

	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j <= i; j++) {
			double sum = 0.;

			for (size_t k = 0; k < N; k++) {
				sum += static_cast<double>(G(i, k)) * static_cast<double>(G(j, k));
			}

			A(i, j) = sum;
			A(j, i) = sum;
		}
	}

	size_t rank;
	SquareMatrix<double, M> L = fullRankCholesky(A, rank);

	// A = L' * L
	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j <= i; j++) {
			double sum = 0.;

			for (size_t k = 0; k < M; k++) {
				sum += L(k, i) * L(k, j);
			}

			A(i, j) = sum;
			A(j, i) = sum;
		}
	}

	SquareMatrix<double, M> X;

	if (!inv(A, X, rank)) {
		return false;
	}

	// A = L * X * X * L' = (G * G')^+, computed as A = X * X, X = L * A, A = X * L'
	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j < M; j++) {
			double sum = 0.;

			for (size_t k = 0; k < M; k++) {
				sum += X(i, k) * X(k, j);
			}

			A(i, j) = sum;
		}
	}

	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j < M; j++) {
			double sum = 0.;

			for (size_t k = 0; k < M; k++) {
				sum += L(i, k) * A(k, j);
			}

			X(i, j) = sum;
		}
	}

	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j < M; j++) {
			double sum = 0.;

			for (size_t k = 0; k < M; k++) {
				sum += X(i, k) * L(j, k);
			}

			A(i, j) = sum;
		}
	}

	// res = G' * A
	for (size_t j = 0; j < N; j++) {
		for (size_t i = 0; i < M; i++) {
			double sum = 0.;

			for (size_t k = 0; k < M; k++) {
				sum += static_cast<double>(G(k, j)) * A(k, i);
			}

			res(j, i) = static_cast<float>(sum);
		}
	}

	return true;
}


template<typename Type>
Type typeEpsilon();

template<> inline
float typeEpsilon<float>()
{
	return FLT_EPSILON;
}

template<> inline
double typeEpsilon<double>()
{
	return DBL_EPSILON;
}

/**
 * Full rank Cholesky factorization of A
 */
template<typename Type, size_t N>
SquareMatrix<Type, N> fullRankCholesky(const SquareMatrix<Type, N> &A,
				       size_t &rank)
{
	// Loses one ulp accuracy per row of diag, relative to largest magnitude
	const Type tol = N * typeEpsilon<Type>() * A.diag().max();

	Matrix<Type, N, N> L;

	size_t r = 0;

	for (size_t k = 0; k < N; k++) {

		if (r == 0) {
			for (size_t i = k; i < N; i++) {
				L(i, r) = A(i, k);
			}

		} else {
			for (size_t i = k; i < N; i++) {
				// Compute LL = L[k:n, :r] * L[k, :r].T
				Type LL = Type();

				for (size_t j = 0; j < r; j++) {
					LL += L(i, j) * L(k, j);
				}

				L(i, r) = A(i, k) - LL;
			}
		}

		if (L(k, r) > tol) {
			L(k, r) = std::sqrt(L(k, r));

			if (k < N - 1) {
				for (size_t i = k + 1; i < N; i++) {
					L(i, r) = L(i, r) / L(k, r);
				}
			}

			r = r + 1;
		}
	}

	// Return rank
	rank = r;

	return L;
}

} // namespace matrix
