#pragma once

#include "math.hpp"

namespace matrix
{

template<typename Type, size_t M, size_t N>
int kalman_correct(
	const Matrix<Type, M, M> &P,
	const Matrix<Type, N, M> &C,
	const Matrix<Type, N, N> &R,
	const Matrix<Type, N, 1> &r,
	Matrix<Type, M, 1> &dx,
	Matrix<Type, M, M> &dP,
	Type &beta
)
{
	SquareMatrix<Type, N> S_I = SquareMatrix<Type, N>(C * P * C.T() + R).I();
	Matrix<Type, M, N> K = P * C.T() * S_I;
	dx = K * r;
	beta = Scalar<Type>(r.T() * S_I * r);
	dP = K * C * P * (-1);
	return 0;
}

} // namespace matrix
