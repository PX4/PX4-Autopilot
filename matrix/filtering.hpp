#pragma once

#include "matrix.hpp"

template<typename Type, size_t M, size_t N>
void kalman_correct(
		const Matrix<Type, M, M> & P,
		const Matrix<Type, N, N> & R,
		const Matrix<Type, N, M> & C,
		const Vector<Type, N> &r,
		Vector<Type, M> & dx,
		float beta
		)
{
	SuareMatrix<Type, N> S_I = SquarMatrix<Type, N>(C*P*C.T() + R).I();
	dx = P*C.T()*S_I*r;
	beta = Scalar(r.T()*S_I*r);
}
