#pragma once

#include "matrix.hpp"

namespace matrix {

template<typename Type, size_t M, size_t N>
void kalman_correct(
    const Matrix<Type, M, M> & P,
    const Matrix<Type, N, M> & C,
    const Matrix<Type, N, N> & R,
    const Vector<Type, N> &r,
    Vector<Type, M> & dx,
    float & beta
)
{
    SquareMatrix<Type, N> S_I = SquareMatrix<Type, N>(C*P*C.T() + R).I();
    dx = P*C.T()*S_I*r;
    beta = Scalar<Type>(r.T()*S_I*r);
}

}; // namespace matrix
