#pragma once

#include "math.hpp"

namespace matrix {

template<typename Type, size_t M, size_t N>
int kalman_correct(
    const SquareMatrix<Type, M> & P,
    const Matrix<Type, N, M> & C,
    const SquareMatrix<Type, N> & R,
    const Vector<Type, N> &r,
    Vector<Type, M> & dx,
    SquareMatrix<Type, M> & dP,
    float & beta
)
{
    SquareMatrix<Type, N> S_I = SquareMatrix<Type, N>(C*P*C.T() + R).I();
    Matrix<Type, M, N> K = P*C.T()*S_I;
    dx = K*r;
    beta = Scalar<Type>(r.T()*S_I*r);
    dP = K*C*P*(-1);
    return 0;
}

}; // namespace matrix
