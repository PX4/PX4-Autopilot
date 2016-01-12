#pragma once

#include "math.hpp"

namespace matrix {

template<typename Type, size_t M, size_t N>
int integrate_rk4(
    Vector<Type, M> (*f)(Type, const Matrix<Type, M, 1> &x, const Matrix<Type, N, 1> & u),
    const Matrix<Type, M, 1> & y0,
    const Matrix<Type, N, 1> & u,
    Type t0,
    Type h,
    Matrix<Type, M, 1> & y1
)
{
    // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    Vector<Type, M> k1, k2, k3, k4;
    k1 = f(t0, y0, u);
    k2 = f(t0 + h/2, y0 + k1*h/2, u);
    k3 = f(t0 + h/2, y0 + k2*h/2, u);
    k4 = f(t0 + h, y0 + k3*h, u);
    y1 = y0 + (k1 + k2*2 + k3*2 + k4)*(h/6);
    return 0;
}

} // namespace matrix
