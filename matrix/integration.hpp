#pragma once

#include "math.hpp"

namespace matrix {

template<typename Type, size_t M>
int integrate_rk4(
    Vector<Type, M> (*f)(Type, Vector<Type, M>),
    const Vector<Type, M> & y0,
    Type t0,
    Type h,
    Vector<Type, M> & y1
)
{
    // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    Vector<Type, M> k1, k2, k3, k4;
    k1 = f(t0, y0);
    k2 = f(t0 + h/2, y0 + k1*h/2);
    k3 = f(t0 + h/2, y0 + k2*h/2);
    k4 = f(t0 + h, y0 + k3*h);
    y1 = y0 + (k1 + k2*2 + k3*2 + k4)*(h/6);
    return 0;
}

}; // namespace matrix
