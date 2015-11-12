#pragma once

#include "math.hpp"

namespace matrix {

template<typename Type, size_t M>
int integrate_rk4(
    Vector<Type, M> (*f)(Type, Vector<Type, M>),
    Vector<Type, M> & y,
    Type & t,
    Type h
)
{
    // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    Vector<Type, M> k1, k2, k3, k4;
    k1 = f(t, y);
    k2 = f(t + h/2, y + k1*h/2);
    k3 = f(t + h/2, y + k2*h/2);
    k4 = f(t + h, y + k3*h);
    y += (k1 + k2*2 + k3*2 + k4)*(h/6);
    t += h;
    return 0;
}

}; // namespace matrix
