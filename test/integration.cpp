#include <assert.h>
#include <stdio.h>

#include <matrix/integration.hpp>

using namespace matrix;

// instantiate template to ensure coverage check
template int integrate_rk4<float, 6>(
    Vector<float, 6> (*f)(float, Vector<float, 6>),
    Vector<float, 6> & y,
    float & t,
    float h
);

Vector<float, 6> f(float t, Vector<float, 6> y);

Vector<float, 6> f(float t, Vector<float, 6> y) {
    return ones<float, 6, 1>();
}

int main()
{
    Vector<float, 6> y = ones<float, 6, 1>();
    float h = 1;
    float t = 1;
    y.T().print();
    integrate_rk4(f, y, t, h);
    y.T().print();
    assert(y == (ones<float, 6, 1>()*2));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
