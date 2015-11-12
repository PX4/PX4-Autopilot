#include <assert.h>
#include <stdio.h>

#include <matrix/integration.hpp>

using namespace matrix;

Vector<float, 6> f(float t, const Vector<float, 6> & y, const Vector<float, 3> & u);

Vector<float, 6> f(float t, const Vector<float, 6> & y, const Vector<float, 3> & u) {
    return ones<float, 6, 1>();
}

int main()
{
    Vector<float, 6> y = ones<float, 6, 1>();
    Vector<float, 3> u = ones<float, 3, 1>();
    float t = 1;
    float h = 0.1f;
    y.T().print();
    integrate_rk4(f, y, u, t, h, y);
    y.T().print();
    assert(y == (ones<float, 6, 1>()*1.1f));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
