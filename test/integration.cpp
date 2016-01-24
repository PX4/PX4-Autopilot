#include <stdio.h>

#include <matrix/integration.hpp>
#include "test_macros.hpp"

using namespace matrix;

Vector<float, 6> f(float t, const Matrix<float, 6, 1> & y, const Matrix<float, 3, 1> & u);

Vector<float, 6> f(float t, const Matrix<float, 6, 1> & y, const Matrix<float, 3, 1> & u) {
    return ones<float, 6, 1>();
}

int main()
{
    Vector<float, 6> y = ones<float, 6, 1>();
    Vector<float, 3> u = ones<float, 3, 1>();
    float t = 1;
    float h = 0.1f;
    integrate_rk4(f, y, u, t, h, y);
    TEST(isEqual(y, (ones<float, 6, 1>()*1.1f)));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
