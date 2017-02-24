#include <matrix/math.hpp>

#include "test_macros.hpp"

using matrix::SquareMatrix;
using matrix::Vector3f;

int main()
{
    Vector3f v;
    v(0) = 1;
    v(1) = 2;
    v(2) = 3;

    static const float eps = 1e-7f;

    TEST(fabsf(v(0) - 1) < eps);
    TEST(fabsf(v(1) - 2) < eps);
    TEST(fabsf(v(2) - 3) < eps);

    Vector3f v2(4, 5, 6);

    TEST(fabsf(v2(0) - 4) < eps);
    TEST(fabsf(v2(1) - 5) < eps);
    TEST(fabsf(v2(2) - 6) < eps);

    SquareMatrix<float, 3> m = diag(Vector3f(1,2,3));
    TEST(fabsf(m(0, 0) - 1) < eps);
    TEST(fabsf(m(1, 1) - 2) < eps);
    TEST(fabsf(m(2, 2) - 3) < eps);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
