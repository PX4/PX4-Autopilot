#include <matrix/math.hpp>

#include "test_macros.hpp"

using namespace matrix;

int main()
{
    Vector3f v;
    v(0) = 1;
    v(1) = 2;
    v(2) = 3;

    static const float eps = 1e-7f;

    TEST(fabs(v(0) - 1) < eps);
    TEST(fabs(v(1) - 2) < eps);
    TEST(fabs(v(2) - 3) < eps);

    Vector3f v2(4, 5, 6);

    TEST(fabs(v2(0) - 4) < eps);
    TEST(fabs(v2(1) - 5) < eps);
    TEST(fabs(v2(2) - 6) < eps);

    SquareMatrix<float, 3> m = diag(Vector3f(1,2,3));
    TEST(fabs(m(0, 0) - 1) < eps);
    TEST(fabs(m(1, 1) - 2) < eps);
    TEST(fabs(m(2, 2) - 3) < eps);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
