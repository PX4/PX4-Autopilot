

#include <matrix/math.hpp>

#include "test_macros.hpp"

using matrix::Matrix;
using matrix::Vector2f;

int main()
{
    Vector2f a(1, 0);
    Vector2f b(0, 1);
    TEST(fabsf(a % b - 1.0f) < 1e-5);

    Vector2f c;
    TEST(fabsf(c(0) - 0) < 1e-5);
    TEST(fabsf(c(1) - 0) < 1e-5);

    Matrix<float, 2, 1> d(a);
    TEST(fabsf(d(0,0) - 1) < 1e-5);
    TEST(fabsf(d(1,0) - 0) < 1e-5);

    Vector2f e(d);
    TEST(fabsf(e(0) - 1) < 1e-5);
    TEST(fabsf(e(1) - 0) < 1e-5);

    float data[] = {4,5};
    Vector2f f(data);
    TEST(fabsf(f(0) - 4) < 1e-5);
    TEST(fabsf(f(1) - 5) < 1e-5);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
