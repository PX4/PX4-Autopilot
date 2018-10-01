

#include <matrix/math.hpp>

#include "test_macros.hpp"

using namespace matrix;

int main()
{
    Vector2f a(1, 0);
    Vector2f b(0, 1);
    TEST(fabs(a % b - 1.0f) < 1e-5);

    Vector2f c;
    TEST(fabs(c(0) - 0) < 1e-5);
    TEST(fabs(c(1) - 0) < 1e-5);

    Matrix<float, 2, 1> d(a);
    TEST(fabs(d(0,0) - 1) < 1e-5);
    TEST(fabs(d(1,0) - 0) < 1e-5);

    Vector2f e(d);
    TEST(fabs(e(0) - 1) < 1e-5);
    TEST(fabs(e(1) - 0) < 1e-5);

    float data[] = {4,5};
    Vector2f f(data);
    TEST(fabs(f(0) - 4) < 1e-5);
    TEST(fabs(f(1) - 5) < 1e-5);

    Vector3f g(1.23f, 423.4f, 3221.f);
    Vector2f h(g);
    TEST(fabs(h(0) - 1.23f) < 1e-5);
    TEST(fabs(h(1) - 423.4f) < 1e-5);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
