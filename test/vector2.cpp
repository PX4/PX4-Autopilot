

#include <matrix/math.hpp>

#include "test_macros.hpp"

using namespace matrix;

int main()
{
    Vector2f a(1, 0);
    Vector2f b(0, 1);
    TEST(fabs(a % b - 1.0f) < __FLT_EPSILON__);

    Vector2f c;
    TEST(fabs(c(0) - 0) < __FLT_EPSILON__);
    TEST(fabs(c(1) - 0) < __FLT_EPSILON__);

    Matrix<float, 2, 1> d(a);
    TEST(fabs(d(0,0) - 1) < __FLT_EPSILON__);
    TEST(fabs(d(1,0) - 0) < __FLT_EPSILON__);

    Vector2f e(d);
    TEST(fabs(e(0) - 1) < __FLT_EPSILON__);
    TEST(fabs(e(1) - 0) < __FLT_EPSILON__);

    float data[] = {4,5};
    Vector2f f(data);
    TEST(fabs(f(0) - 4) < __FLT_EPSILON__);
    TEST(fabs(f(1) - 5) < __FLT_EPSILON__);

    Vector3f g(1.23f, 423.4f, 3221.f);
    Vector2f h(g);
    TEST(fabs(h(0) - 1.23f) < __FLT_EPSILON__);
    TEST(fabs(h(1) - 423.4f) < __FLT_EPSILON__);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
