

#include <matrix/math.hpp>

#include "test_macros.hpp"

using namespace matrix;

int main()
{
    Vector2f a(1, 0);
    Vector2f b(0, 1);
    TEST(fabs(a % b - 1.0f) < FLT_EPSILON);

    Vector2f c;
    TEST(fabs(c(0) - 0) < FLT_EPSILON);
    TEST(fabs(c(1) - 0) < FLT_EPSILON);

    Matrix<float, 2, 1> d(a);
    TEST(fabs(d(0,0) - 1) < FLT_EPSILON);
    TEST(fabs(d(1,0) - 0) < FLT_EPSILON);

    Vector2f e(d);
    TEST(fabs(e(0) - 1) < FLT_EPSILON);
    TEST(fabs(e(1) - 0) < FLT_EPSILON);

    float data[] = {4,5};
    Vector2f f(data);
    TEST(fabs(f(0) - 4) < FLT_EPSILON);
    TEST(fabs(f(1) - 5) < FLT_EPSILON);

    Vector3f g(1.23f, 423.4f, 3221.f);
    Vector2f h(g);
    TEST(fabs(h(0) - 1.23f) < FLT_EPSILON);
    TEST(fabs(h(1) - 423.4f) < FLT_EPSILON);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
