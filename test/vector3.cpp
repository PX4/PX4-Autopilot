#include "test_macros.hpp"

#include <matrix/math.hpp>

using matrix::Vector3f;
using matrix::Matrix;

int main()
{
    Vector3f a(1, 0, 0);
    Vector3f b(0, 1, 0);
    Vector3f c = a.cross(b);
    TEST(matrix::isEqual(c, Vector3f(0,0,1)));
    c = a % b;
    TEST(matrix::isEqual(c, Vector3f(0,0,1)));
    Matrix<float, 3, 1> d(c);
    Vector3f e(d);
    TEST (matrix::isEqual(e, d));
    float data[] = {4, 5, 6};
    Vector3f f(data);
    TEST(matrix::isEqual(f, Vector3f(4, 5, 6)));

    TEST(matrix::isEqual(a + b, Vector3f(1, 1, 0)));
    TEST(matrix::isEqual(a - b, Vector3f(1, -1, 0)));
    TEST(matrix::isEqualF(a * b, 0.0f));
    TEST(matrix::isEqual(-a, Vector3f(-1, 0, 0)));
    TEST(matrix::isEqual(a.unit(), a));
    TEST(matrix::isEqual(a.unit(), a.normalized()));
    TEST(matrix::isEqual(a*2.0, Vector3f(2, 0, 0)));

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
