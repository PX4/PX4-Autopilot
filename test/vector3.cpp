#include <stdio.h>

#include <matrix/math.hpp>
#include "test_macros.hpp"

using namespace matrix;

template class Vector<float, 3>;

int main()
{
    Vector3f a(1, 0, 0);
    Vector3f b(0, 1, 0);
    Vector3f c = a.cross(b);
    TEST(isEqual(c, Vector3f(0,0,1)));
    c = a % b;
    TEST (isEqual(c, Vector3f(0,0,1)));
    Matrix<float, 3, 1> d(c);
    Vector3f e(d);
    TEST (isEqual(e, d));
    float data[] = {4, 5, 6};
    Vector3f f(data);
    TEST(isEqual(f, Vector3f(4, 5, 6)));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
