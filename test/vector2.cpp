#include <assert.h>
#include <stdio.h>

#include <matrix/math.hpp>

using namespace matrix;

template class Vector<float, 3>;

int main()
{
    Vector2f a(1, 0);
    Vector2f b(0, 1);
    assert(fabs(a % b - 1.0f) < 1e-5);

    Vector2f c;
    assert(fabs(c(0) - 0) < 1e-5);
    assert(fabs(c(1) - 0) < 1e-5);

    Matrix<float, 2, 1> d(a);
    assert(fabs(d(0,0) - 1) < 1e-5);
    assert(fabs(d(1,0) - 0) < 1e-5);

    Vector2f e(d);
    assert(fabs(e(0) - 1) < 1e-5);
    assert(fabs(e(1) - 0) < 1e-5);

    float data[] = {4,5};
    Vector2f f(data);
    assert(fabs(f(0) - 4) < 1e-5);
    assert(fabs(f(1) - 5) < 1e-5);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
