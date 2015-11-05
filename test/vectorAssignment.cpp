#include "Vector3.hpp"
#include <assert.h>

using namespace matrix;

template class Vector<float, 3>;

int main()
{
    Vector3f v;
    v(0) = 1;
    v(1) = 2;
    v(2) = 3;

    v.print();

    static const float eps = 1e-7f;

    assert(fabs(v(0) - 1) < eps);
    assert(fabs(v(1) - 2) < eps);
    assert(fabs(v(2) - 3) < eps);

    Vector3f v2(4, 5, 6);

    v2.print();

    assert(fabs(v2(0) - 4) < eps);
    assert(fabs(v2(1) - 5) < eps);
    assert(fabs(v2(2) - 6) < eps);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
