#include <assert.h>
#include <stdio.h>

#include <matrix/math.hpp>

using namespace matrix;

template class Vector<float, 3>;

int main()
{
    Vector2f a(1, 0);
    Vector2f b(0, 1);
    assert (fabs(a % b - 1.0f) < 1e-5);
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
