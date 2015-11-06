#include <assert.h>
#include <stdio.h>

#include "matrix.hpp"

using namespace matrix;

template class Vector<float, 3>;

int main()
{
    Vector3f a(1, 0, 0);
    Vector3f b(0, 1, 0);
    Vector3f c = a.cross(b);
    c.print();
    assert (c == Vector3f(0,0,1));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
