#include <assert.h>
#include <stdio.h>

#include "matrix.hpp"

using namespace matrix;

template class Vector<float, 5>;

int main()
{
    Vector<float, 5> v;
    float n = v.norm();
    (void)n;
    float r = v.dot(v);
    (void)r;

    Vector<float, 5> v2(v);
    assert(v == v2);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
