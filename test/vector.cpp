#include <assert.h>
#include <stdio.h>

#include "matrix.hpp"

using namespace matrix;

template class Vector<float, 5>;

int main()
{
    float data1[] = {1,2,3,4,5};
    float data2[] = {6,7,8,9,10};
    Vector<float, 5> v1(data1);
    assert(fabs(v1.norm() - 7.416198487095663f) < 1e-5);
    Vector<float, 5> v2(data2);
    assert(fabs(v1.dot(v2) - 130.0f) < 1e-5);
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
