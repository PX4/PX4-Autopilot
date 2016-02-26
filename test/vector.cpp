#include <stdio.h>

#include <matrix/math.hpp>
#include "test_macros.hpp"

using namespace matrix;

template class Vector<float, 5>;

int main()
{
    float data1[] = {1,2,3,4,5};
    float data2[] = {6,7,8,9,10};
    Vector<float, 5> v1(data1);
    TEST(fabs(v1.norm() - 7.416198487095663f) < 1e-5);
    Vector<float, 5> v2(data2);
    TEST(fabs(v1.dot(v2) - 130.0f) < 1e-5);
    v2.normalize();
    Vector<float, 5> v3(v2);
    TEST(v2 == v3);
    float data1_sq[] = {1,4,9,16,25};
    Vector<float, 5> v4(data1_sq);
    TEST(v1 == v4.pow(0.5));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
