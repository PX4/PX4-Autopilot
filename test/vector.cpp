#include "test_macros.hpp"

#include <matrix/math.hpp>

using namespace matrix;

int main()
{
    float data1[] = {1,2,3,4,5};
    float data2[] = {6,7,8,9,10};
    Vector<float, 5> v1(data1);
    TEST(isEqualF(v1.norm(), 7.416198487095663f));
    TEST(isEqualF(v1.norm(), v1.length()));
    Vector<float, 5> v2(data2);
    TEST(isEqualF(v1.dot(v2), 130.0f));
    v2.normalize();
    Vector<float, 5> v3(v2);
    TEST(isEqual(v2, v3));
    float data1_sq[] = {1,4,9,16,25};
    Vector<float, 5> v4(data1_sq);
    TEST(isEqual(v1, v4.pow(0.5)));

    // dot product operator
    v1 = Vector<float, 5>(data1);
    v2 = Vector<float, 5>(data2);
    float dprod = v1 * v2;
    TEST(isEqualF(dprod, 130.0f));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
