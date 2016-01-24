#include <stdio.h>

#include <matrix/math.hpp>
#include "test_macros.hpp"

using namespace matrix;

int main()
{
    float data[9] = {1, 0, 0, 0, 1, 0, 1, 0, 1};
    Matrix3f A(data);
    float data_check[9] = {1, 0, 0, 0, 1, 0, -1, 0, 1};
    Matrix3f A_I(data_check);
    Matrix3f I;
    I.setIdentity();
    Matrix3f R = A * A_I;
    TEST(isEqual(R, I));

    Matrix3f R2 = A;
    R2 *= A_I;
    TEST(isEqual(R2, I));


    Matrix3f A2 = eye<float, 3>()*2;
    Matrix3f B = A2.emult(A2);
    Matrix3f B_check = eye<float, 3>()*4;
    TEST(isEqual(B, B_check));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
