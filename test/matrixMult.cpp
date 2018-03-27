#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
    float data[9] = {1, 0, 0, 0, 1, 0, 1, 0, 1};
    Matrix3f A(data);

    const Matrix3f Const(data);
    const float * raw_data = Const.data();
    const float eps = 1e-4f;
    for (int i=0; i<9; i++) {
        TEST(fabs(raw_data[i] - data[i]) < eps);
    }

    float data_check[9] = {1, 0, 0, 0, 1, 0, -1, 0, 1};
    Matrix3f A_I(data_check);
    Matrix3f I;
    I.setIdentity();
    Matrix3f R = A * A_I;
    TEST(isEqual(R, I));

    Matrix3f R2 = A;
    R2 *= A_I;
    TEST(isEqual(R2, I));

    TEST(R2==I);
    TEST(A!=A_I);
    Matrix3f A2 = eye<float, 3>()*2;
    Matrix3f B = A2.emult(A2);
    Matrix3f B_check = eye<float, 3>()*4;
    Matrix3f C_check = eye<float, 3>()*2;
    TEST(isEqual(B, B_check));
    Matrix3f C = B_check.edivide(C_check);
    TEST(isEqual(C, C_check));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
