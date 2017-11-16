#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
    float eps = 1e-6f;

    // Vector3 copyTo
    Vector3f v(1, 2, 3);
    float dst3[3] = {};
    v.copyTo(dst3);
    for (size_t i = 0; i < 3; i++) {
        TEST(fabs(v(i) - dst3[i]) < eps);
    }

    // Quaternion copyTo
    Quatf q(1, 2, 3, 4);
    float dst4[4] = {};
    q.copyTo(dst4);
    for (size_t i = 0; i < 4; i++) {
        TEST(fabs(q(i) - dst4[i]) < eps);
    }

    // Matrix copyTo
    Matrix<float, 2, 3> A;
    A(0,0) = 1;
    A(0,1) = 2;
    A(0,2) = 3;
    A(1,0) = 4;
    A(1,1) = 5;
    A(1,2) = 6;
    float array_A[6] = {};
    A.copyTo(array_A);
    float array_row[6] = {1, 2, 3, 4, 5, 6};
    for (size_t i = 0; i < 6; i++) {
        TEST(fabs(array_A[i] - array_row[i]) < eps);
    }

    // Matrix copyTo with a pointer
    A.copyToRaw(static_cast <float *> (array_A));
    float array_row_p[6] = {1, 2, 3, 4, 5, 6};
    for (size_t i = 0; i < 6; i++) {
        TEST(fabs(array_A[i] - array_row_p[i]) < eps);
    }

    // Matrix copyToColumnMajor
    A.copyToColumnMajor(array_A);
    float array_column[6] = {1, 4, 2, 5, 3, 6};
    for (size_t i = 0; i < 6; i++) {
        TEST(fabs(array_A[i] - array_column[i]) < eps);
    }

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
