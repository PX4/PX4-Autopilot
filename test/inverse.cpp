#include "Matrix.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
    float data[9] = {1, 0, 0, 0, 1, 0, 1, 0, 1};
    Matrix3f A(data);
    Matrix3f A_I = A.inverse();
    float data_check[9] = {1, 0, 0, 0, 1, 0, -1, 0, 1};
    Matrix3f A_I_check(data_check);
    (void)A_I;
    assert(A_I == A_I_check);

    // stess test
    static const size_t n = 50;
    Matrix<float, n, n> A_large;
    A_large.setIdentity();
    Matrix<float, n, n> A_large_I;
    A_large_I.setZero();

    for (size_t i = 0; i < 50; i++) {
        A_large_I = A_large.inverse();
        assert(A_large == A_large_I);
    }

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
