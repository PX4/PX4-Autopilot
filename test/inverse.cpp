#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

static const size_t n_large = 50;

int main()
{
    float data[9] = {0, 2, 3,
                     4, 5, 6,
                     7, 8, 10
                    };
    float data_check[9] = {
        -0.4f, -0.8f,  0.6f,
        -0.4f,  4.2f, -2.4f,
        0.6f, -2.8f,  1.6f
    };

    SquareMatrix<float, 3> A(data);
    SquareMatrix<float, 3> A_I = inv(A);
    SquareMatrix<float, 3> A_I_check(data_check);
    TEST((A_I - A_I_check).abs().max() < 1e-6f);

    // stess test
    SquareMatrix<float, n_large> A_large;
    A_large.setIdentity();
    SquareMatrix<float, n_large> A_large_I;
    A_large_I.setZero();

    for (size_t i = 0; i < n_large; i++) {
        A_large_I = inv(A_large);
        TEST(isEqual(A_large, A_large_I));
    }

    SquareMatrix<float, 3> zero_test = zeros<float, 3, 3>();
    inv(zero_test);

    // test pivotting
    float data2[81] = {
        -2,   1,   1,  -1,  -5,   1,   2,  -1,   0,
        -3,   2,  -1,   0,   2,   2,  -1,  -5,   3,
        0,   0,   0,   1,   4,  -3,   3,   0,  -2,
        2,   2,  -1,  -2,  -1,   0,   3,   0,   1,
        -1,   2,  -1,  -1,  -3,   3,   0,  -2,   3,
        0,   1,   1,  -3,   3,  -2,   0,  -4,   0,
        1,   0,   0,   0,   0,   0,  -2,   4,  -3,
        1,  -1,   0,  -1,  -1,   1,  -1,  -3,   4,
        0,   3,  -1,  -2,   2,   1,  -2,   0,  -1
    };

    float data2_check[81] = {
        6, -4,   3, -3, -9, -8, -10,   8,  14,
        -2, -7,  -5, -3, -2, -2, -16,  -5,   8,
        -2,  0, -23,  7, -24, -5, -28, -14,   9,
        3, -7,   2, -5,  -4, -6, -13,   4,  13,
        -1,  4,  -8,  5,  -8,  0,  -3,  -5,  -2,
        6,  7,  -7,  7, -21, -7,  -5,   3,   6,
        1,  4,  -4,  4,  -7, -1,   0,  -1,  -1,
        -7,  3, -11,  5,   1,  6,  -1, -13, -10,
        -8,  0, -11,  3,   3,  6,  -5, -14,  -8
    };
    SquareMatrix<float, 9> A2(data2);
    SquareMatrix<float, 9> A2_I = inv(A2);
    SquareMatrix<float, 9> A2_I_check(data2_check);
    TEST((A2_I - A2_I_check).abs().max() < 1e-3f);
    float data3[9] = {
        0, 1, 2,
        3, 4, 5,
        6, 7, 9
    };
    float data3_check[9] = {
        -0.3333333f, -1.6666666f, 1,
        -1, 4, -2,
        1, -2, 1
    };
    SquareMatrix<float, 3> A3(data3);
    SquareMatrix<float, 3> A3_I = inv(A3);
    SquareMatrix<float, 3> A3_I_check(data3_check);
    TEST(isEqual(inv(A3), A3_I_check));
    TEST(isEqual(A3_I, A3_I_check));
    TEST(A3.I(A3_I));
    TEST(isEqual(A3_I, A3_I_check));

    // cover singular matrices
    A3(0, 0) = 0;
    A3(0, 1) = 0;
    A3(0, 2) = 0;
    A3_I = inv(A3);
    SquareMatrix<float, 3>  Z3 = zeros<float, 3, 3>();
    TEST(!A3.I(A3_I));
    TEST(!Z3.I(A3_I));
    TEST(isEqual(A3_I, Z3));
    TEST(isEqual(A3.I(), Z3));

    // cover NaN
    A3(0, 0) = NAN;
    A3(0, 1) = 0;
    A3(0, 2) = 0;
    A3_I = inv(A3);
    TEST(isEqual(A3_I, Z3));
    TEST(isEqual(A3.I(), Z3));

    float data4[9] = {
        1.33471626f,  0.74946721f, -0.0531679f,
        0.74946721f,  1.07519593f,  0.08036323f,
        -0.0531679f,  0.08036323f,  1.01618474f
    };
    SquareMatrix<float, 3> A4(data4);

    float data4_cholesky[9] = {
        1.15529921f,  0.f,  0.f,
        0.6487213f,  0.80892311f,  0.f,
        -0.04602089f,  0.13625271f,  0.99774847f
    };
    SquareMatrix<float, 3> A4_cholesky_check(data4_cholesky);
    SquareMatrix<float, 3> A4_cholesky = cholesky(A4);
    TEST(isEqual(A4_cholesky_check, A4_cholesky));

    SquareMatrix<float, 3> I3;
    I3.setIdentity();
    TEST(isEqual(choleskyInv(A4)*A4, I3));
    TEST(isEqual(cholesky(Z3), Z3));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
