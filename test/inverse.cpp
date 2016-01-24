#include <stdio.h>

#include <matrix/math.hpp>
#include "test_macros.hpp"

using namespace matrix;

static const size_t n_large = 50;

template class SquareMatrix<float, 3>;
template class SquareMatrix<float, n_large>;

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
    TEST((A_I - A_I_check).abs().max() < 1e-5);

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

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
