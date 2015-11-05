#include <assert.h>
#include <stdio.h>

#include "matrix.hpp"

using namespace matrix;

static const size_t n_large = 50;

template class SquareMatrix<float, 3>;
template class SquareMatrix<float, n_large>;

int main()
{
    float data[9] = {1, 2, 3,
                     4, 5, 6,
                     7, 8, 10};
    float data_check[9] = {-0.66666667f, -1.33333333f,  1.        ,
                           -0.66666667f,  3.66666667f, -2.        ,
                            1.        , -2.        ,  1.        };

    SquareMatrix<float, 3> A(data);
    SquareMatrix<float, 3> A_I = A.inverse();
    SquareMatrix<float, 3> A_I_check(data_check);
    A_I.print();
    A_I_check.print();
    assert(A_I == A_I_check);

    // stess test
    SquareMatrix<float, n_large> A_large;
    A_large.setIdentity();
    SquareMatrix<float, n_large> A_large_I;
    A_large_I.setZero();

    for (size_t i = 0; i < n_large; i++) {
        A_large_I = A_large.inverse();
        assert(A_large == A_large_I);
    }

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
