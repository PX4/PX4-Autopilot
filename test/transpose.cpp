#include <stdio.h>

#include <matrix/math.hpp>
#include "test_macros.hpp"

using namespace matrix;

template class Matrix<float, 2, 3>;
template class Matrix<float, 3, 2>;

int main()
{
    float data[6] = {1, 2, 3, 4, 5, 6};
    Matrix<float, 2, 3> A(data);
    Matrix<float, 3, 2> A_T = A.transpose();
    float data_check[6] = {1, 4, 2, 5, 3, 6};
    Matrix<float, 3, 2> A_T_check(data_check);
    TEST(isEqual(A_T, A_T_check));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
