#include <matrix/math.hpp>
#include "test_macros.hpp"

using namespace matrix;

template class Matrix<float, 3, 3>;

int main()
{
    Matrix3f A;
    A.setIdentity();

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                TEST( fabs(A(i, j) -  1) < 1e-7);

            } else {
                TEST( fabs(A(i, j) -  0) < 1e-7);
            }
        }
    }

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
