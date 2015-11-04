#include "Matrix.hpp"
#include <assert.h>

using namespace matrix;

template class Matrix<float, 3, 3>;

int main()
{
    Matrix3f A;
    A.setIdentity();

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                assert(A(i, j) == 1);

            } else {
                assert(A(i, j) == 0);
            }
        }
    }

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
