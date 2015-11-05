#include "filter.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

template class Vector<float, 5>;

int main()
{
    const size_t n_x = 6;
    const size_t n_y = 5;
    SquareMatrix<float, n_x> P = eye<float, n_x>()*0.1;
    SquareMatrix<float, n_y> R = eye<float, n_y>()*0.1;
    Matrix<float, n_y, n_x> C;
    C(0,0) = 1;
    Vector<float, n_y> r;
    r.setZero();
    r(0) = 1;

    Vector<float, n_x> dx;
    float beta = 0;
    kalman_correct<float, 6, 5>(P, C, R, r, dx, beta);
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
