#include <assert.h>

#include "matrix.hpp"

using namespace matrix;

template class Matrix<float, 3, 3>;

int main()
{
    Matrix3f m;
    m.setZero();
    m(0, 0) = 1;
    m(0, 1) = 2;
    m(0, 2) = 3;
    m(1, 0) = 4;
    m(1, 1) = 5;
    m(1, 2) = 6;
    m(2, 0) = 7;
    m(2, 1) = 8;
    m(2, 2) = 9;

    m.print();

    float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    Matrix3f m2(data);
    m2.print();

    for(int i=0; i<9; i++) {
        assert(fabs(data[i] - m2.data()[i]) < 1e-6f);
    }

    float data_times_2[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
    Matrix3f m3(data_times_2);

    assert(m == m2);
    assert(!(m == m3));

    m2 *= 2;
    assert(m2 == m3);

    m2 /= 2;
    m2 -= 1;
    float data_minus_1[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    assert(Matrix3f(data_minus_1) == m2);

    m2 += 1;
    assert(Matrix3f(data) == m2);

    m3 -= m2;

    assert(m3 == m2);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
