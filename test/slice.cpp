#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
    float data[9] = {0, 2, 3,
                     4, 5, 6,
                     7, 8, 10
                    };
    SquareMatrix<float, 3> A(data);

    // Test row slicing
    Matrix<float, 2, 3> B_rowslice(A.slice<2, 3>(1, 0));
    float data_check_rowslice[6] = {
        4, 5, 6,
        7, 8, 10
    };
    Matrix<float, 2, 3> B_check_rowslice(data_check_rowslice);
    TEST(isEqual(B_rowslice, B_check_rowslice));

    // Test column slicing
    Matrix<float, 3, 2> B_colslice(A.slice<3, 2>(0, 1));
    float data_check_colslice[6] = {
        2, 3,
        5, 6,
        8, 10
    };
    Matrix<float, 3, 2> B_check_colslice(data_check_colslice);
    TEST(isEqual(B_colslice, B_check_colslice));

    // Test slicing both
    Matrix<float, 2, 2> B_bothslice(A.slice<2, 2>(1, 1));
    float data_check_bothslice[4] = {
        5, 6,
        8, 10
    };
    Matrix<float, 2, 2> B_check_bothslice(data_check_bothslice);
    TEST(isEqual(B_bothslice, B_check_bothslice));

    //Test block writing
    float data_2[4] = {
        11, 12,
        13, 14
    };

    Matrix<float, 2, 2> C(data_2);
    A.set(C, 1, 1);

    float data_2_check[9] = {
        0, 2, 3,
        4, 11, 12,
        7, 13, 14
    };
    Matrix<float, 3, 3> D(data_2_check);
    TEST(isEqual(A, D));

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
