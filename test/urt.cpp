#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
    float data[9] = {1, 2, 3,
                     4, 5, 6,
                     7, 8, 10
                    };
    float urt[6] = {1, 2, 3, 5, 6, 10};

    SquareMatrix<float, 3> A(data);

    for(int i=0; i<6; i++) {
        TEST(fabs(urt[i] - A.urt().data()[i]) < 1e-6f);
    }

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
