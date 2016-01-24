#include <stdio.h>

#include <matrix/helper_functions.hpp>
#include "test_macros.hpp"

using namespace matrix;


int main()
{
    TEST(fabs(wrap_pi(4.0) - (4.0 - 2*M_PI)) < 1e-5);
    TEST(fabs(wrap_pi(-4.0) - (-4.0 + 2*M_PI)) < 1e-5);
    TEST(fabs(wrap_pi(3.0) - (3.0)) < 1e-3);
    wrap_pi(NAN);

    Vector3f a(1, 2, 3);
    Vector3f b(4, 5, 6);
    a.T().print();
    TEST(!isEqual(a, b));
    TEST(isEqual(a, a));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
