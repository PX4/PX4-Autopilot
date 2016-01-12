#include <assert.h>
#include <stdio.h>

#include <matrix/helper_functions.hpp>

using namespace matrix;


int main()
{
    assert(fabs(wrap_pi(4.0) - (4.0 - 2*M_PI)) < 1e-5);
    assert(fabs(wrap_pi(-4.0) - (-4.0 + 2*M_PI)) < 1e-5);
    assert(fabs(wrap_pi(3.0) - (3.0)) < 1e-3);
    wrap_pi(NAN);
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
