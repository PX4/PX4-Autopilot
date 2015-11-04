#include "Quaternion.hpp"
#include <cassert>
#include <cstdio>

using namespace matrix;

int main()
{
    Quatf p(1, 2, 3, 4);
    assert(p(0) == 1);
    assert(p(1) == 2);
    assert(p(2) == 3);
    assert(p(3) == 4);

    Quatf q(0, 1, 0, 0);
    Quatf r = p*q;
    Dcmf dcm = Dcmf(p);
    Eulerf e = Eulerf(p);
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
