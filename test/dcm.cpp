#include <assert.h>
#include <stdio.h>

#include "matrix.hpp"

using namespace matrix;

int main()
{
    Dcmf dcm;
    Quatf q(1,0,0,0);
    dcm = Dcmf(q);
    Matrix3f I = eye<float, 3>();
    dcm = Dcmf(q);
    Eulerf e = Eulerf(dcm);
    return 0;
};

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
