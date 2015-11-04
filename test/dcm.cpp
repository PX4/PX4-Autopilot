#include "Dcm.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
    Dcmf dcm;
    Quatf q = Quatf(dcm);
    Eulerf e = Eulerf(dcm);
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
