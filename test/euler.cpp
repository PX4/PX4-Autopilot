#include "Euler.hpp"
#include "Scalar.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
    Eulerf e;
    float dp = Scalarf(e.T()*e);
    (void)dp;
    Dcmf dcm = Dcmf(e);
    Quatf q = Quatf(e);
    float n = e.norm();
    (void)n;
    return 0;
}
