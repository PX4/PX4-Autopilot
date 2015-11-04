#include "Vector.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
    Vector<float, 5> v;
    float n = v.norm();
    (void)n;
    float r = v.dot(v);
    (void)r;
    return 0;
}
