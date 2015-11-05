#include "Quaternion.hpp"
#include <cassert>
#include <cstdio>

using namespace matrix;

// instantiate so coverage works
template class Quaternion<float>;
template class Euler<float>;
template class Dcm<float>;

int main()
{
    // test default ctor
    Quatf q;
    assert(q(0) == 1);
    assert(q(1) == 0);
    assert(q(2) == 0);
    assert(q(3) == 0);

    q = Quatf(1,2,3,4);
    assert(q(0) == 1);
    assert(q(1) == 2);
    assert(q(2) == 3);
    assert(q(3) == 4);

    q = Quatf(0.1825742f, 0.3651484f, 0.5477226f, 0.7302967f);
    assert(q(0) == 0.1825742f);
    assert(q(1) == 0.3651484f);
    assert(q(2) == 0.5477226f);
    assert(q(3) == 0.7302967f);

    // test euler ctor
    q = Quatf(Eulerf(0.1f, 0.2f, 0.3f));
    assert((q - Quatf(0.983347f, 0.034271f, 0.106021f, 0.143572f)).norm() < 1e-5);

    // test dcm ctor
    q = Quatf(Dcmf());
    assert(q == Quatf(1.0f, 0.0f, 0.0f, 0.0f));
    // TODO test derivative
    // test accessors
    q(0) = 0.1f;
    q(1) = 0.2f;
    q(2) = 0.3f;
    q(3) = 0.4f;
    assert(q == Quatf(0.1f, 0.2f, 0.3f, 0.4f));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
