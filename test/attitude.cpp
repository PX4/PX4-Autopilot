#include <cassert>
#include <cstdio>

#include "Quaternion.hpp"
#include "Vector3.hpp"
#include "matrix.hpp"

using namespace matrix;

template class Quaternion<float>;
template class Euler<float>;
template class Dcm<float>;

int main()
{
    double eps = 1e-6;

    // check data
    Eulerf euler_check(0.1, 0.2, 0.3);
    Quatf q_check(0.98334744, 0.0342708, 0.10602051, .14357218);
    float dcm_data[] =  {
        0.93629336, -0.27509585,  0.21835066,
        0.28962948,  0.95642509, -0.03695701,
        -0.19866933,  0.0978434 ,  0.97517033};
    Dcmf dcm_check(dcm_data);

    // euler ctor
    euler_check.T().print();
    assert((euler_check - Vector3f(0.1, 0.2, 0.3)).norm() < eps);

    // quaternion ctor
    Quatf q(1, 2, 3, 4);
    assert(q(0) == 1);
    assert(q(1) == 2);
    assert(q(2) == 3);
    assert(q(3) == 4);

    q.T().print();
    q.normalize();
    q.T().print();
    assert((q - Quatf(
        0.18257419,  0.36514837,  0.54772256,  0.73029674)
        ).norm() < eps);

    // euler to quaternion
    q = Quatf(euler_check);
    q.T().print();
    assert((q - q_check).norm() < eps);

    // euler to dcm
    Dcmf dcm(euler_check);
    dcm.print();
    assert((dcm - dcm_check).abs().max() < eps);

    // quaternion to euler
    Eulerf e1(q_check);
    assert((e1 - euler_check).norm() < eps);

    // quaternion to dcm
    Dcmf dcm1(q_check);
    assert((dcm1 - dcm_check).abs().max() < eps);

    // dcm to euler
    Eulerf e2(dcm_check);
    assert((e2 - euler_check).norm() < eps);

    // dcm to quaterion
    Quatf q2(dcm_check);
    assert((q2 - q_check).norm() < eps);

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
