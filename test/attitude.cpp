#include <cassert>
#include <cstdio>

#include <matrix/math.hpp>

using namespace matrix;

template class Quaternion<float>;
template class Euler<float>;
template class Dcm<float>;

int main()
{
    double eps = 1e-6;

    // check data
    Eulerf euler_check(0.1f, 0.2f, 0.3f);
    Quatf q_check(0.98334744f, 0.0342708f, 0.10602051f, .14357218f);
    float dcm_data[] =  {
        0.93629336f, -0.27509585f,  0.21835066f,
        0.28962948f,  0.95642509f, -0.03695701f,
        -0.19866933f,  0.0978434f,  0.97517033f
    };
    Dcmf dcm_check(dcm_data);

    // euler ctor
    euler_check.T().print();
    assert(euler_check == Vector3f(0.1f, 0.2f, 0.3f));

    // euler default ctor
    Eulerf e;
    Eulerf e_zero = zeros<float, 3, 1>();
    assert(e == e_zero);
    assert(e == e);

    // euler vector ctor
    Vector<float, 3> v;
    v(0) = 0.1f;
    v(1) = 0.2f;
    v(2) = 0.3f;
    Eulerf euler_copy(v);
    assert(euler_copy == euler_check);

    // quaternion ctor
    Quatf q(1, 2, 3, 4);
    assert(fabs(q(0) - 1) < eps);
    assert(fabs(q(1) - 2) < eps);
    assert(fabs(q(2) - 3) < eps);
    assert(fabs(q(3) - 4) < eps);

    // quat normalization
    q.T().print();
    q.normalize();
    q.T().print();
    assert(q == Quatf(0.18257419f,  0.36514837f,
                      0.54772256f,  0.73029674f));

    // quat default ctor
    q = Quatf();
    assert(q == Quatf(1, 0, 0, 0));

    // euler to quaternion
    q = Quatf(euler_check);
    q.T().print();
    assert(q == q_check);

    // euler to dcm
    Dcmf dcm(euler_check);
    dcm.print();
    assert(dcm == dcm_check);

    // quaternion to euler
    Eulerf e1(q_check);
    assert(e1 == euler_check);

    // quaternion to dcm
    Dcmf dcm1(q_check);
    dcm1.print();
    assert(dcm1 == dcm_check);

    // dcm default ctor
    Dcmf dcm2;
    dcm2.print();
    SquareMatrix<float, 3> I = eye<float, 3>();
    assert(dcm2 == I);

    // dcm to euler
    Eulerf e2(dcm_check);
    assert(e2 == euler_check);

    // dcm to quaterion
    Quatf q2(dcm_check);
    assert(q2 == q_check);


    // euler gimbal lock check
    // note if theta = pi/2, then roll is set to zero
    float pi_2 = float(M_PI_2);
    Eulerf euler_gimbal_lock(0.1f, pi_2, 0.2f);
    Dcmf dcm_lock(euler_gimbal_lock);
    Eulerf euler_gimbal_lock_out(dcm_lock);
    euler_gimbal_lock_out.T().print();
    euler_gimbal_lock.T().print();
    assert(euler_gimbal_lock == euler_gimbal_lock_out);

    // note if theta = pi/2, then roll is set to zero
    Eulerf euler_gimbal_lock2(0.1f, -pi_2, 0.2f);
    Dcmf dcm_lock2(euler_gimbal_lock2);
    Eulerf euler_gimbal_lock_out2(dcm_lock2);
    euler_gimbal_lock_out2.T().print();
    euler_gimbal_lock2.T().print();
    assert(euler_gimbal_lock2 == euler_gimbal_lock_out2);

    // quaterion copy ctors
    float data_v4[] = {1, 2, 3, 4};
    Vector<float, 4> v4(data_v4);
    Quatf q_from_v(v4);
    assert(q_from_v == v4);

    Matrix<float, 4, 1> m4(data_v4);
    Quatf q_from_m(m4);
    assert(q_from_m == m4);

    // quaternion derivate
    Vector<float, 4> q_dot = q.derivative(Vector3f(1, 2, 3));
    printf("q_dot:\n");
    q_dot.T().print();

    // quaternion product
    Quatf q_prod_check(
        0.93394439f, 0.0674002f, 0.20851f, 0.28236266f);
    assert(q_prod_check == q_check*q_check);
    q_check *= q_check;
    assert(q_prod_check == q_check);

    // quaternion inverse
    q = q_check.inversed();
    assert(fabsf(q_check(0) - q(0)) < eps);
    assert(fabsf(q_check(1) + q(1)) < eps);
    assert(fabsf(q_check(2) + q(2)) < eps);
    assert(fabsf(q_check(3) + q(3)) < eps);

    q = q_check;
    q.invert();
    assert(fabsf(q_check(0) - q(0)) < eps);
    assert(fabsf(q_check(1) + q(1)) < eps);
    assert(fabsf(q_check(2) + q(2)) < eps);
    assert(fabsf(q_check(3) + q(3)) < eps);

    // rotate quaternion (nonzero rotation)
    Quatf qI(1.0f, 0.0f, 0.0f, 0.0f);
    Vector<float, 3> rot;
    rot(0) = 1.0f;
    rot(1) = rot(2) = 0.0f;
    qI.rotate(rot);
    Quatf q_true(cosf(1.0f / 2), sinf(1.0f / 2), 0.0f, 0.0f);
    assert(fabsf(qI(0) - q_true(0)) < eps);
    assert(fabsf(qI(1) - q_true(1)) < eps);
    assert(fabsf(qI(2) - q_true(2)) < eps);
    assert(fabsf(qI(3) - q_true(3)) < eps);

    // rotate quaternion (zero rotation)
    qI = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    rot(0) = 0.0f;
    rot(1) = rot(2) = 0.0f;
    qI.rotate(rot);
    q_true = Quatf(cosf(0.0f), sinf(0.0f), 0.0f, 0.0f);
    assert(fabsf(qI(0) - q_true(0)) < eps);
    assert(fabsf(qI(1) - q_true(1)) < eps);
    assert(fabsf(qI(2) - q_true(2)) < eps);
    assert(fabsf(qI(3) - q_true(3)) < eps);

    // get rotation axis from quaternion (nonzero rotation)
    q = Quatf(cosf(1.0f / 2), 0.0f, sinf(1.0f / 2), 0.0f);
    rot = q.to_axis_angle();
    assert(fabsf(rot(0)) < eps);
    assert(fabsf(rot(1) -1.0f) < eps);
    assert(fabsf(rot(2)) < eps);

    // get rotation axis from quaternion (zero rotation)
    q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    rot = q.to_axis_angle();
    assert(fabsf(rot(0)) < eps);
    assert(fabsf(rot(1)) < eps);
    assert(fabsf(rot(2)) < eps);

    // from axis angle (zero rotation)
    rot(0) = rot(1) = rot(2) = 0.0f;
    q.from_axis_angle(rot, 0.0f);
    q_true = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    assert(fabsf(q(0) - q_true(0)) < eps);
    assert(fabsf(q(1) - q_true(1)) < eps);
    assert(fabsf(q(2) - q_true(2)) < eps);
    assert(fabsf(q(3) - q_true(3)) < eps);

};

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
