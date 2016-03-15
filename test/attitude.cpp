#include <cstdio>
#include <stdexcept>

#include <matrix/math.hpp>
#include "test_macros.hpp"

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
    TEST(isEqual(euler_check, Vector3f(0.1f, 0.2f, 0.3f)));

    // euler default ctor
    Eulerf e;
    Eulerf e_zero = zeros<float, 3, 1>();
    TEST(isEqual(e, e_zero));
    TEST(isEqual(e, e));

    // euler vector ctor
    Vector<float, 3> v;
    v(0) = 0.1f;
    v(1) = 0.2f;
    v(2) = 0.3f;
    Eulerf euler_copy(v);
    TEST(isEqual(euler_copy, euler_check));

    // quaternion ctor
    Quatf q0(1, 2, 3, 4);
    Quatf q(q0);
    TEST(fabs(q(0) - 1) < eps);
    TEST(fabs(q(1) - 2) < eps);
    TEST(fabs(q(2) - 3) < eps);
    TEST(fabs(q(3) - 4) < eps);

    // quat normalization
    q.normalize();
    TEST(isEqual(q, Quatf(0.18257419f,  0.36514837f,
                          0.54772256f,  0.73029674f)));
    TEST(isEqual(q0.unit(), q));

    // quat default ctor
    q = Quatf();
    TEST(isEqual(q, Quatf(1, 0, 0, 0)));

    // euler to quaternion
    q = Quatf(euler_check);
    TEST(isEqual(q, q_check));

    // euler to dcm
    Dcmf dcm(euler_check);
    TEST(isEqual(dcm, dcm_check));

    // quaternion to euler
    Eulerf e1(q_check);
    TEST(isEqual(e1, euler_check));

    // quaternion to dcm
    Dcmf dcm1(q_check);
    TEST(isEqual(dcm1, dcm_check));

    // dcm default ctor
    Dcmf dcm2;
    SquareMatrix<float, 3> I = eye<float, 3>();
    TEST(isEqual(dcm2, I));

    // dcm to euler
    Eulerf e2(dcm_check);
    TEST(isEqual(e2, euler_check));

    // dcm to quaterion
    Quatf q2(dcm_check);
    TEST(isEqual(q2, q_check));

    // constants
    double deg2rad = M_PI/180.0;
    double rad2deg = 180.0/M_PI;

    // euler dcm round trip check
    for (int roll=-90; roll<=90; roll+=90) {
        for (int pitch=-90; pitch<=90; pitch+=90) {
            for (int yaw=-179; yaw<=180; yaw+=90) {
                // note if theta = pi/2, then roll is set to zero
                int roll_expected = roll;
                int yaw_expected = yaw;
                if (pitch == 90) {
                    roll_expected = 0;
                    yaw_expected = yaw - roll;
                } else if (pitch == -90) {
                    roll_expected = 0;
                    yaw_expected = yaw + roll;
                }
                if (yaw_expected < -180) yaw_expected += 360;
                if (yaw_expected > 180) yaw_expected -= 360;

                //printf("roll:%d pitch:%d yaw:%d\n", roll, pitch, yaw);
                Euler<double> euler_expected(
                    deg2rad*double(roll_expected),
                    deg2rad*double(pitch),
                    deg2rad*double(yaw_expected));
                Euler<double> euler(
                    deg2rad*double(roll),
                    deg2rad*double(pitch),
                    deg2rad*double(yaw));
                Dcm<double> dcm_from_euler(euler);
                //dcm_from_euler.print();
                Euler<double> euler_out(dcm_from_euler);
                TEST(isEqual(rad2deg*euler_expected, rad2deg*euler_out));

                Eulerf eulerf_expected(
                    float(deg2rad)*float(roll_expected),
                    float(deg2rad)*float(pitch),
                    float(deg2rad)*float(yaw_expected));
                Eulerf eulerf(float(deg2rad)*float(roll),
                              float(deg2rad)*float(pitch),
                              float(deg2rad)*float(yaw));
                Dcm<float> dcm_from_eulerf(eulerf);
                Euler<float> euler_outf(dcm_from_eulerf);
                TEST(isEqual(float(rad2deg)*eulerf_expected,
                             float(rad2deg)*euler_outf));
            }
        }
    }

    // quaterion copy ctors
    float data_v4[] = {1, 2, 3, 4};
    Vector<float, 4> v4(data_v4);
    Quatf q_from_v(v4);
    TEST(isEqual(q_from_v, v4));

    Matrix<float, 4, 1> m4(data_v4);
    Quatf q_from_m(m4);
    TEST(isEqual(q_from_m, m4));

    // quaternion derivate
    Vector<float, 4> q_dot = q.derivative(Vector3f(1, 2, 3));

    // quaternion product
    Quatf q_prod_check(
        0.93394439f, 0.0674002f, 0.20851f, 0.28236266f);
    TEST(isEqual(q_prod_check, q_check*q_check));
    q_check *= q_check;
    TEST(isEqual(q_prod_check, q_check));

    // Quaternion scalar multiplication
    float scalar = 0.5;
    Quatf q_scalar_mul(1.0f, 2.0f, 3.0f, 4.0f);
    Quatf q_scalar_mul_check(1.0f * scalar, 2.0f * scalar,
                             3.0f * scalar,  4.0f * scalar);
    Quatf q_scalar_mul_res = scalar * q_scalar_mul;
    TEST(isEqual(q_scalar_mul_check, q_scalar_mul_res));
    Quatf q_scalar_mul_res2 = q_scalar_mul * scalar;
    TEST(isEqual(q_scalar_mul_check, q_scalar_mul_res2));
    Quatf q_scalar_mul_res3(q_scalar_mul);
    q_scalar_mul_res3 *= scalar;
    TEST(isEqual(q_scalar_mul_check, q_scalar_mul_res3));

    // quaternion inverse
    q = q_check.inversed();
    TEST(fabsf(q_check(0) - q(0)) < eps);
    TEST(fabsf(q_check(1) + q(1)) < eps);
    TEST(fabsf(q_check(2) + q(2)) < eps);
    TEST(fabsf(q_check(3) + q(3)) < eps);

    q = q_check;
    q.invert();
    TEST(fabsf(q_check(0) - q(0)) < eps);
    TEST(fabsf(q_check(1) + q(1)) < eps);
    TEST(fabsf(q_check(2) + q(2)) < eps);
    TEST(fabsf(q_check(3) + q(3)) < eps);

    // rotate quaternion (nonzero rotation)
    Quatf qI(1.0f, 0.0f, 0.0f, 0.0f);
    Vector<float, 3> rot;
    rot(0) = 1.0f;
    rot(1) = rot(2) = 0.0f;
    qI.rotate(rot);
    Quatf q_true(cosf(1.0f / 2), sinf(1.0f / 2), 0.0f, 0.0f);
    TEST(fabsf(qI(0) - q_true(0)) < eps);
    TEST(fabsf(qI(1) - q_true(1)) < eps);
    TEST(fabsf(qI(2) - q_true(2)) < eps);
    TEST(fabsf(qI(3) - q_true(3)) < eps);

    // rotate quaternion (zero rotation)
    qI = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    rot(0) = 0.0f;
    rot(1) = rot(2) = 0.0f;
    qI.rotate(rot);
    q_true = Quatf(cosf(0.0f), sinf(0.0f), 0.0f, 0.0f);
    TEST(fabsf(qI(0) - q_true(0)) < eps);
    TEST(fabsf(qI(1) - q_true(1)) < eps);
    TEST(fabsf(qI(2) - q_true(2)) < eps);
    TEST(fabsf(qI(3) - q_true(3)) < eps);

    // get rotation axis from quaternion (nonzero rotation)
    q = Quatf(cosf(1.0f / 2), 0.0f, sinf(1.0f / 2), 0.0f);
    rot = q.to_axis_angle();
    TEST(fabsf(rot(0)) < eps);
    TEST(fabsf(rot(1) -1.0f) < eps);
    TEST(fabsf(rot(2)) < eps);

    // get rotation axis from quaternion (zero rotation)
    q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    rot = q.to_axis_angle();
    TEST(fabsf(rot(0)) < eps);
    TEST(fabsf(rot(1)) < eps);
    TEST(fabsf(rot(2)) < eps);

    // from axis angle (zero rotation)
    rot(0) = rot(1) = rot(2) = 0.0f;
    q.from_axis_angle(rot, 0.0f);
    q_true = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    TEST(fabsf(q(0) - q_true(0)) < eps);
    TEST(fabsf(q(1) - q_true(1)) < eps);
    TEST(fabsf(q(2) - q_true(2)) < eps);
    TEST(fabsf(q(3) - q_true(3)) < eps);

};

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
