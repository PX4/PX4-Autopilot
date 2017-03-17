#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

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
    TEST(isEqual(q0.unit(), q0.normalized()));

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

    // dcm renormalize
    Dcmf A = eye<float, 3>();
    Dcmf R(euler_check);
    for (size_t i = 0; i < 1000; i++) {
        A = R * A;
    }

    A.renormalize();
    float err = 0.0f;

    for (auto & row : A._data) {
        Vector3f rvec(row);
        err += fabs(1.0f - rvec.length());
    }
    TEST(err < eps);

    // constants
    double deg2rad = M_PI / 180.0;
    double rad2deg = 180.0 / M_PI;

    // euler dcm round trip check
    for (int roll = -90; roll <= 90; roll += 90) {
        for (int pitch = -90; pitch <= 90; pitch += 90) {
            for (int yaw = -179; yaw <= 180; yaw += 90) {
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

                if (yaw_expected < -180) {
                    yaw_expected += 360;
                }

                if (yaw_expected > 180) {
                    yaw_expected -= 360;
                }

                //printf("roll:%d pitch:%d yaw:%d\n", roll, pitch, yaw);
                Euler<double> euler_expected(
                    deg2rad * double(roll_expected),
                    deg2rad * double(pitch),
                    deg2rad * double(yaw_expected));
                Euler<double> euler(
                    deg2rad * double(roll),
                    deg2rad * double(pitch),
                    deg2rad * double(yaw));
                Dcm<double> dcm_from_euler(euler);
                //dcm_from_euler.print();
                Euler<double> euler_out(dcm_from_euler);
                TEST(isEqual(rad2deg * euler_expected, rad2deg * euler_out));

                Eulerf eulerf_expected(
                    float(deg2rad)*float(roll_expected),
                    float(deg2rad)*float(pitch),
                    float(deg2rad)*float(yaw_expected));
                Eulerf eulerf(float(deg2rad)*float(roll),
                              float(deg2rad)*float(pitch),
                              float(deg2rad)*float(yaw));
                Dcm<float> dcm_from_eulerf;
                dcm_from_eulerf = eulerf;
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

    // quaternion derivative in frame 1
    Quatf q1(0, 1, 0, 0);
    Vector<float, 4> q1_dot1 = q1.derivative1(Vector3f(1, 2, 3));
    float data_q_dot1_check[] = { -0.5f, 0.0f, 1.5f, -1.0f};
    Vector<float, 4> q1_dot1_check(data_q_dot1_check);
    TEST(isEqual(q1_dot1, q1_dot1_check));

    // quaternion derivative in frame 2
    Vector<float, 4> q1_dot2 = q1.derivative2(Vector3f(1, 2, 3));
    float data_q_dot2_check[] = { -0.5f, 0.0f, -1.5f, 1.0f};
    Vector<float, 4> q1_dot2_check(data_q_dot2_check);
    TEST(isEqual(q1_dot2, q1_dot2_check));

    // quaternion product
    Quatf q_prod_check(
        0.93394439f, 0.0674002f, 0.20851f, 0.28236266f);
    TEST(isEqual(q_prod_check, q_check * q_check));
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
    TEST(fabs(q_check(0) - q(0)) < eps);
    TEST(fabs(q_check(1) + q(1)) < eps);
    TEST(fabs(q_check(2) + q(2)) < eps);
    TEST(fabs(q_check(3) + q(3)) < eps);

    q = q_check;
    q.invert();
    TEST(fabs(q_check(0) - q(0)) < eps);
    TEST(fabs(q_check(1) + q(1)) < eps);
    TEST(fabs(q_check(2) + q(2)) < eps);
    TEST(fabs(q_check(3) + q(3)) < eps);

    // non-unit quaternion invese
    Quatf qI(1.0f, 0.0f, 0.0f, 0.0f);
    Quatf q_nonunit(0.1f, 0.2f, 0.3f, 0.4f);
    TEST(isEqual(qI, q_nonunit*q_nonunit.inversed()));

    // rotate quaternion (nonzero rotation)
    Vector<float, 3> rot;
    rot(0) = 1.0f;
    rot(1) = rot(2) = 0.0f;
    qI.rotate(rot);
    Quatf q_true(cos(1.0f / 2), sin(1.0f / 2), 0.0f, 0.0f);
    TEST(fabs(qI(0) - q_true(0)) < eps);
    TEST(fabs(qI(1) - q_true(1)) < eps);
    TEST(fabs(qI(2) - q_true(2)) < eps);
    TEST(fabs(qI(3) - q_true(3)) < eps);

    // rotate quaternion (zero rotation)
    qI = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    rot(0) = 0.0f;
    rot(1) = rot(2) = 0.0f;
    qI.rotate(rot);
    q_true = Quatf(cos(0.0f), sin(0.0f), 0.0f, 0.0f);
    TEST(fabs(qI(0) - q_true(0)) < eps);
    TEST(fabs(qI(1) - q_true(1)) < eps);
    TEST(fabs(qI(2) - q_true(2)) < eps);
    TEST(fabs(qI(3) - q_true(3)) < eps);

    // get rotation axis from quaternion (nonzero rotation)
    q = Quatf(cos(1.0f / 2), 0.0f, sin(1.0f / 2), 0.0f);
    rot = q.to_axis_angle();
    TEST(fabs(rot(0)) < eps);
    TEST(fabs(rot(1) - 1.0f) < eps);
    TEST(fabs(rot(2)) < eps);

    // get rotation axis from quaternion (zero rotation)
    q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    rot = q.to_axis_angle();
    TEST(fabs(rot(0)) < eps);
    TEST(fabs(rot(1)) < eps);
    TEST(fabs(rot(2)) < eps);

    // from axis angle (zero rotation)
    rot(0) = rot(1) = rot(2) = 0.0f;
    q.from_axis_angle(rot, 0.0f);
    q_true = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    TEST(fabs(q(0) - q_true(0)) < eps);
    TEST(fabs(q(1) - q_true(1)) < eps);
    TEST(fabs(q(2) - q_true(2)) < eps);
    TEST(fabs(q(3) - q_true(3)) < eps);

    // Quaternion initialisation per array
    float q_array[] = {0.9833f, -0.0343f, -0.1060f, -0.1436f};
    Quaternion<float>q_from_array(q_array);

    for (size_t i = 0; i < 4; i++) {
        TEST(fabs(q_from_array(i) - q_array[i]) < eps);
    }

    // axis angle
    AxisAnglef aa_true(Vector3f(1.0f, 2.0f, 3.0f));
    TEST(isEqual(aa_true, Vector3f(1.0f, 2.0f, 3.0f)));
    AxisAnglef aa_empty;
    TEST(isEqual(aa_empty, AxisAnglef(0.0f, 0.0f, 0.0f)));
    float aa_data[] =  {4.0f, 5.0f, 6.0f};
    AxisAnglef aa_data_init(aa_data);
    TEST(isEqual(aa_data_init, AxisAnglef(4.0f, 5.0f, 6.0f)));

    AxisAnglef aa_norm_check(Vector3f(0.0f, 0.0f, 0.0f));
    TEST(isEqual(aa_norm_check.axis(), Vector3f(1, 0, 0)));
    TEST(isEqualF(aa_norm_check.angle(), 0.0f));

    q = Quatf(-0.29555112749297824f, 0.25532186f,  0.51064372f,  0.76596558f);
    TEST(isEqual(q.imag(), Vector3f(0.25532186f,  0.51064372f,  0.76596558f)));

    // from dcm
    TEST(isEqual(Eulerf(q.from_dcm(Dcmf(q))), Eulerf(q)));

    // to dcm
    TEST(isEqual(Dcmf(q), q.to_dcm()));

    // conjugate
    Vector3f v1(1.5f, 2.2f, 3.2f);
    TEST(isEqual(q.conjugate_inversed(v1), Dcmf(q)*v1));
    TEST(isEqual(q.conjugate(v1), Dcmf(q).T()*v1));

    AxisAnglef aa_q_init(q);
    TEST(isEqual(aa_q_init, AxisAnglef(1.0f, 2.0f, 3.0f)));

    AxisAnglef aa_euler_init(Eulerf(0.0f, 0.0f, 0.0f));
    TEST(isEqual(aa_euler_init, Vector3f(0.0f, 0.0f, 0.0f)));

    Dcmf dcm_aa_check = AxisAnglef(dcm_check);
    TEST(isEqual(dcm_aa_check, dcm_check));

    AxisAnglef aa_axis_angle_init(Vector3f(1.0f, 2.0f, 3.0f), 3.0f);
    TEST(isEqual(aa_axis_angle_init, Vector3f(0.80178373f, 1.60356745f, 2.40535118f)));
    TEST(isEqual(aa_axis_angle_init.axis(), Vector3f(0.26726124f,  0.53452248f,  0.80178373f)));
    TEST(isEqualF(aa_axis_angle_init.angle(), 3.0f));
    TEST(isEqual(Quatf((AxisAnglef(Vector3f(0.0f, 0.0f, 1.0f), 0.0f))),
                 Quatf(1.0f, 0.0f, 0.0f, 0.0f)));

    // check consistentcy of quaternion and dcm product
    Dcmf dcm3(Eulerf(1, 2, 3));
    Dcmf dcm4(Eulerf(4, 5, 6));
    Dcmf dcm34 = dcm3 * dcm4;
    TEST(isEqual(Eulerf(Quatf(dcm4)*Quatf(dcm3)), Eulerf(dcm34)));

    // check corner cases of matrix to quaternion conversion
    q = Quatf(0,1,0,0); // 180 degree rotation around the x axis
    R = Dcmf(q);
    TEST(isEqual(q, Quatf(R)));
    q = Quatf(0,0,1,0); // 180 degree rotation around the y axis
    R = Dcmf(q);
    TEST(isEqual(q, Quatf(R)));
    q = Quatf(0,0,0,1); // 180 degree rotation around the z axis
    R = Dcmf(q);
    TEST(isEqual(q, Quatf(R)));


    // Quaternion copyTo
    q = Quatf(1, 2, 3, 4);
    float dst[4] = {};
    q.copyTo(dst);
    TEST(fabs(q(0) - dst[0]) < eps);
    TEST(fabs(q(1) - dst[1]) < eps);
    TEST(fabs(q(2) - dst[2]) < eps);
    TEST(fabs(q(3) - dst[3]) < eps);

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
