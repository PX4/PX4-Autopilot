/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <matrix/math.hpp>

using namespace matrix;

TEST(MatrixAttitudeTest, Attitude)
{
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
	EXPECT_EQ(euler_check, Vector3f(0.1f, 0.2f, 0.3f));

	// euler default ctor
	Eulerf e;
	Eulerf e_zero = zeros<float, 3, 1>();
	EXPECT_EQ(e, e_zero);
	EXPECT_EQ(e, e);

	// euler vector ctor
	Vector3f v(0.1f, 0.2f, 0.3f);
	Eulerf euler_copy(v);
	EXPECT_EQ(euler_copy, euler_check);

	// quaternion ctor
	Quatf q0(1, 2, 3, 4);
	Quatf q(q0);
	EXPECT_FLOAT_EQ(q(0), 1);
	EXPECT_FLOAT_EQ(q(1), 2);
	EXPECT_FLOAT_EQ(q(2), 3);
	EXPECT_FLOAT_EQ(q(3), 4);

	// quaternion ctor: vector to vector
	// identity test
	Quatf quat_v(v, v);
	EXPECT_EQ(quat_v.rotateVector(v), v);
	// random test (vector norm can not be preserved with a pure rotation)
	Vector3f v1(-80.1f, 1.5f, -6.89f);
	quat_v = Quatf(v1, v);
	EXPECT_EQ(quat_v.rotateVector(v1).normalized() * v.norm(), v);
	// special 180 degree case 1
	v1 = Vector3f(0.f, 1.f, 1.f);
	quat_v = Quatf(v1, -v1);
	EXPECT_EQ(quat_v.rotateVector(v1), -v1);
	// special 180 degree case 2
	v1 = Vector3f(1.f, 2.f, 0.f);
	quat_v = Quatf(v1, -v1);
	EXPECT_EQ(quat_v.rotateVector(v1), -v1);
	// special 180 degree case 3
	v1 = Vector3f(0.f, 0.f, 1.f);
	quat_v = Quatf(v1, -v1);
	EXPECT_EQ(quat_v.rotateVector(v1), -v1);
	// special 180 degree case 4
	v1 = Vector3f(1.f, 1.f, 1.f);
	quat_v = Quatf(v1, -v1);
	EXPECT_EQ(quat_v.rotateVector(v1), -v1);

	// quat normalization
	q.normalize();
	EXPECT_EQ(q, Quatf(0.18257419f,  0.36514837f, 0.54772256f,  0.73029674f));
	EXPECT_EQ(q0.unit(), q);
	EXPECT_EQ(q0.unit(), q0.normalized());

	// quat default ctor
	q = Quatf();
	EXPECT_EQ(q, Quatf(1, 0, 0, 0));

	// quaternion exponential with v=0
	v = Vector3f();
	q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	Dcmf M = Dcmf() * 0.5f;
	EXPECT_EQ(q, Quatf::expq(v));
	EXPECT_EQ(M, Quatf::inv_r_jacobian(v));

	// quaternion exponential with small v
	v = Vector3f(0.001f, 0.002f, -0.003f);
	q = Quatf(0.999993000008167f, 0.000999997666668f,
		  0.001999995333337f, -0.002999993000005f);
	{
		float M_data[] =  {
			0.499997833331311f, 0.001500333333644f,  0.000999499999533f,
			-0.001499666666356f, 0.499998333331778f, -0.000501000000933f,
			-0.001000500000467f, 0.000498999999067f,  0.499999166665889f
		};
		M = Dcmf(M_data);
	}
	EXPECT_EQ(q, Quatf::expq(v));
	EXPECT_EQ(M, Quatf::inv_r_jacobian(v));

	// quaternion exponential with v
	v = Vector3f(1.0f, -2.0f, 3.0f);
	q = Quatf(-0.825299062075259f, -0.150921327219964f,
		  0.301842654439929f, -0.452763981659893f);
	{
		float M_data[] =  {
			2.574616981530584f, -1.180828156687602f, -1.478757764968596f,
			1.819171843312398f,  2.095859216561988f,  0.457515529937193f,
			0.521242235031404f,  1.457515529937193f,  1.297929608280994f
		};
		M = Dcmf(M_data);
	}
	EXPECT_EQ(q, Quatf::expq(v));
	EXPECT_EQ(M, Quatf::inv_r_jacobian(v));

	// quaternion kinematic update
	q = Quatf();
	float h = 0.001f;  // sampling time [s]
	Vector3f w_B = Vector3f(0.1f, 0.2f, 0.3f); // body rate in body frame
	Quatf qa = q + 0.5f * h * q.derivative1(w_B);
	qa.normalize();
	Quatf qb = q * Quatf::expq(0.5f * h * w_B);
	EXPECT_EQ(qa, qb);

	// euler to quaternion
	q = Quatf(euler_check);
	EXPECT_EQ(q, q_check);

	// euler to dcm
	Dcmf dcm(euler_check);
	EXPECT_EQ(dcm, dcm_check);

	// quaternion to euler
	Eulerf e1(q_check);
	EXPECT_EQ(e1, euler_check);

	// quaternion to dcm
	Dcmf dcm1(q_check);
	EXPECT_EQ(dcm1, dcm_check);
	// quaternion z-axis unit base vector
	Vector3f q_z = q_check.dcm_z();
	Vector3f R_z(dcm_check(0, 2), dcm_check(1, 2), dcm_check(2, 2));
	EXPECT_EQ(q_z, R_z);

	// dcm default ctor
	Dcmf dcm2;
	SquareMatrix<float, 3> I = eye<float, 3>();
	EXPECT_EQ(dcm2, I);

	// dcm to euler
	Eulerf e2(dcm_check);
	EXPECT_EQ(e2, euler_check);

	// dcm to quaterion
	Quatf q2(dcm_check);
	EXPECT_EQ(q2, q_check);

	// dcm renormalize
	Dcmf A = eye<float, 3>();
	Dcmf R(euler_check);

	for (size_t i = 0; i < 1000; i++) {
		A = R * A;
	}

	A.renormalize();

	for (size_t r = 0; r < 3; r++) {
		Vector3f rvec(matrix::Matrix<float, 1, 3>(A.row(r)).transpose());
		EXPECT_FLOAT_EQ(1.0f, rvec.length());
	}

	// constants
	double deg2rad = M_PI / 180.0;
	double rad2deg = 180.0 / M_PI;

	// euler dcm round trip check
	for (double roll = -90; roll <= 90; roll += 90) {
		for (double pitch = -90; pitch <= 90; pitch += 90) {
			for (double yaw = -179; yaw <= 180; yaw += 90) {
				// note if theta = pi/2, then roll is set to zero
				double roll_expected = roll;
				double yaw_expected = yaw;

				if (isEqualF(pitch, 90.0)) {
					roll_expected = 0;
					yaw_expected = yaw - roll;

				} else if (isEqualF(pitch, -90.0)) {
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
					deg2rad * roll_expected,
					deg2rad * pitch,
					deg2rad * yaw_expected);
				Euler<double> euler(
					deg2rad * roll,
					deg2rad * pitch,
					deg2rad * yaw);
				Dcm<double> dcm_from_euler(euler);
				//dcm_from_euler.print();
				Euler<double> euler_out(dcm_from_euler);
				EXPECT_EQ(rad2deg * euler_expected, rad2deg * euler_out);

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
				EXPECT_EQ(float(rad2deg)*eulerf_expected,
					  float(rad2deg)*euler_outf);
			}
		}
	}

	// quaterion copy ctors
	float data_v4[] = {1, 2, 3, 4};
	Vector4f v4(data_v4);
	Quatf q_from_v(v4);
	EXPECT_EQ(q_from_v, v4);

	Matrix<float, 4, 1> m4(data_v4);
	Quatf q_from_m(m4);
	EXPECT_EQ(q_from_m, m4);

	// quaternion derivative in frame 1
	Quatf q1(0, 1, 0, 0);
	Vector4f q1_dot1 = q1.derivative1(Vector3f(1, 2, 3));
	Vector4f q1_dot1_check(-0.5f, 0.0f, -1.5f, 1.0f);
	EXPECT_EQ(q1_dot1, q1_dot1_check);

	// quaternion derivative in frame 2
	Vector4f q1_dot2 = q1.derivative2(Vector3f(1, 2, 3));
	Vector4f q1_dot2_check(-0.5f, 0.0f, 1.5f, -1.0f);
	EXPECT_EQ(q1_dot2, q1_dot2_check);

	// quaternion product
	Quatf q_prod_check(
		0.93394439f, 0.0674002f, 0.20851f, 0.28236266f);
	EXPECT_EQ(q_prod_check, q_check * q_check);
	q_check *= q_check;
	EXPECT_EQ(q_prod_check, q_check);

	// Quaternion scalar multiplication
	float scalar = 0.5;
	Quatf q_scalar_mul(1.0f, 2.0f, 3.0f, 4.0f);
	Quatf q_scalar_mul_check(1.0f * scalar, 2.0f * scalar,
				 3.0f * scalar,  4.0f * scalar);
	Quatf q_scalar_mul_res = scalar * q_scalar_mul;
	EXPECT_EQ(q_scalar_mul_check, q_scalar_mul_res);
	Quatf q_scalar_mul_res2 = q_scalar_mul * scalar;
	EXPECT_EQ(q_scalar_mul_check, q_scalar_mul_res2);
	Quatf q_scalar_mul_res3(q_scalar_mul);
	q_scalar_mul_res3 *= scalar;
	EXPECT_EQ(q_scalar_mul_check, q_scalar_mul_res3);

	// quaternion inverse
	q = q_check.inversed();
	EXPECT_FLOAT_EQ(q_check(0), q(0));
	EXPECT_FLOAT_EQ(q_check(1), -q(1));
	EXPECT_FLOAT_EQ(q_check(2), -q(2));
	EXPECT_FLOAT_EQ(q_check(3), -q(3));

	q = q_check;
	q.invert();
	EXPECT_FLOAT_EQ(q_check(0), q(0));
	EXPECT_FLOAT_EQ(q_check(1), -q(1));
	EXPECT_FLOAT_EQ(q_check(2), -q(2));
	EXPECT_FLOAT_EQ(q_check(3), -q(3));

	// quaternion canonical
	Quatf q_non_canonical_1(-0.7f, 0.4f, 0.3f, -0.3f);
	Quatf q_canonical_1(0.7f, -0.4f, -0.3f, 0.3f);
	Quatf q_canonical_ref_1(0.7f, -0.4f, -0.3f, 0.3f);
	EXPECT_EQ(q_non_canonical_1.canonical(), q_canonical_ref_1);
	EXPECT_EQ(q_canonical_1.canonical(), q_canonical_ref_1);
	q_non_canonical_1.canonicalize();
	q_canonical_1.canonicalize();
	EXPECT_EQ(q_non_canonical_1, q_canonical_ref_1);
	EXPECT_EQ(q_canonical_1, q_canonical_ref_1);

	Quatf q_non_canonical_2(0.0f, -1.0f, 0.0f, 0.0f);
	Quatf q_canonical_2(0.0f, 1.0f, 0.0f, 0.0f);
	Quatf q_canonical_ref_2(0.0f, 1.0f, 0.0f, 0.0f);
	EXPECT_EQ(q_non_canonical_2.canonical(), q_canonical_ref_2);
	EXPECT_EQ(q_canonical_2.canonical(), q_canonical_ref_2);
	q_non_canonical_2.canonicalize();
	q_canonical_2.canonicalize();
	EXPECT_EQ(q_non_canonical_2, q_canonical_ref_2);
	EXPECT_EQ(q_canonical_2, q_canonical_ref_2);

	Quatf q_non_canonical_3(0.0f, 0.0f, -1.0f, 0.0f);
	Quatf q_canonical_3(0.0f, 0.0f, 1.0f, 0.0f);
	Quatf q_canonical_ref_3(0.0f, 0.0f, 1.0f, 0.0f);
	EXPECT_EQ(q_non_canonical_3.canonical(), q_canonical_ref_3);
	EXPECT_EQ(q_canonical_3.canonical(), q_canonical_ref_3);
	q_non_canonical_3.canonicalize();
	q_canonical_3.canonicalize();
	EXPECT_EQ(q_non_canonical_3, q_canonical_ref_3);
	EXPECT_EQ(q_canonical_3, q_canonical_ref_3);

	Quatf q_non_canonical_4(0.0f, 0.0f, 0.0f, -1.0f);
	Quatf q_canonical_4(0.0f, 0.0f, 0.0f, 1.0f);
	Quatf q_canonical_ref_4(0.0f, 0.0f, 0.0f, 1.0f);
	EXPECT_EQ(q_non_canonical_4.canonical(), q_canonical_ref_4);
	EXPECT_EQ(q_canonical_4.canonical(), q_canonical_ref_4);
	q_non_canonical_4.canonicalize();
	q_canonical_4.canonicalize();
	EXPECT_EQ(q_non_canonical_4, q_canonical_ref_4);
	EXPECT_EQ(q_canonical_4, q_canonical_ref_4);

	Quatf q_non_canonical_5(0.0f, 0.0f, 0.0f, 0.0f);
	Quatf q_canonical_5(0.0f, 0.0f, 0.0f, 0.0f);
	Quatf q_canonical_ref_5(0.0f, 0.0f, 0.0f, 0.0f);
	EXPECT_EQ(q_non_canonical_5.canonical(), q_canonical_ref_5);
	EXPECT_EQ(q_canonical_5.canonical(), q_canonical_ref_5);
	q_non_canonical_5.canonicalize();
	q_canonical_5.canonicalize();
	EXPECT_EQ(q_non_canonical_5, q_canonical_ref_5);
	EXPECT_EQ(q_canonical_5, q_canonical_ref_5);

	// quaternion setIdentity
	Quatf q_nonIdentity(-0.7f, 0.4f, 0.5f, -0.3f);
	q_nonIdentity.setIdentity();
	EXPECT_EQ(q_nonIdentity, Quatf());

	// non-unit quaternion invese
	Quatf q_nonunit(0.1f, 0.2f, 0.3f, 0.4f);
	EXPECT_EQ(q_nonunit * q_nonunit.inversed(), Quatf());

	// rotate quaternion (nonzero rotation)
	Vector3f rot(1.f, 0.f, 0.f);
	Quatf q_test;
	q_test.rotate(rot);
	Quatf q_true(std::cos(1.0f / 2), sin(1.0f / 2), 0.0f, 0.0f);
	EXPECT_EQ(q_test, q_true);

	// rotate quaternion (zero rotation)
	rot(0) = rot(1) = rot(2) = 0.0f;
	q_test = Quatf();
	q_test.rotate(rot);
	q_true = Quatf(std::cos(0.0f), sin(0.0f), 0.0f, 0.0f);
	EXPECT_EQ(q_test, q_true);

	// rotate quaternion (random non-commutating rotation)
	q = Quatf(AxisAnglef(5.1f, 3.2f, 8.4f));
	rot = Vector3f(1.1f, 2.5f, 3.8f);
	q.rotate(rot);
	q_true = Quatf(0.3019f, 0.2645f, 0.2268f, 0.8874f);
	EXPECT_EQ(q, q_true);

	// get rotation axis from quaternion (nonzero rotation)
	q = Quatf(std::cos(1.0f / 2), 0.0f, sin(1.0f / 2), 0.0f);
	rot = AxisAnglef(q);
	EXPECT_FLOAT_EQ(rot(0), 0.0f);
	EXPECT_FLOAT_EQ(rot(1), 1.0f);
	EXPECT_FLOAT_EQ(rot(2), 0.0f);

	// get rotation axis from quaternion (zero rotation)
	q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	rot = AxisAnglef(q);
	EXPECT_FLOAT_EQ(rot(0), 0.0f);
	EXPECT_FLOAT_EQ(rot(1), 0.0f);
	EXPECT_FLOAT_EQ(rot(2), 0.0f);

	// from axis angle (zero rotation)
	rot(0) = rot(1) = rot(2) = 0.0f;
	q = Quatf(AxisAnglef(rot));
	q_true = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	EXPECT_EQ(q, q_true);

	// from axis angle, with length of vector the rotation
	float n = float(std::sqrt(4 * M_PI_F * M_PI_F / 3));
	q = AxisAnglef(n, n, n);
	EXPECT_EQ(q, Quatf(-1, 0, 0, 0));
	q = AxisAnglef(0, 0, 0);
	EXPECT_EQ(q, Quatf(1, 0, 0, 0));

	// Quaternion initialisation per array
	float q_array[] = {0.9833f, -0.0343f, -0.1060f, -0.1436f};
	Quaternion<float>q_from_array(q_array);

	for (size_t i = 0; i < 4; i++) {
		EXPECT_FLOAT_EQ(q_from_array(i), q_array[i]);
	}

	// axis angle
	AxisAnglef aa_true(Vector3f(1.0f, 2.0f, 3.0f));
	EXPECT_EQ(aa_true, Vector3f(1.0f, 2.0f, 3.0f));
	AxisAnglef aa_empty;
	EXPECT_EQ(aa_empty, AxisAnglef(0.0f, 0.0f, 0.0f));
	float aa_data[] =  {4.0f, 5.0f, 6.0f};
	AxisAnglef aa_data_init(aa_data);
	EXPECT_EQ(aa_data_init, AxisAnglef(4.0f, 5.0f, 6.0f));

	AxisAnglef aa_norm_check(Vector3f(0.0f, 0.0f, 0.0f));
	EXPECT_EQ(aa_norm_check.axis(), Vector3f(1, 0, 0));
	EXPECT_FLOAT_EQ(aa_norm_check.angle(), 0.0f);

	q = Quatf(-0.29555112749297824f, 0.25532186f,  0.51064372f,  0.76596558f);
	float r_array[9] = {-0.6949206f, 0.713521f, 0.089292854f, -0.19200698f, -0.30378509f, 0.93319237f, 0.69297814f, 0.63134968f, 0.34810752f};
	R = Dcmf(r_array);
	EXPECT_EQ(q.imag(), Vector3f(0.25532186f,  0.51064372f,  0.76596558f));

	// from dcm
	EXPECT_EQ(Quatf(R), q);
	EXPECT_EQ(Quatf(Dcmf(q)), q);

	// to dcm
	EXPECT_EQ(Dcmf(q), R);
	EXPECT_EQ(Dcmf(Quatf(R)), R);

	// conjugate
	v = Vector3f(1.5f, 2.2f, 3.2f);
	EXPECT_EQ(q.rotateVectorInverse(v1), Dcmf(q).T()*v1);
	EXPECT_EQ(q.rotateVector(v1), Dcmf(q)*v1);

	AxisAnglef aa_q_init(q);
	EXPECT_EQ(aa_q_init, AxisAnglef(1.0f, 2.0f, 3.0f));

	AxisAnglef aa_euler_init(Eulerf(0.0f, 0.0f, 0.0f));
	EXPECT_EQ(aa_euler_init, Vector3f(0.0f, 0.0f, 0.0f));

	Dcmf dcm_aa_check = AxisAnglef(dcm_check);
	EXPECT_EQ(dcm_aa_check, dcm_check);

	AxisAnglef aa_axis_angle_init(Vector3f(1.0f, 2.0f, 3.0f), 3.0f);
	EXPECT_EQ(aa_axis_angle_init, Vector3f(0.80178373f, 1.60356745f, 2.40535118f));
	EXPECT_EQ(aa_axis_angle_init.axis(), Vector3f(0.26726124f,  0.53452248f,  0.80178373f));
	EXPECT_EQ(aa_axis_angle_init.angle(), 3.0f);
	EXPECT_EQ(Quatf((AxisAnglef(Vector3f(0.0f, 0.0f, 1.0f), 0.0f))),
		  Quatf(1.0f, 0.0f, 0.0f, 0.0f));


	// check consistentcy of quaternion and dcm product
	Dcmf dcm3(Eulerf(1, 2, 3));
	Dcmf dcm4(Eulerf(4, 5, 6));
	Dcmf dcm34 = dcm3 * dcm4;
	EXPECT_EQ(Eulerf(Quatf(dcm3)*Quatf(dcm4)), Eulerf(dcm34));

	// check corner cases of matrix to quaternion conversion
	q = Quatf(0, 1, 0, 0); // 180 degree rotation around the x axis
	R = Dcmf(q);
	EXPECT_EQ(q, Quatf(R));
	q = Quatf(0, 0, 1, 0); // 180 degree rotation around the y axis
	R = Dcmf(q);
	EXPECT_EQ(q, Quatf(R));
	q = Quatf(0, 0, 0, 1); // 180 degree rotation around the z axis
	R = Dcmf(q);
	EXPECT_EQ(q, Quatf(R));
}
