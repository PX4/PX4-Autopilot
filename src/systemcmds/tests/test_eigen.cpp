/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file test_eigen.cpp
 *
 * Eigen test (based of mathlib test)
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#include <px4_eigen.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "tests.h"

namespace Eigen
{
typedef Matrix<float, 10, 1> Vector10f;
}

static constexpr size_t OPERATOR_ITERATIONS = 60000;

#define TEST_OP(_title, _op)							 								\
	{ 																						\
		const hrt_abstime t0 = hrt_absolute_time(); 										\
		for (size_t j = 0; j < OPERATOR_ITERATIONS; j++) { 									\
			_op; 																			\
		} 																					\
		printf(_title ": %.6fus\n", static_cast<double>(hrt_absolute_time() - t0) / OPERATOR_ITERATIONS); \
	}

#define VERIFY_OP(_title, _op, __OP_TEST__)		 										\
	{ 																						\
		_op; 																				\
		if(!(__OP_TEST__)) {																\
			printf(_title " Failed! ("#__OP_TEST__")\n");										\
		}																					\
	}

#define TEST_OP_VERIFY(_title, _op, __OP_TEST__) 										\
	VERIFY_OP(_title, _op, __OP_TEST__) 												\
	TEST_OP(_title, _op)

/**
* @brief
*	Prints an Eigen::Matrix to stdout
**/
template<typename T>
void printEigen(const Eigen::MatrixBase<T> &b)
{
	for (int i = 0; i < b.rows(); ++i) {
		printf("(");

		for (int j = 0; j < b.cols(); ++i) {
			if (j > 0) {
				printf(",");
			}

			printf("%.3f", static_cast<double>(b(i, j)));
		}

		printf(")%s\n", i + 1 < b.rows() ? "," : "");
	}
}

/**
* @brief
*	Construct new Eigen::Quaternion from euler angles
**/
template<typename T>
Eigen::Quaternion<T> quatFromEuler(const T roll, const T pitch, const T yaw)
{
	Eigen::AngleAxis<T> rollAngle(roll, Eigen::Matrix<T, 3, 1>::UnitZ());
	Eigen::AngleAxis<T> yawAngle(yaw, Eigen::Matrix<T, 3, 1>::UnitY());
	Eigen::AngleAxis<T> pitchAngle(pitch, Eigen::Matrix<T, 3, 1>::UnitX());
	return rollAngle * yawAngle * pitchAngle;
}


int test_eigen(int argc, char *argv[])
{
	int rc = 0;
	warnx("testing eigen");

	{
		Eigen::Vector2f v;
		Eigen::Vector2f v1(1.0f, 2.0f);
		Eigen::Vector2f v2(1.0f, -1.0f);
		float data[2] = {1.0f, 2.0f};
		TEST_OP("Constructor Vector2f()", Eigen::Vector2f v3);
		TEST_OP_VERIFY("Constructor Vector2f(Vector2f)", Eigen::Vector2f v3(v1), v3.isApprox(v1));
		TEST_OP_VERIFY("Constructor Vector2f(float[])", Eigen::Vector2f v3(data), v3[0] == data[0] && v3[1] == data[1]);
		TEST_OP_VERIFY("Constructor Vector2f(float, float)", Eigen::Vector2f v3(1.0f, 2.0f), v3(0) == 1.0f && v3(1) == 2.0f);
		TEST_OP_VERIFY("Vector2f = Vector2f", v = v1, v.isApprox(v1));
		TEST_OP_VERIFY("Vector2f + Vector2f", v + v1, v.isApprox(v1 + v1));
		TEST_OP_VERIFY("Vector2f - Vector2f", v - v1, v.isApprox(v1));
		TEST_OP_VERIFY("Vector2f += Vector2f", v += v1, v.isApprox(v1 + v1));
		TEST_OP_VERIFY("Vector2f -= Vector2f", v -= v1, v.isApprox(v1));
		TEST_OP_VERIFY("Vector2f dot Vector2f", v.dot(v1), fabs(v.dot(v1) - 5.0f) <= FLT_EPSILON);
		//TEST_OP("Vector2f cross Vector2f", v1.cross(v2)); //cross product for 2d array?
	}

	{
		Eigen::Vector3f v;
		Eigen::Vector3f v1(1.0f, 2.0f, 0.0f);
		Eigen::Vector3f v2(1.0f, -1.0f, 2.0f);
		float data[3] = {1.0f, 2.0f, 3.0f};
		TEST_OP("Constructor Vector3f()", Eigen::Vector3f v3);
		TEST_OP("Constructor Vector3f(Vector3f)", Eigen::Vector3f v3(v1));
		TEST_OP("Constructor Vector3f(float[])", Eigen::Vector3f v3(data));
		TEST_OP("Constructor Vector3f(float, float, float)", Eigen::Vector3f v3(1.0f, 2.0f, 3.0f));
		TEST_OP("Vector3f = Vector3f", v = v1);
		TEST_OP("Vector3f + Vector3f", v + v1);
		TEST_OP("Vector3f - Vector3f", v - v1);
		TEST_OP("Vector3f += Vector3f", v += v1);
		TEST_OP("Vector3f -= Vector3f", v -= v1);
		TEST_OP("Vector3f * float", v1 * 2.0f);
		TEST_OP("Vector3f / float", v1 / 2.0f);
		TEST_OP("Vector3f *= float", v1 *= 2.0f);
		TEST_OP("Vector3f /= float", v1 /= 2.0f);
		TEST_OP("Vector3f dot Vector3f", v.dot(v1));
		TEST_OP("Vector3f cross Vector3f", v1.cross(v2));
		TEST_OP("Vector3f length", v1.norm());
		TEST_OP("Vector3f length squared", v1.squaredNorm());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
		// Need pragma here intead of moving variable out of TEST_OP and just reference because
		// TEST_OP measures performance of vector operations.
		TEST_OP("Vector<3> element read", volatile float a = v1(0));
		TEST_OP("Vector<3> element read direct", volatile float a = v1.data()[0]);
#pragma GCC diagnostic pop
		TEST_OP("Vector<3> element write", v1(0) = 1.0f);
		TEST_OP("Vector<3> element write direct", v1.data()[0] = 1.0f);
	}

	{
		Eigen::Vector4f v(0.0f, 0.0f, 0.0f, 0.0f);
		Eigen::Vector4f v1(1.0f, 2.0f, 0.0f, -1.0f);
		Eigen::Vector4f v2(1.0f, -1.0f, 2.0f, 0.0f);
		Eigen::Vector4f vres;
		float data[4] = {1.0f, 2.0f, 3.0f, 4.0f};
		TEST_OP("Constructor Vector<4>()", Eigen::Vector4f v3);
		TEST_OP("Constructor Vector<4>(Vector<4>)", Eigen::Vector4f v3(v1));
		TEST_OP("Constructor Vector<4>(float[])", Eigen::Vector4f v3(data));
		TEST_OP("Constructor Vector<4>(float, float, float, float)", Eigen::Vector4f v3(1.0f, 2.0f, 3.0f, 4.0f));
		TEST_OP("Vector<4> = Vector<4>", v = v1);
		TEST_OP("Vector<4> + Vector<4>", v + v1);
		TEST_OP("Vector<4> - Vector<4>", v - v1);
		//TEST_OP("Vector<4> += Vector<4>", v += v1);
		//TEST_OP("Vector<4> -= Vector<4>", v -= v1);
		TEST_OP("Vector<4> dot Vector<4>", v.dot(v1));
	}

	{
		Eigen::Vector10f v1;
		v1.Zero();
		float data[10];
		TEST_OP("Constructor Vector<10>()", Eigen::Vector10f v3);
		TEST_OP("Constructor Vector<10>(Vector<10>)", Eigen::Vector10f v3(v1));
		TEST_OP("Constructor Vector<10>(float[])", Eigen::Vector10f v3(data));
	}

	{
		Eigen::Matrix3f m1;
		m1.setIdentity();
		Eigen::Matrix3f m2;
		m2.setIdentity();
		Eigen::Vector3f v1(1.0f, 2.0f, 0.0f);
		TEST_OP("Matrix3f * Vector3f", m1 * v1);
		TEST_OP("Matrix3f + Matrix3f", m1 + m2);
		TEST_OP("Matrix3f * Matrix3f", m1 * m2);
	}

	{
		Eigen::Matrix<float, 10, 10> m1;
		m1.setIdentity();
		Eigen::Matrix<float, 10, 10> m2;
		m2.setIdentity();
		Eigen::Vector10f v1;
		v1.Zero();
		TEST_OP("Matrix<10, 10> * Vector<10>", m1 * v1);
		TEST_OP("Matrix<10, 10> + Matrix<10, 10>", m1 + m2);
		TEST_OP("Matrix<10, 10> * Matrix<10, 10>", m1 * m2);
	}

	{
		warnx("Nonsymmetric matrix operations test");
		// test nonsymmetric +, -, +=, -=

		const Eigen::Matrix<float, 2, 3> m1_orig =
			(Eigen::Matrix<float, 2, 3>() << 1, 3, 3,
			 4, 6, 6).finished();

		Eigen::Matrix<float, 2, 3> m1(m1_orig);

		Eigen::Matrix<float, 2, 3> m2;
		m2 << 2, 4, 6,
		8, 10, 12;

		Eigen::Matrix<float, 2, 3> m3;
		m3 << 3, 6, 9,
		12, 15, 18;

		if (m1 + m2 != m3) {
			warnx("Matrix<2, 3> + Matrix<2, 3> failed!");
			printEigen(m1 + m2);
			printf("!=\n");
			printEigen(m3);
			rc = 1;
		}

		if (m3 - m2 != m1) {
			warnx("Matrix<2, 3> - Matrix<2, 3> failed!");
			printEigen(m3 - m2);
			printf("!=\n");
			printEigen(m1);
			rc = 1;
		}

		m1 += m2;

		if (m1 != m3) {
			warnx("Matrix<2, 3> += Matrix<2, 3> failed!");
			printEigen(m1);
			printf("!=\n");
			printEigen(m3);
			rc = 1;
		}

		m1 -= m2;

		if (m1 != m1_orig) {
			warnx("Matrix<2, 3> -= Matrix<2, 3> failed!");
			printEigen(m1);
			printf("!=\n");
			printEigen(m1_orig);
			rc = 1;
		}

	}

	{
		// test conversion rotation matrix to quaternion and back
		Eigen::Matrix3f R_orig;
		Eigen::Matrix3f R;
		Eigen::Quaternionf q;
		float diff = 0.1f;
		float tol = 0.00001f;

		warnx("Quaternion transformation methods test.");

		for (float roll = -M_PI_F; roll <= M_PI_F; roll += diff) {
			for (float pitch = -M_PI_2_F; pitch <= M_PI_2_F; pitch += diff) {
				for (float yaw = -M_PI_F; yaw <= M_PI_F; yaw += diff) {
					R_orig.eulerAngles(roll, pitch, yaw);
					q = R_orig; //from_dcm
					R = q.toRotationMatrix();

					for (size_t i = 0; i < 3; i++) {
						for (size_t j = 0; j < 3; j++) {
							if (fabsf(R_orig(i, j) - R(i, j)) > 0.00001f) {
								warnx("Quaternion method 'from_dcm' or 'toRotationMatrix' outside tolerance!");
								rc = 1;
							}
						}
					}
				}
			}
		}

		// test against some known values
		tol = 0.0001f;
		Eigen::Quaternionf q_true = {1.0f, 0.0f, 0.0f, 0.0f};
		R_orig.setIdentity();
		q = R_orig;

		for (size_t i = 0; i < 4; i++) {
			if (!q.isApprox(q_true, tol)) {
				warnx("Quaternion method 'from_dcm()' outside tolerance!");
				rc = 1;
			}
		}

		q_true = quatFromEuler(0.3f, 0.2f, 0.1f);
		q = {0.9833f, 0.1436f, 0.1060f, 0.0343f};

		for (size_t i = 0; i < 4; i++) {
			if (!q.isApprox(q_true, tol)) {
				warnx("Quaternion method 'eulerAngles()' outside tolerance!");
				rc = 1;
			}
		}

		q_true = quatFromEuler(-1.5f, -0.2f, 0.5f);
		q = {0.7222f, -0.6391f, -0.2386f, 0.1142f};

		for (size_t i = 0; i < 4; i++) {
			if (!q.isApprox(q_true, tol)) {
				warnx("Quaternion method 'eulerAngles()' outside tolerance!");
				rc = 1;
			}
		}

		q_true = quatFromEuler(M_PI_2_F, -M_PI_2_F, -M_PI_F / 3);
		q = {0.6830f, 0.1830f, -0.6830f, 0.1830f};

		for (size_t i = 0; i < 4; i++) {
			if (!q.isApprox(q_true, tol)) {
				warnx("Quaternion method 'eulerAngles()' outside tolerance!");
				rc = 1;
			}
		}

	}

	{
		// test quaternion method "rotate" (rotate vector by quaternion)
		Eigen::Vector3f vector = {1.0f, 1.0f, 1.0f};
		Eigen::Vector3f vector_q;
		Eigen::Vector3f vector_r;
		Eigen::Quaternionf q;
		Eigen::Matrix3f R;
		float diff = 0.1f;
		float tol = 0.00001f;

		warnx("Quaternion vector rotation method test.");

		for (float roll = -M_PI_F; roll <= M_PI_F; roll += diff) {
			for (float pitch = -M_PI_2_F; pitch <= M_PI_2_F; pitch += diff) {
				for (float yaw = -M_PI_F; yaw <= M_PI_F; yaw += diff) {
					R.eulerAngles(roll, pitch, yaw);
					q = quatFromEuler(roll, pitch, yaw);
					vector_r = R * vector;
					vector_q = q._transformVector(vector);

					for (int i = 0; i < 3; i++) {
						if (fabsf(vector_r(i) - vector_q(i)) > tol) {
							warnx("Quaternion method 'rotate' outside tolerance");
							rc = 1;
						}
					}
				}
			}
		}

		// test some values calculated with matlab
		tol = 0.0001f;
		q = quatFromEuler(M_PI_2_F, 0.0f, 0.0f);
		vector_q = q._transformVector(vector);
		Eigen::Vector3f vector_true = {1.00f, -1.00f, 1.00f};

		for (size_t i = 0; i < 3; i++) {
			if (fabsf(vector_true(i) - vector_q(i)) > tol) {
				warnx("Quaternion method 'rotate' outside tolerance");
				rc = 1;
			}
		}

		q = quatFromEuler(0.3f, 0.2f, 0.1f);
		vector_q = q._transformVector(vector);
		vector_true = {1.1566, 0.7792, 1.0273};

		for (size_t i = 0; i < 3; i++) {
			if (fabsf(vector_true(i) - vector_q(i)) > tol) {
				warnx("Quaternion method 'rotate' outside tolerance");
				rc = 1;
			}
		}

		q = quatFromEuler(-1.5f, -0.2f, 0.5f);
		vector_q = q._transformVector(vector);
		vector_true = {0.5095, 1.4956, -0.7096};

		for (size_t i = 0; i < 3; i++) {
			if (fabsf(vector_true(i) - vector_q(i)) > tol) {
				warnx("Quaternion method 'rotate' outside tolerance");
				rc = 1;
			}
		}

		q = quatFromEuler(M_PI_2_F, -M_PI_2_F, -M_PI_F / 3.0f);
		vector_q = q._transformVector(vector);
		vector_true = { -1.3660, 0.3660, 1.0000};

		for (size_t i = 0; i < 3; i++) {
			if (fabsf(vector_true(i) - vector_q(i)) > tol) {
				warnx("Quaternion method 'rotate' outside tolerance");
				rc = 1;
			}
		}
	}
	return rc;
}