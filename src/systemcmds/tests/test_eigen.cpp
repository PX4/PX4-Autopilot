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
 * @author Nuno Marques <n.marques21@hotmail.com>
 */

#include <cmath>
#include <px4_eigen.h>
#include <mathlib/mathlib.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "tests.h"

using namespace Eigen;

typedef Matrix<float, 10, 1> Vector10f;

static constexpr size_t OPERATOR_ITERATIONS = 30000;

const float min = -M_PI_F;
const float max = M_PI_F;
const float step = M_PI_F / 12;
const float epsilon_f = 1E-4;
uint8_t err_num;

#define TEST_OP(_title, _op)															\
	{																						\
		const hrt_abstime t0 = hrt_absolute_time();						\
		for (size_t j = 0; j < OPERATOR_ITERATIONS; j++) {									\
			_op;																			\
		}																					\
		printf("%llu: %s: finished in: %.6fus\n", (unsigned long long)t0, _title, static_cast<double>(hrt_absolute_time() - t0) / OPERATOR_ITERATIONS); \
	}

#define VERIFY_OP(_title, _op, __OP_TEST__)												\
	{																						\
		_op;																				\
		if (!(__OP_TEST__)) {																 \
			printf(_title ": Failed! (" # __OP_TEST__ ")\n");									   \
			++err_num;																	\
		}																					\
	}

#define TEST_OP_VERIFY(_title, _op, __OP_TEST__)										\
	VERIFY_OP(_title, _op, __OP_TEST__)												\
	TEST_OP(_title, _op)

#define EXPECT_QUATERNION(q_exp, q_act, epsilon)	\
	(fabsf(q_exp.x() - q_act.x()) <= epsilon &&			\
	 fabsf(q_exp.y() - q_act.y()) <= epsilon && 			\
	 fabsf(q_exp.z() - q_act.z()) <= epsilon &&			\
	 fabsf(q_exp.w() - q_act.w()) <= epsilon)

#define EXPECT_NEAR(expected, actual, epsilon)		\
	(fabsf(expected - actual) <= epsilon)

/**
 * @brief
 *	Prints an Eigen::Matrix to stdout
 **/
template<typename T>
void printEigen(const Eigen::MatrixBase<T> &b)
{
	for (int i = 0; i < b.rows(); ++i) {
		printf("(");

		for (int j = 0; j < b.cols(); ++j) {
			if (j > 0) {
				printf(",");
			}

			printf("%.3f", static_cast<double>(b(i, j)));
		}

		printf(")%s\n", i + 1 < b.rows() ? "," : "");
	}
}

// Methods definition
Eigen::Quaternionf quatFromEuler(const Eigen::Vector3f &rpy);
Eigen::Vector3f eulerFromQuat(const Eigen::Quaternionf &q);
Eigen::Matrix3f matrixFromEuler(const Eigen::Vector3f &rpy);
Eigen::Quaternionf eigenqFromPx4q(const math::Quaternion &q);
math::Quaternion px4qFromEigenq(const Eigen::Quaternionf &q);

/**
 * @brief
 *	Construct new Eigen::Quaternion from euler angles
 *	Right order is YPR.
 **/
Eigen::Quaternionf quatFromEuler(const Eigen::Vector3f &rpy)
{
	return Eigen::Quaternionf(
		       Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
		       Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
		       Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX())
	       );
}

/**
 * @brief
 *	Construct new Eigen::Vector3f of euler angles from quaternion
 *	Right order is YPR.
 **/
Eigen::Vector3f eulerFromQuat(const Eigen::Quaternionf &q)
{
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

/**
 * @brief
 *	Construct new Eigen::Matrix3f from euler angles
 **/
Eigen::Matrix3f matrixFromEuler(const Eigen::Vector3f &rpy)
{
	return quatFromEuler(rpy).toRotationMatrix();
}

/**
 * @brief
 *	Adjust PX4 math::quaternion to Eigen::Quaternionf
 **/
Eigen::Quaternionf eigenqFromPx4q(const math::Quaternion &q)
{
	return Eigen::Quaternionf(q.data[1], q.data[2], q.data[3], q.data[0]);
}

/**
 * @brief
 *	Adjust Eigen::Quaternionf to PX4 math::quaternion
 **/
math::Quaternion px4qFromEigenq(const Eigen::Quaternionf &q)
{
	return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

/**
 * @brief
 *	Testing main routine
 **/
int test_eigen(int argc, char *argv[])
{
	int rc = 0;
	warnx("Testing Eigen math...");

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
		VERIFY_OP("Vector2f + Vector2f", v = v + v1, v.isApprox(v1 + v1));
		VERIFY_OP("Vector2f - Vector2f", v = v - v1, v.isApprox(v1));
		VERIFY_OP("Vector2f += Vector2f", v += v1, v.isApprox(v1 + v1));
		VERIFY_OP("Vector2f -= Vector2f", v -= v1, v.isApprox(v1));
		TEST_OP_VERIFY("Vector2f dot Vector2f", v.dot(v1), fabs(v.dot(v1) - 5.0f) <= FLT_EPSILON);
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
		TEST_OP("Vector<4> += Vector<4>", v += v1);
		TEST_OP("Vector<4> -= Vector<4>", v -= v1);
		TEST_OP("Vector<4> dot Vector<4>", v.dot(v1));
	}

	{
		Vector10f v1;
		v1.Zero();
		float data[10];
		TEST_OP("Constructor Vector<10>()", Vector10f v3);
		TEST_OP("Constructor Vector<10>(Vector<10>)", Vector10f v3(v1));
		TEST_OP("Constructor Vector<10>(float[])", Vector10f v3(data));
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
		Vector10f v1;
		v1.Zero();
		TEST_OP("Matrix<10, 10> * Vector<10>", m1 * v1);
		TEST_OP("Matrix<10, 10> + Matrix<10, 10>", m1 + m2);
		TEST_OP("Matrix<10, 10> * Matrix<10, 10>", m1 * m2);
	}

	{
		printf("%llu: Nonsymmetric matrix operations test...\n", (unsigned long long)hrt_absolute_time());
		// test nonsymmetric +, -, +=, -=

		const Eigen::Matrix<float, 2, 3> m1_orig =
			(Eigen::Matrix<float, 2, 3>() << 1, 2, 3,
			 4, 5, 6).finished();

		Eigen::Matrix<float, 2, 3> m1(m1_orig);

		Eigen::Matrix<float, 2, 3> m2;
		m2 << 2, 4, 6,
		8, 10, 12;

		Eigen::Matrix<float, 2, 3> m3;
		m3 << 3, 6, 9,
		12, 15, 18;

		if (m1 + m2 != m3) {
			printf("%llu: Matrix<2, 3> + Matrix<2, 3> failed!\n", (unsigned long long)hrt_absolute_time());
			printEigen(m1 + m2);
			printf("!=\n");
			printEigen(m3);
			++err_num;
			rc = 1;
		}

		if (m3 - m2 != m1) {
			printf("%llu: Matrix<2, 3> - Matrix<2, 3> failed!\n", (unsigned long long)hrt_absolute_time());
			printEigen(m3 - m2);
			printf("!=\n");
			printEigen(m1);
			++err_num;
			rc = 1;
		}

		m1 += m2;

		if (m1 != m3) {
			printf("%llu: Matrix<2, 3> += Matrix<2, 3> failed!\n", (unsigned long long)hrt_absolute_time());
			printEigen(m1);
			printf("!=\n");
			printEigen(m3);
			++err_num;
			rc = 1;
		}

		m1 -= m2;

		if (m1 != m1_orig) {
			printf("%llu: Matrix<2, 3> -= Matrix<2, 3> failed!\n", (unsigned long long)hrt_absolute_time());
			printEigen(m1);
			printf("!=\n");
			printEigen(m1_orig);
			++err_num;
			rc = 1;
		}
	}

	/* QUATERNION TESTS */
	{
		// Test conversion rotation matrix to quaternion and back
		// against existing PX4 mathlib
		math::Matrix<3, 3> R_orig;
		Eigen::Quaternionf q_true;
		Eigen::Quaternionf q;
		Eigen::Matrix3f R;

		printf("%llu: Conversion method: Quaternion transformation methods test...\n", (unsigned long long)hrt_absolute_time());
		printf("%llu: Conversion method: Testing known values...\n", (unsigned long long)hrt_absolute_time());

		/******************************************** TEST 1 ****************************************************/
		q_true = {0.0f, 0.0f, 0.0f, 1.0f};
		math::Quaternion q_px4 = {1.0f, 0.0f, 0.0f, 0.0f};
		Eigen::Quaternionf q_eigen(eigenqFromPx4q(q_px4));

		if (!EXPECT_QUATERNION(q_true, q_eigen, FLT_EPSILON)) {
			printf("%llu: Value of: Quaternion1 [1.0, 0.0, 0.0, 0.0]\n", (unsigned long long)hrt_absolute_time());
			printf("Actual:  \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
			printf("Expected: \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q_true.x(), q_true.y(), q_true.z(), q_true.w());
			++err_num;
			rc = 1;
		}

		/********************************************************************************************************/
		/******************************************** TEST 2 ****************************************************/
		q_true = {1.0f, 0.0f, 0.0f, 0.0f};
		Eigen::Quaternionf q2_eigen = {0.0f, 0.0f, 0.0f, 1.0f};
		math::Quaternion q2_px4(px4qFromEigenq(q2_eigen));
		Eigen::Quaternionf q2_eigen_(q2_px4.data[3], q2_px4.data[0], q2_px4.data[1], q2_px4.data[2]);

		if (!EXPECT_QUATERNION(q_true, q2_eigen_, FLT_EPSILON)) {
			printf("%llu: Value of: Quaternion2 [0.0, 0.0, 0.0, 1.0]\n", (unsigned long long)hrt_absolute_time());
			printf("Actual:  \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q2_px4.data[0], q2_px4.data[1], q2_px4.data[2], q2_px4.data[3]);
			printf("Expected: \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q_true.x(), q_true.y(), q_true.z(), q_true.w());
			++err_num;
			rc = 1;
		}

		/********************************************************************************************************/
		/******************************************** TEST 3 ****************************************************/
		q_true = quatFromEuler(Eigen::Vector3f(0.3f, 0.2f, 0.1f));
		q = {0.9833474432563558f, 0.14357217502739184f, 0.10602051106179561f, 0.0342707985504821f};

		if (!EXPECT_QUATERNION(q_true, q, FLT_EPSILON)) {
			printf("%llu: Value of: Quaternion [0.9833, 0.1436, 0.1060, 0.0343]\n", (unsigned long long)hrt_absolute_time());
			printf("Actual:  \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q.w(), q.x(), q.y(), q.z());
			printf("Expected: \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q_true.w(), q_true.x(), q_true.y(), q_true.z());
			++err_num;
			rc = 1;
		}

		/********************************************************************************************************/
		/******************************************** TEST 4 ****************************************************/
		q_true = quatFromEuler(Eigen::Vector3f(-1.5f, -0.2f, 0.5f));
		q = {0.7222365948153096f, -0.6390766544101811f, -0.2385737751841646f, 0.11418355701173476f};

		if (!EXPECT_QUATERNION(q_true, q, FLT_EPSILON)) {
			printf("%llu: Value of: Quaternion [0.7222, -0.6391, -0.2386, 0.1142]\n", (unsigned long long)hrt_absolute_time());
			printf("Actual:  \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q.w(), q.x(), q.y(), q.z());
			printf("Expected: \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q_true.w(), q_true.x(), q_true.y(), q_true.z());
			++err_num;
			rc = 1;
		}

		/********************************************************************************************************/
		/******************************************** TEST 5 ****************************************************/
		q_true = quatFromEuler(Eigen::Vector3f(M_PI_2_F, -M_PI_2_F, -M_PI_F / 3));
		q = {0.6830127018922193f, 0.18301270189221933f, -0.6830127018922193f, 0.18301270189221933f};

		for (size_t i = 0; i < 4; i++) {
			if (!EXPECT_QUATERNION(q_true, q, FLT_EPSILON)) {
				printf("%llu: It[%d]: Value of: Quaternion [0.6830, 0.1830, -0.6830, 0.1830]\n",
				       (unsigned long long)hrt_absolute_time(), (int)i);
				printf("Actual:  \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q.w(), q.x(), q.y(), q.z());
				printf("Expected: \t[%8.5f, %8.5f, %8.5f, %8.5f]\n", q_true.w(), q_true.x(), q_true.y(), q_true.z());
				++err_num;
				rc = 1;
			}
		}

		/********************************************************************************************************/
		/******************************************** TEST 6 ****************************************************/
		printf("%llu: Conversion method: Testing transformation range...\n", (unsigned long long)hrt_absolute_time());

		for (float roll = min; roll <= max; roll += step) {
			for (float pitch = min; pitch <= max; pitch += step) {
				for (float yaw = min; yaw <= max; yaw += step) {

					q = Eigen::Quaternionf(quatFromEuler(Eigen::Vector3f(roll, pitch, yaw)));

					R = q.toRotationMatrix();
					R_orig.from_euler(roll, pitch, yaw);

					for (size_t i = 0; i < 3; i++) {
						for (size_t j = 0; j < 3; j++) {
							if (!EXPECT_NEAR(R_orig(i, j), R(i, j), epsilon_f)) {
								printf("%llu: (%d, %d) Value of: Quaternion constructor or 'toRotationMatrix'\n",
								       (unsigned long long)hrt_absolute_time(), (int)i, (int)j);
								printf("Actual:  \t%8.5f\n", R(i, j));
								printf("Expected: \t%8.5f\n", R_orig(i, j));
								++err_num;
								rc = 1;
							}
						}
					}
				}
			}
		}
	}

	{
		// Test rotation method (rotate vector by quaternion)
		Eigen::Vector3f vector = {1.0f, 1.0f, 1.0f};
		Eigen::Vector3f vector_q;
		Eigen::Vector3f vector_r;
		Eigen::Quaternionf q;
		Eigen::Matrix3f R;

		printf("%llu: Rotation method: Quaternion vector rotation method test...\n", (unsigned long long)hrt_absolute_time());
		printf("%llu: Rotation method: Testing known values...\n", (unsigned long long)hrt_absolute_time());

		/******************************************** TEST 1 ****************************************************/
		q = quatFromEuler(Eigen::Vector3f(0.0f, 0.0f, M_PI_2_F));
		vector_q = q._transformVector(vector);
		Eigen::Vector3f vector_true = { -1.00f, 1.00f, 1.00f};

		for (size_t i = 0; i < 3; i++) {
			if (!EXPECT_NEAR(vector_true(i), vector_q(i), FLT_EPSILON)) {
				printf("%llu: It[%d]: Value of: Quaternion method 'rotate'\n", (unsigned long long)hrt_absolute_time(), (int)i);
				printf("Actual:  \t[%8.5f, %8.5f, %8.5f]\n", vector_q(0), vector_q(1), vector_q(2));
				printf("Expected: \t[%8.5f, %8.5f, %8.5f]\n", vector_true(0), vector_true(1), vector_true(2));
				++err_num;
				rc = 1;
			}
		}

		/********************************************************************************************************/
		/******************************************** TEST 2 ****************************************************/
		q = quatFromEuler(Eigen::Vector3f(0.1f, 0.2f, 0.3f));
		vector_q = q._transformVector(vector);
		vector_true = {0.8795481794122900f, 1.2090975499501229f, 0.874344391414010f};

		for (size_t i = 0; i < 3; i++) {
			if (!EXPECT_NEAR(vector_true(i), vector_q(i), epsilon_f)) {
				printf("%llu: It[%d]: Value of: Quaternion method 'rotate'\n", (unsigned long long)hrt_absolute_time(), (int)i);
				printf("Actual:  \t[%8.5f, %8.5f, %8.5f]\n", (double)vector_q(0), (double)vector_q(1), (double)vector_q(2));
				printf("Expected: \t[%8.5f, %8.5f, %8.5f]\n", (double)vector_true(0), (double)vector_true(1), (double)vector_true(2));
				++err_num;
				rc = 1;
			}
		}

		/********************************************************************************************************/
		/******************************************** TEST 3 ****************************************************/
		q = quatFromEuler(Eigen::Vector3f(0.5f, -0.2f, -1.5f));
		vector_q = q._transformVector(vector);
		vector_true = {0.447416342848463f, -0.6805264343934600f, 1.528627615949624f};

		for (size_t i = 0; i < 3; i++) {
			if (!EXPECT_NEAR(vector_true(i), vector_q(i), epsilon_f)) {
				printf("%llu: It[%d]: Value of: Quaternion method 'rotate'\n", (unsigned long long)hrt_absolute_time(), (int)i);
				printf("Actual:  \t[%8.5f, %8.5f, %8.5f]\n", (double)vector_q(0), (double)vector_q(1), (double)vector_q(2));
				printf("Expected: \t[%8.5f, %8.5f, %8.5f]\n", (double)vector_true(0), (double)vector_true(1), (double)vector_true(2));
				++err_num;
				rc = 1;
			}
		}

		/********************************************************************************************************/
		/******************************************** TEST 4 ****************************************************/
		q = quatFromEuler(Eigen::Vector3f(-M_PI_F / 3.0f, -M_PI_2_F, M_PI_2_F));
		vector_q = q._transformVector(vector);
		vector_true = { -1.366030f, 0.366025f, 1.000000f};

		for (size_t i = 0; i < 3; i++) {
			if (!EXPECT_NEAR(vector_true(i), vector_q(i), epsilon_f)) {
				printf("%llu: It[%d]: Value of: Quaternion method 'rotate'\n", (unsigned long long)hrt_absolute_time(), (int)i);
				printf("Actual:  \t[%8.5f, %8.5f, %8.5f]\n", (double)vector_q(0), (double)vector_q(1), (double)vector_q(2));
				printf("Expected: \t[%8.5f, %8.5f, %8.5f]\n", (double)vector_true(0), (double)vector_true(1), (double)vector_true(2));
				++err_num;
				rc = 1;
			}
		}

		/********************************************************************************************************/
		/******************************************** TEST 5 ****************************************************/
		printf("%llu: Rotation method: Testing transformation range...\n", (unsigned long long)hrt_absolute_time());

		Eigen::Vector3f vectorR(1.0f, 1.0f, 1.0f);

		for (float roll = min; roll <= max; roll += step) {
			for (float pitch = min; pitch <= max; pitch += step) {
				for (float yaw = min; yaw <= max; yaw += step) {

					R = matrixFromEuler(Eigen::Vector3f(roll, pitch, yaw));
					q = quatFromEuler(Eigen::Vector3f(roll, pitch, yaw));

					vector_r = R * vectorR;
					vector_q = q._transformVector(vectorR);

					for (int i = 0; i < 3; i++) {
						if (!EXPECT_NEAR(vector_r(i), vector_q(i), epsilon_f)) {
							printf("%llu: (%d) Value of: Quaternion method 'rotate'\n", (unsigned long long)hrt_absolute_time(), i);
							printf("Actual:  \t%8.5f\n", vector_q(i));
							printf("Expected: \t%8.5f\n", vector_r(i));
							++err_num;
							rc = 1;
						}
					}
				}
			}
		}
	}
	printf("%llu: Finished Eigen math tests with %d error(s)...\n", (unsigned long long)hrt_absolute_time(), err_num);
	return rc;
}
