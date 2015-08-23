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
 * @brief Eigen math unittest
 * @author Nuno Marques <n.marques21@hotmail.com>
 */

#include <gtest/gtest.h>
#include <px4_eigen.h>
#include <mathlib/mathlib.h>
#include <iostream>

using namespace Eigen;

const float min = -M_PI_F;
const float max = M_PI_F;
const float step = M_PI_F / 12;
const float epsilon_f = 1E-5;

#define EXPECT_QUATERNION_EIGEN(q1, q2, epsilon)															\
	if (std::copysign(1.0, q1.w()) == std::copysign(1.0, q2.w()))											 \
		EXPECT_NEAR(q1.w(), q2.w(), epsilon);																\
	else																									\
		EXPECT_NEAR(-q1.w(), q2.w(), epsilon);																\
	if (std::copysign(1.0, q1.x()) == std::copysign(1.0, q2.x()))											 \
		EXPECT_NEAR(q1.x(), q2.x(), epsilon);																\
	else																									\
		EXPECT_NEAR(-q1.x(), q2.x(), epsilon);																\
	if (std::copysign(1.0, q1.y()) == std::copysign(1.0, q2.y()))											 \
		EXPECT_NEAR(q1.y(), q2.y(), epsilon);																\
	else																									\
		EXPECT_NEAR(-q1.y(), q2.y(), epsilon);																\
	if (std::copysign(1.0, q1.z()) == std::copysign(1.0, q2.z()))											 \
		EXPECT_NEAR(q1.z(), q2.z(), epsilon);																\
	else																									\
		EXPECT_NEAR(-q1.z(), q2.z(), epsilon)

#define EXPECT_QUATERNION_PX4(q1, q2, epsilon)																\
	if (std::copysign(1.0, q1.data[0]) == std::copysign(1.0, q2.data[0]))									 \
		EXPECT_NEAR(q1.data[0], q2.data[0], epsilon);														\
	else																									\
		EXPECT_NEAR(-q1.data[0], q2.data[0], epsilon);														\
	if (std::copysign(1.0, q1.data[1]) == std::copysign(1.0, q2.data[1]))									 \
		EXPECT_NEAR(q1.data[1], q2.data[1], epsilon);														\
	else																									\
		EXPECT_NEAR(-q1.data[1], q2.data[1], epsilon);														\
	if (std::copysign(1.0, q1.data[2]) == std::copysign(1.0, q2.data[2]))									 \
		EXPECT_NEAR(q1.data[2], q2.data[2], epsilon);														\
	else																									\
		EXPECT_NEAR(-q1.data[2], q2.data[2], epsilon);														\
	if (std::copysign(1.0, q1.data[3]) == std::copysign(1.0, q2.data[3]))									 \
		EXPECT_NEAR(q1.data[3], q2.data[3], epsilon);														\
	else																									\
		EXPECT_NEAR(-q1.data[3], q2.data[3], epsilon)

namespace Eigen {
/*
 * Custom print functions for vectors and matrices (PrintTo overriding)
 */
void PrintTo(const Vector3f& vec, ::std::ostream* os) {
	*os <<  "|  " << vec(0) << "\t|\n" <<
	"\t  |  " << vec(1) << "\t|\n" <<
	"\t  |  " << vec(2) << "\t|\n";
}

void PrintTo(const Vector4f& vec, ::std::ostream* os) {
	*os <<  "|  " << vec(0) << "\t|\n" <<
	"\t  |  " << vec(1) << "\t|\n" <<
	"\t  |  " << vec(2) << "\t|\n" <<
	"\t  |  " << vec(3) << "\t|\n";
}
}

static void printPx4Quaternion(const math::Quaternion &q)
{
	std::cout << "w: "   << q.data[0] << "\n" <<
	"\t x: " << q.data[1] << "\n" <<
	"\t y: " << q.data[2] << "\n" <<
	"\t z: " << q.data[3] << "\n";
}

static void printEigenQuaternion(const Quaternionf &q)
{
	std::cout << "w: "   << q.w() << "\n" <<
	"\t x: " << q.x() << "\n" <<
	"\t y: " << q.y() << "\n" <<
	"\t z: " << q.z() << "\n";
}

static void printEigenRotationMatrix(const Matrix3f &rm)
{
	for (size_t i = 0; i < 2; ++i)
	{
		std::printf("|\t%8.7f\t%8.7f\t%8.7f\t|\n\t ", rm(i,0), rm(i,1), rm(i,2));
	}
	std::printf("|\t%8.7f\t%8.7f\t%8.7f\t|\n", rm(2,0), rm(2,1), rm(2,2));
}

static void printPx4RotationMatrix(const math::Matrix<3,3> &rm)
{
	for (size_t i = 0; i < 2; ++i)
	{
		std::printf("|\t%8.7f\t%8.7f\t%8.7f\t|\n\t ", rm(i,0), rm(i,1), rm(i,2));
	}
	std::printf("|\t%8.7f\t%8.7f\t%8.7f\t|\n", rm(2,0), rm(2,1), rm(2,2));
}

TEST(EigenTest, Constructor_Vector2f__1)
{
	Vector2f v {0.0f, 0.0f};
	EXPECT_EQ(Vector2f::Zero(), v);
}

TEST(EigenTest, Constructor_Vector2f__2)
{
	Vector2f v1;
	Vector2f v2(v1);
	EXPECT_EQ(v1, v2);
}

TEST(EigenTest, Constructor_Vector2f__3)
{
	Vector2f data;
	Vector2f v(data);
	EXPECT_TRUE(v(0) == data(0) &&
		v(1) == data(1));
}

TEST(EigenTest, Constructor_Vector2f__4)
{
	Vector2f v(1.0f, 2.0f);
	EXPECT_TRUE(v(0) == 1.0f &&
		v(1) == 2.0f);
}

TEST(EigenTest, Vector2f_equal_Vector2f)
{
	Vector2f v1;
	Vector2f v2 = v1;
	EXPECT_EQ(v1, v2);
}

TEST(EigenTest, Vector2f_plus_Vector2f)
{
	Vector2f v1;
	Vector2f v2 = v1;
	EXPECT_EQ(v1 + v1, v2 = v2 + v1);
}

TEST(EigenTest, Vector2f_minus_Vector2f)
{
	Vector2f v1;
	Vector2f v2  = 2 * v1;
	EXPECT_EQ(v1, v2 = v2 - v1);
}

TEST(EigenTest, Vector2f_plusequal_Vector2f)
{
	Vector2f v1;
	Vector2f v2 = v1;
	EXPECT_EQ(v1 + v1, v2 += v1);
}

TEST(EigenTest, Vector2f_minusequal_Vector2f)
{
	Vector2f v1;
	Vector2f v2 = 2 * v1;
	EXPECT_EQ(v1, v2 -= v1);
}

TEST(EigenTest, Vector2f_dot_Vector2f)
{
	Vector2f v1 = Vector2f::Ones();
	Vector2f v2 = Vector2f::Ones();
	EXPECT_EQ(2.0f, v2.dot(v1));
}

/*------------------------------------------------------*/

TEST(EigenTest, Constructor_Vector3f__1)
{
	Vector3f v {0.0f, 0.0f, 0.0f};
	EXPECT_EQ(Vector3f::Zero(), v);
}

TEST(EigenTest, Constructor_Vector3f__2)
{
	Vector3f v1;
	Vector3f v2(v1);
	EXPECT_EQ(v1, v2);
}

TEST(EigenTest, Constructor_Vector3f__3)
{
	Vector3f data;
	Vector3f v(data);
	EXPECT_TRUE(v(0) == data(0) &&
		v(1) == data(1) &&
		v(2) == data(2));
}

TEST(EigenTest, Constructor_Vector3f__4)
{
	Vector3f v(1.0f, 2.0f, 3.0f);
	EXPECT_TRUE(v(0) == 1.0f &&
		v(1) == 2.0f &&
		v(2) == 3.0f);
}

TEST(EigenTest, Vector3f_equal_Vector3f)
{
	Vector3f v1;
	Vector3f v2 = v1;
	EXPECT_EQ(v1, v2);
}

TEST(EigenTest, Vector3f_plus_Vector3f)
{
	Vector3f v1;
	Vector3f v2 = v1;
	EXPECT_EQ(v1 + v1, v2 = v2 + v1);
}

TEST(EigenTest, Vector3f_minus_Vector3f)
{
	Vector3f v1;
	Vector3f v2  = 2 * v1;
	EXPECT_EQ(v1, v2 = v2 - v1);
}

TEST(EigenTest, Vector3f_plusequal_Vector3f)
{
	Vector3f v1;
	Vector3f v2 = v1;
	EXPECT_EQ(v1 + v1, v2 += v1);
}

TEST(EigenTest, Vector3f_minusequal_Vector3f)
{
	Vector3f v1;
	Vector3f v2 = 2 * v1;
	EXPECT_EQ(v1, v2 -= v1);
}

TEST(EigenTest, Vector3f_times_float)
{
	Vector3f v1 {1.0f, 1.0f, 1.0f};
	EXPECT_EQ(Vector3f(2.0f, 2.0f, 2.0f), v1 * 2.0f);
}

TEST(EigenTest, Vector3f_divided_by_float)
{
	Vector3f v1 {2.0f, 2.0f, 2.0f};
	EXPECT_EQ(Vector3f(1.0f, 1.0f, 1.0f), v1 / 2.0f);
}

TEST(EigenTest, Vector3f_timesequal_float)
{
	Vector3f v1 {1.0f, 1.0f, 1.0f};
	EXPECT_EQ(Vector3f(2.0f, 2.0f, 2.0f), v1 *= 2.0f);
}

TEST(EigenTest, Vector3f_divided_byequal_float)
{
	Vector3f v1 {2.0f, 2.0f, 2.0f};
	EXPECT_EQ(Vector3f(1.0f, 1.0f, 1.0f), v1 /= 2.0f);
}

TEST(EigenTest, Vector3f_dot_Vector3f)
{
	Vector3f v1 = Vector3f::Ones();
	Vector3f v2 = Vector3f::Ones();
	EXPECT_EQ(3.0f, v2.dot(v1));
}

TEST(EigenTest, Vector3f_cross_Vector3f)
{
	Vector3f v1 {1.0f, 2.0f, 3.0f};
	Vector3f v2 {3.0f, 2.0f, 1.0f};
	EXPECT_EQ(Vector3f(4.0f, -8.0f, 4.0f), v2.cross(v1));
}

TEST(EigenTest, Vector3f_length)
{
	Vector3f v {1.0f, 2.0f, 3.0f};
	EXPECT_NEAR(sqrt(14.0f), v.norm(), epsilon_f);
}

TEST(EigenTest, Vector3f_squared_length)
{
	Vector3f v {1.0f, 2.0f, 3.0f};
	EXPECT_EQ(14.0f, v.squaredNorm());
}

TEST(EigenTest, Vector3f_element_read)
{
	Vector3f v {1.0f, 1.0f, 1.0f};
	volatile float a;
	EXPECT_EQ(v(0), a = v(0));
}

TEST(EigenTest, Vector3f_element_write)
{
	Vector3f v;
	EXPECT_EQ(1.0f, v(0) = 1.0f);
}

/*------------------------------------------------------*/

TEST(EigenTest, Constructor_Vector4f__1)
{
	Vector4f v {0.0f, 0.0f, 0.0f, 0.0f};
	EXPECT_EQ(Vector4f::Zero(), v);
}

TEST(EigenTest, Constructor_Vector4f__2)
{
	Vector4f v1;
	Vector4f v2(v1);
	EXPECT_EQ(v1, v2);
}

TEST(EigenTest, Constructor_Vector4f__3)
{
	Vector4f data;
	Vector4f v(data);
	EXPECT_TRUE(v(0) == data(0) &&
		v(1) == data(1) &&
		v(2) == data(2) &&
		v(3) == data(3));
}

TEST(EigenTest, Constructor_Vector4f__4)
{
	Vector4f v(1.0f, 2.0f, 3.0f, 4.0f);
	EXPECT_TRUE(v(0) == 1.0f &&
		v(1) == 2.0f &&
		v(2) == 3.0f &&
		v(3) == 4.0f);
}

TEST(EigenTest, Vector4f_equal_Vector4f)
{
	Vector4f v1;
	Vector4f v2 = v1;
	EXPECT_EQ(v1, v2);
}

TEST(EigenTest, Vector4f_plus_Vector4f)
{
	Vector4f v1;
	Vector4f v2 = v1;
	EXPECT_EQ(v1 + v1, v2 = v2 + v1);
}

TEST(EigenTest, Vector4f_minus_Vector4f)
{
	Vector4f v1;
	Vector4f v2  = 2 * v1;
	EXPECT_EQ(v1, v2 = v2 - v1);
}

TEST(EigenTest, Vector4f_plusequal_Vector4f)
{
	Vector4f v1;
	Vector4f v2 = v1;
	EXPECT_EQ(v1 + v1, v2 += v1);
}

TEST(EigenTest, Vector4f_minusequal_Vector4f)
{
	Vector4f v1;
	Vector4f v2 = 2 * v1;
	EXPECT_EQ(v1, v2 -= v1);
}

TEST(EigenTest, Vector4f_times_float)
{
	Vector4f v1 {1.0f, 1.0f, 1.0f, 1.0f};
	EXPECT_EQ(Vector4f(2.0f, 2.0f, 2.0f, 2.0f), v1 * 2.0f);
}

TEST(EigenTest, Vector4f_divided_by_float)
{
	Vector4f v1 {2.0f, 2.0f, 2.0f, 2.0f};
	EXPECT_EQ(Vector4f(1.0f, 1.0f, 1.0f, 1.0f), v1 / 2.0f);
}

TEST(EigenTest, Vector4f_timesequal_float)
{
	Vector4f v1 {1.0f, 1.0f, 1.0f, 1.0f};
	EXPECT_EQ(Vector4f(2.0f, 2.0f, 2.0f, 2.0f), v1 *= 2.0f);
}

TEST(EigenTest, Vector4f_divided_byequal_float)
{
	Vector4f v1 {2.0f, 2.0f, 2.0f, 2.0f};
	EXPECT_EQ(Vector4f(1.0f, 1.0f, 1.0f, 1.0f), v1 /= 2.0f);
}

TEST(EigenTest, Vector4f_dot_Vector4f)
{
	Vector4f v1 = Vector4f::Ones();
	Vector4f v2 = Vector4f::Ones();
	EXPECT_EQ(4.0f, v2.dot(v1));
}

TEST(EigenTest, Vector4f_length)
{
	Vector4f v {1.0f, 2.0f, 3.0f, 4.0f};
	EXPECT_NEAR(sqrt(30.0f), v.norm(), epsilon_f);
}

TEST(EigenTest, Vector4f_squared_length)
{
	Vector4f v {1.0f, 2.0f, 3.0f, 4.0f};
	EXPECT_EQ(30.0f, v.squaredNorm());
}

TEST(EigenTest, Vector4f_element_read)
{
	Vector4f v {1.0f, 1.0f, 1.0f, 1.0f};
	volatile float a;
	EXPECT_EQ(v(0), a = v(0));
}

TEST(EigenTest, Vector4f_element_write)
{
	Vector4f v;
	EXPECT_EQ(1.0f, v(0) = 1.0f);
}

/*------------------------------------------------------*/

TEST(EigenTest, Quaternion_conversion_PX4_to_Eigen)
{
	Quaternionf expected = {1.0f, 0.0f, 0.0f, 0.0f};	// Constructor: (w, x, y, z) [!]But stores internally as x, y, z, w
	math::Quaternion q_px4 = {1.0f, 0.0f, 0.0f, 0.0f};	// Constructor: (w, x, y, z)
	Quaternionf q_eigen(eigenqFromPx4q(q_px4));

	std::cout << "Expected:";
	printEigenQuaternion(expected);
	std::cout << "Actual:  ";
	printEigenQuaternion(q_eigen);

	EXPECT_QUATERNION_EIGEN(expected, q_eigen, FLT_EPSILON);
}

TEST(EigenTest, Quaternion_conversion_Eigen_to_PX4)
{
	math::Quaternion expected = {1.0f, 0.0f, 0.0f, 0.0f};	// Constructor: (w, x, y, z)
	Quaternionf q_eigen = {1.0f, 0.0f, 0.0f, 0.0f};		// Constructor: (w, x, y, z) [!]But stores internally as x, y, z, w
	math::Quaternion q_px4(px4qFromEigenq(q_eigen));

	std::cout << "Expected:";
	printPx4Quaternion(expected);
	std::cout << "Actual:  ";
	printPx4Quaternion(q_px4);

	EXPECT_QUATERNION_PX4(expected, q_px4, FLT_EPSILON);
}

TEST(EigenTest, Quaternion_from_Euler)
{
	Quaternionf expected {0.9833474432563558f, 0.14357217502739184f, 0.10602051106179561f, 0.0342707985504821f};
	Quaternionf q(quatFromEuler(Vector3f(0.3f, 0.2f, 0.1f)));

	std::cout << "Expected:";
	printEigenQuaternion(expected);
	std::cout << "Actual:  ";
	printEigenQuaternion(q);

	EXPECT_QUATERNION_EIGEN(expected, q, FLT_EPSILON);
}

TEST(EigenTest, Quaternion_from_Euler_Eigen_vs_PX4)
{
	math::Quaternion q_px4;
	Quaternionf q_eigen;

	float roll, pitch, yaw;

	for (roll = min; roll <= max; roll += step)
		for (pitch = min; pitch <= max; pitch += step)
			for (yaw = min; yaw <= max; yaw += step) {
				q_px4.from_euler(roll, pitch, yaw);
				q_eigen = quatFromEuler(Vector3f(Vector3f(roll, pitch, yaw)));

				/*std::cout << "Expected:";
				   printPx4Quaternion(q_px4);
				   std::cout << "\nActual:  ";
				   printEigenQuaternion(q_eigen);
				   std::cout << "\n";*/

				EXPECT_QUATERNION_EIGEN(eigenqFromPx4q(q_px4), q_eigen, epsilon_f);
			}
}

TEST(EigenTest, RotationMatrix_from_Euler_Eigen_vs_PX4)
{
	math::Matrix<3,3> R_px4;
	Matrix3f R_eigen;

	float roll, pitch, yaw;

	for (roll = min; roll <= max; roll += step)
		for (pitch = min; pitch <= max; pitch += step)
			for (yaw = min; yaw <= max; yaw += step) {
				R_px4.from_euler(roll, pitch, yaw);
				R_eigen = matrixFromEuler(Vector3f(roll, pitch, yaw));

				/*std::cout << "Expected:";
				   printPx4RotationMatrix(R_px4);
				   std::cout << "\nActual:  ";
				   printEigenRotationMatrix(R_eigen);
				   std::cout << "\n";*/

				for (size_t i = 0; i < 3; i++)
					for (size_t j = 0; j < 3; j++)
						EXPECT_NEAR(R_px4(i, j), R_eigen(i, j), epsilon_f);
			}
}

TEST(EigenTest, Quaternion_from_RotationMatrix_Eigen_vs_PX4)
{
	math::Quaternion q_px4;
	math::Matrix<3,3> R_px4;
	Quaternionf q_eigen;

	float roll, pitch, yaw;

	for (roll = min; roll <= max; roll += step)
		for (pitch = min; pitch <= max; pitch += step)
			for (yaw = min; yaw <= max; yaw += step) {
				R_px4.from_euler(roll, pitch, yaw);
				q_px4.from_dcm(R_px4);
				q_eigen = Matrix3f(matrixFromEuler(Vector3f(roll, pitch, yaw)));

				/*std::cout << "Expected:";
				   printPx4Quaternion(q_px4);
				   std::cout << "\nActual:  ";
				   printEigenQuaternion(q_eigen);
				   std::cout << "\n";*/

				EXPECT_QUATERNION_EIGEN(eigenqFromPx4q(q_px4), q_eigen, epsilon_f);
			}
}
