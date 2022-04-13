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

TEST(MatrixMultiplicationTest, Multiplication)
{
	float data[9] = {1, 0, 0, 0, 1, 0, 1, 0, 1};
	Matrix3f A(data);

	float data_check[9] = {1, 0, 0, 0, 1, 0, -1, 0, 1};
	Matrix3f A_I(data_check);
	Matrix3f I;
	I.setIdentity();
	Matrix3f R = A * A_I;
	EXPECT_EQ(R, I);

	Matrix3f R2 = A;
	R2 *= A_I;
	EXPECT_EQ(R2, I);

	EXPECT_EQ(R2, I);
	EXPECT_NE(A, A_I);
	Matrix3f A2 = eye<float, 3>() * 2;
	Matrix3f B = A2.emult(A2);
	Matrix3f B_check = eye<float, 3>() * 4;
	Matrix3f C_check = eye<float, 3>() * 2;
	EXPECT_EQ(B, B_check);
	Matrix3f C = B_check.edivide(C_check);

	float off_diagonal_nan[9] = {2, NAN, NAN, NAN, 2, NAN, NAN, NAN, 2};
	// off diagonal are NANs because division by 0
	EXPECT_EQ(C, Matrix3f(off_diagonal_nan));

	// Test non-square matrix
	float data_43[12] = {1, 3, 2,
			     2, 2, 1,
			     5, 2, 1,
			     2, 3, 4
			    };
	float data_32[6] = {2, 3,
			    1, 7,
			    5, 4
			   };

	Matrix<float, 4, 3> m43(data_43);
	Matrix<float, 3, 2> m32(data_32);

	Matrix<float, 4, 2> m42 = m43 * m32;

	float data_42[8] = {15, 32,
			    11, 24,
			    17, 33,
			    27, 43
			   };
	Matrix<float, 4, 2> m42_check(data_42);
	EXPECT_EQ(m42, m42_check);

	float data_42_plus2[8] = {17, 34,
				  13, 26,
				  19, 35,
				  29, 45
				 };
	Matrix<float, 4, 2> m42_plus2_check(data_42_plus2);
	Matrix<float, 4, 2> m42_plus2 = m42 - (-2);
	EXPECT_EQ(m42_plus2, m42_plus2_check);
}
