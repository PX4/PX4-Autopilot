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

static const size_t n_large = 50;

TEST(MatrixInverseTest, Inverse)
{
	float data[9] = {0, 2, 3,
			 4, 5, 6,
			 7, 8, 10
			};
	float data_check[9] = {
		-0.4f, -0.8f,  0.6f,
			-0.4f,  4.2f, -2.4f,
			0.6f, -2.8f,  1.6f
		};

	SquareMatrix<float, 3> A(data);
	SquareMatrix<float, 3> A_I = inv(A);
	SquareMatrix<float, 3> A_I_check(data_check);
	EXPECT_EQ(A_I, A_I_check);

	float data_2x2[4] = {12, 2,
			     -7, 5
			    };
	float data_2x2_check[4] = {
		0.0675675675f, -0.02702702f,
		0.0945945945f, 0.162162162f
	};

	SquareMatrix<float, 2> A2x2(data_2x2);
	SquareMatrix<float, 2> A2x2_I = inv(A2x2);
	SquareMatrix<float, 2> A2x2_I_check(data_2x2_check);
	EXPECT_EQ(A2x2_I, A2x2_I_check);

	SquareMatrix<float, 2> A2x2_sing = ones<float, 2, 2>();
	SquareMatrix<float, 2> A2x2_sing_I;
	EXPECT_FALSE(inv(A2x2_sing, A2x2_sing_I));

	SquareMatrix<float, 3> A3x3_sing = ones<float, 3, 3>();
	SquareMatrix<float, 3> A3x3_sing_I;
	EXPECT_FALSE(inv(A3x3_sing, A3x3_sing_I));

	// stess test
	SquareMatrix<float, n_large> A_large;
	A_large.setIdentity();
	SquareMatrix<float, n_large> A_large_I;
	A_large_I.setZero();

	for (size_t i = 0; i < n_large; i++) {
		A_large_I = inv(A_large);
		EXPECT_EQ(A_large, A_large_I);
	}

	SquareMatrix<float, 3> zero_test = zeros<float, 3, 3>();
	EXPECT_EQ(inv(zero_test), zero_test);

	// test pivotting
	float data2[81] = {
		-2,   1,   1,  -1,  -5,   1,   2,  -1,   0,
			-3,   2,  -1,   0,   2,   2,  -1,  -5,   3,
			0,   0,   0,   1,   4,  -3,   3,   0,  -2,
			2,   2,  -1,  -2,  -1,   0,   3,   0,   1,
			-1,   2,  -1,  -1,  -3,   3,   0,  -2,   3,
			0,   1,   1,  -3,   3,  -2,   0,  -4,   0,
			1,   0,   0,   0,   0,   0,  -2,   4,  -3,
			1,  -1,   0,  -1,  -1,   1,  -1,  -3,   4,
			0,   3,  -1,  -2,   2,   1,  -2,   0,  -1
		};

	float data2_check[81] = {
		6, -4,   3, -3, -9, -8, -10,   8,  14,
		-2, -7,  -5, -3, -2, -2, -16,  -5,   8,
		-2,  0, -23,  7, -24, -5, -28, -14,   9,
		3, -7,   2, -5,  -4, -6, -13,   4,  13,
		-1,  4,  -8,  5,  -8,  0,  -3,  -5,  -2,
		6,  7,  -7,  7, -21, -7,  -5,   3,   6,
		1,  4,  -4,  4,  -7, -1,   0,  -1,  -1,
		-7,  3, -11,  5,   1,  6,  -1, -13, -10,
		-8,  0, -11,  3,   3,  6,  -5, -14,  -8
	};
	SquareMatrix<float, 9> A2(data2);
	SquareMatrix<float, 9> A2_I = inv(A2);
	SquareMatrix<float, 9> A2_I_check(data2_check);
	EXPECT_TRUE(isEqual(A2_I, A2_I_check, 1e-3f));

	float data3[9] = {
		0, 1, 2,
		3, 4, 5,
		6, 7, 9
	};
	float data3_check[9] = {
		-0.3333333f, -1.6666666f, 1,
			-1, 4, -2,
			1, -2, 1
		};
	SquareMatrix<float, 3> A3(data3);
	SquareMatrix<float, 3> A3_I = inv(A3);
	SquareMatrix<float, 3> A3_I_check(data3_check);
	EXPECT_EQ(inv(A3), A3_I_check);
	EXPECT_EQ(A3_I, A3_I_check);
	EXPECT_TRUE(A3.I(A3_I));
	EXPECT_EQ(A3_I, A3_I_check);

	// cover singular matrices
	A3(0, 0) = 0;
	A3(0, 1) = 0;
	A3(0, 2) = 0;
	A3_I = inv(A3);
	SquareMatrix<float, 3>  Z3 = zeros<float, 3, 3>();
	EXPECT_FALSE(A3.I(A3_I));
	EXPECT_FALSE(Z3.I(A3_I));
	EXPECT_EQ(A3_I, Z3);
	EXPECT_EQ(A3.I(), Z3);

	for (size_t i = 0; i < 9; i++) {
		A2(0, i) = 0;
	}

	A2_I = inv(A2);
	SquareMatrix<float, 9> Z9 = zeros<float, 9, 9>();
	EXPECT_FALSE(A2.I(A2_I));
	EXPECT_FALSE(Z9.I(A2_I));
	EXPECT_EQ(A2_I, Z9);
	EXPECT_EQ(A2.I(), Z9);

	// cover NaN
	A3(0, 0) = NAN;
	A3(0, 1) = 0;
	A3(0, 2) = 0;
	A3_I = inv(A3);
	EXPECT_EQ(A3_I, Z3);
	EXPECT_EQ(A3.I(), Z3);

	A2(0, 0) = NAN;
	A2_I = inv(A2);
	EXPECT_EQ(A2_I, Z9);
	EXPECT_EQ(A2.I(), Z9);

	float data4[9] = {
		1.33471626f,  0.74946721f, -0.0531679f,
		0.74946721f,  1.07519593f,  0.08036323f,
		-0.0531679f,  0.08036323f,  1.01618474f
	};
	SquareMatrix<float, 3> A4(data4);

	float data4_cholesky[9] = {
		1.15529921f,  0.f,  0.f,
		0.6487213f,  0.80892311f,  0.f,
		-0.04602089f,  0.13625271f,  0.99774847f
	};
	SquareMatrix<float, 3> A4_cholesky_check(data4_cholesky);
	SquareMatrix<float, 3> A4_cholesky = cholesky(A4);
	EXPECT_EQ(A4_cholesky_check, A4_cholesky);

	SquareMatrix<float, 3> I3;
	I3.setIdentity();
	EXPECT_EQ(choleskyInv(A4)*A4, I3);
	EXPECT_EQ(cholesky(Z3), Z3);
}
