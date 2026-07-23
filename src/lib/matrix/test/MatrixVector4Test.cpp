/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

TEST(MatrixVector4Test, Vector4)
{
	Vector4f a(1.f, 2.f, 3.f, 4.f);
	EXPECT_EQ(a(0), 1.f);
	EXPECT_EQ(a(1), 2.f);
	EXPECT_EQ(a(2), 3.f);
	EXPECT_EQ(a(3), 4.f);

	Vector4f zero;
	EXPECT_EQ(zero, Vector4f(0.f, 0.f, 0.f, 0.f));

	Vector4f b(4.f, 3.f, 2.f, 1.f);
	EXPECT_EQ(a + b, Vector4f(5.f, 5.f, 5.f, 5.f));
	EXPECT_EQ(a - b, Vector4f(-3.f, -1.f, 1.f, 3.f));
	EXPECT_EQ(-a, Vector4f(-1.f, -2.f, -3.f, -4.f));
	EXPECT_EQ(a * 2.f, Vector4f(2.f, 4.f, 6.f, 8.f));
	EXPECT_FLOAT_EQ(a * b, 1.f * 4.f + 2.f * 3.f + 3.f * 2.f + 4.f * 1.f);

	Matrix<float, 4, 1> m(a);
	Vector4f from_matrix(m);
	EXPECT_EQ(from_matrix, a);

	float data[] = {7.f, 8.f, 9.f, 10.f};
	Vector4f from_array(data);
	EXPECT_EQ(from_array, Vector4f(7.f, 8.f, 9.f, 10.f));
}
