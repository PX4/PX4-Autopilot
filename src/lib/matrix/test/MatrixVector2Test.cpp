
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

TEST(MatrixVector2Test, Vector2)
{
	Vector2f a(1, 0);
	Vector2f b(0, 1);
	EXPECT_FLOAT_EQ(a % b, 1.f);

	Vector2f c;
	EXPECT_FLOAT_EQ(c(0), 0.f);
	EXPECT_FLOAT_EQ(c(1), 0.f);

	Matrix<float, 2, 1> d(a);
	EXPECT_FLOAT_EQ(d(0, 0), 1.f);
	EXPECT_FLOAT_EQ(d(1, 0), 0.f);

	Vector2f e(d);
	EXPECT_FLOAT_EQ(e(0), 1.f);
	EXPECT_FLOAT_EQ(e(1), 0.f);

	float data[] = {4, 5};
	Vector2f f(data);
	EXPECT_FLOAT_EQ(f(0), 4.f);
	EXPECT_FLOAT_EQ(f(1), 5.f);

	Vector3f g(1.23f, 423.4f, 3221.f);
	Vector2f h(g);
	EXPECT_FLOAT_EQ(h(0), 1.23f);
	EXPECT_FLOAT_EQ(h(1), 423.4f);
}
