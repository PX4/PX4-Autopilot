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

TEST(MatrixVector3Test, Vector3)
{
	Vector3f a(1, 0, 0);
	Vector3f b(0, 1, 0);
	Vector3f c = a.cross(b);
	EXPECT_EQ(c, Vector3f(0, 0, 1));
	c = a % b;
	EXPECT_EQ(c, Vector3f(0, 0, 1));
	Matrix<float, 3, 1> d(c);
	Vector3f e(d);
	EXPECT_EQ(e, d);
	float data[] = {4, 5, 6};
	Vector3f f(data);
	EXPECT_EQ(f, Vector3f(4, 5, 6));

	EXPECT_EQ(a + b, Vector3f(1, 1, 0));
	EXPECT_EQ(a - b, Vector3f(1, -1, 0));
	EXPECT_FLOAT_EQ(a * b, 0.0f);
	EXPECT_EQ(-a, Vector3f(-1, 0, 0));
	EXPECT_EQ(a.unit(), a);
	EXPECT_EQ(a.unit(), a.normalized());
	EXPECT_EQ(a * 2.0, Vector3f(2, 0, 0));

	Vector2f g2(1, 3);
	Vector3f g3(7, 11, 17);
	g3.xy() = g2;
	EXPECT_EQ(g3, Vector3f(1, 3, 17));

	const Vector3f g4(g3);
	Vector2f g5 = g4.xy();
	EXPECT_EQ(g5, g2);
	EXPECT_EQ(g2, Vector2f(g4.xy()));

	Vector3f h;
	EXPECT_EQ(h, Vector3f(0, 0, 0));
	Vector4f j(1.f, 2.f, 3.f, 4.f);
	Vector3f k = j.slice<3, 1>(0, 0);
	Vector3f k_test(1, 2, 3);
	EXPECT_EQ(k, k_test);

	Vector3f m1(1, 2, 3);
	Vector3f m2(3.1f, 4.1f, 5.1f);
	EXPECT_EQ(m2, m1 + 2.1f);
	EXPECT_EQ(m2 - 2.1f, m1);

	// Test Addition and Subtraction of Slices
	Vector3f v1(3, 13, 0);
	Vector3f v2(42, 6, 256);

	EXPECT_EQ(v1.xy() - v2.xy(), Vector2f(-39, 7));
	EXPECT_EQ(v1.xy() + v2.xy(), Vector2f(45, 19));
	EXPECT_EQ(v1.xy() + 2.f, Vector2f(5, 15));
	EXPECT_EQ(v1.xy() - 2.f, Vector2f(1, 11));
}
