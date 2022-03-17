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

TEST(MatrixVectorTest, Vector)
{
	// test data
	float data1[] = {1, 2, 3, 4, 5};
	float data2[] = {6, 7, 8, 9, 10};
	Vector<float, 5> v1(data1);
	Vector<float, 5> v2(data2);

	// copy constructor
	Vector<float, 5> v3(v2);
	EXPECT_EQ(v2, v3);

	// norm, dot product
	EXPECT_FLOAT_EQ(v1.norm(), 7.416198487095663f);
	EXPECT_FLOAT_EQ(v1.norm_squared(), v1.norm() * v1.norm());
	EXPECT_FLOAT_EQ(v1.norm(), v1.length());
	EXPECT_FLOAT_EQ(v1.dot(v2), 130.0f);
	EXPECT_FLOAT_EQ(v1.dot(v2), v1 * v2);

	// unit, unit_zero, normalize
	EXPECT_FLOAT_EQ(v2.unit().norm(), 1.f);
	EXPECT_FLOAT_EQ(v2.unit_or_zero().norm(), 1.f);
	EXPECT_FLOAT_EQ((Vector<float, 5>().unit_or_zero().norm()), 0.f);
	v2.normalize();
	EXPECT_FLOAT_EQ(v2.norm(), 1.f);

	// sqrt
	float data1_sq[] = {1, 4, 9, 16, 25};
	Vector<float, 5> v4(data1_sq);
	EXPECT_EQ(v1, v4.sqrt());

	// longerThan
	Vector<float, 2> v5;
	v5(0) = 3;
	v5(1) = 4;
	EXPECT_TRUE(v5.longerThan(4.99f));
	EXPECT_FALSE(v5.longerThan(5.f));
}
