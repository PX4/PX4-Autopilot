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

TEST(MatrixHelperTest, SignFloat)
{
	EXPECT_FLOAT_EQ(sign(-100.f), -1.f);
	EXPECT_FLOAT_EQ(sign(-FLT_EPSILON), -1.f);
	EXPECT_FLOAT_EQ(sign(0.f), 0.f);
	EXPECT_FLOAT_EQ(sign(FLT_EPSILON), 1.f);
	EXPECT_FLOAT_EQ(sign(100.f), 1.f);
}

TEST(MatrixHelperTest, SignInt)
{
	EXPECT_FLOAT_EQ(sign(-100), -1);
	EXPECT_FLOAT_EQ(sign(-1), -1);
	EXPECT_FLOAT_EQ(sign(0), 0);
	EXPECT_FLOAT_EQ(sign(1), 1);
	EXPECT_FLOAT_EQ(sign(100), 1);
}

TEST(MatrixHelperTest, Helper)
{
	// general wraps
	EXPECT_FLOAT_EQ(wrap(4., 0., 10.), 4.);
	EXPECT_FLOAT_EQ(wrap(4., 0., 1.), 0.);
	EXPECT_FLOAT_EQ(wrap(-4., 0., 10.), 6.);
	EXPECT_FLOAT_EQ(wrap(-18., 0., 10.), 2.);
	EXPECT_FLOAT_EQ(wrap(-1.5, 3., 5.), 4.5);
	EXPECT_FLOAT_EQ(wrap(15.5, 3., 5.), 3.5);
	EXPECT_FLOAT_EQ(wrap(-1., 30., 40.), 39.);
	EXPECT_FLOAT_EQ(wrap(-8000., -555., 1.), -216.);
	EXPECT_FLOAT_EQ(wrap(0., 0., 360.), 0.);
	EXPECT_FLOAT_EQ(wrap(0. - FLT_EPSILON, 0., 360.), 360. - FLT_EPSILON);
	EXPECT_FLOAT_EQ(wrap(0. + FLT_EPSILON, 0., 360.), FLT_EPSILON);
	EXPECT_FLOAT_EQ(wrap(360., 0., 360.), 0.);
	EXPECT_FLOAT_EQ(wrap(360. - FLT_EPSILON, 0., 360.), 360. - FLT_EPSILON);
	EXPECT_FLOAT_EQ(wrap(360. + FLT_EPSILON, 0., 360.), FLT_EPSILON);

	// integer wraps
	EXPECT_EQ(wrap(-10, 0, 10), 0);
	EXPECT_EQ(wrap(-4, 0, 10), 6);
	EXPECT_EQ(wrap(0, 0, 10), 0);
	EXPECT_EQ(wrap(4, 0, 10), 4);
	EXPECT_EQ(wrap(10, 0, 10), 0);

	// wrap pi
	EXPECT_FLOAT_EQ(wrap_pi(0.), 0.);
	EXPECT_FLOAT_EQ(wrap_pi(4.), (4. - (2 * M_PI_PRECISE)));
	EXPECT_FLOAT_EQ(wrap_pi(-4.), (-4. + (2 * M_PI_PRECISE)));
	EXPECT_FLOAT_EQ(wrap_pi(3.), 3.);
	EXPECT_FLOAT_EQ(wrap_pi(100.), (100. - 32. * M_PI_PRECISE));
	EXPECT_FLOAT_EQ(wrap_pi(-100.), (-100. + 32. * M_PI_PRECISE));
	EXPECT_FLOAT_EQ(wrap_pi(-101.), (-101. + 32. * M_PI_PRECISE));
	EXPECT_FALSE(std::isfinite(wrap_pi(NAN)));

	// wrap 2pi
	EXPECT_FLOAT_EQ(wrap_2pi(0.), 0.);
	EXPECT_FLOAT_EQ(wrap_2pi(-4.), (-4. + 2. * M_PI_PRECISE));
	EXPECT_FLOAT_EQ(wrap_2pi(3.), (3.));
	EXPECT_FLOAT_EQ(wrap_2pi(200.), (200. - 31. * (2 * M_PI_PRECISE)));
	EXPECT_FLOAT_EQ(wrap_2pi(-201.), (-201. + 32. * (2 * M_PI_PRECISE)));
	EXPECT_FALSE(std::isfinite(wrap_2pi(NAN)));

	// Equality checks
	EXPECT_TRUE(isEqualF(1., 1.));
	EXPECT_FALSE(isEqualF(1., 2.));
	EXPECT_FALSE(isEqualF(NAN, 1.f));
	EXPECT_FALSE(isEqualF(1.f, NAN));
	EXPECT_FALSE(isEqualF(INFINITY, 1.f));
	EXPECT_FALSE(isEqualF(1.f, INFINITY));
	EXPECT_TRUE(isEqualF(NAN, NAN));
	EXPECT_TRUE(isEqualF(NAN, -NAN));
	EXPECT_TRUE(isEqualF(-NAN, NAN));
	EXPECT_TRUE(isEqualF(INFINITY, INFINITY));
	EXPECT_FALSE(isEqualF(INFINITY, -INFINITY));
	EXPECT_FALSE(isEqualF(-INFINITY, INFINITY));
	EXPECT_TRUE(isEqualF(-INFINITY, -INFINITY));

	Vector3f a(1, 2, 3);
	Vector3f b(4, 5, 6);
	EXPECT_FALSE(isEqual(a, b));
	EXPECT_TRUE(isEqual(a, a));

	Vector3f c(1, 2, 3);
	Vector3f d(1, 2, NAN);
	EXPECT_FALSE(isEqual(c, d));
	EXPECT_TRUE(isEqual(c, c));
	EXPECT_TRUE(isEqual(d, d));
}
