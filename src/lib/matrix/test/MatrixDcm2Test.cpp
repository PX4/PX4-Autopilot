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
#include <matrix/Dcm.hpp>
#include <matrix/Dcm2.hpp>

using namespace matrix;

TEST(MatrixDcm2Test, CompareToDcm)
{
	// 2D matrix and 3D matrix rotating around yaw are equivalent
	Dcm2f R2(1.f);
	Dcmf R3(Eulerf(0.f, 0.f, 1.f));
	EXPECT_EQ(Dcm2f(R3.slice<2, 2>(0, 0)), R2);

	// Rotating a 2D and a 3D vector respectively is equivalent
	Vector2f v2(1.f, 2.f);
	Vector3f v3(v2(0), v2(1), 0.f);
	Vector2f result2 = R2 * v2;
	Vector3f result3 = R3 * v3;
	EXPECT_EQ(Vector2f(result2), result2);
	EXPECT_EQ(result2, result3.xy());
}

TEST(MatrixDcm2Test, 45DegreeRotation)
{
	Vector2f v(1.0f, 0.0f);
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(1.f, 1.f) * (sqrt(2) / 2));
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(0.f, 1.f));
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(-1.f, 1.f) * (sqrt(2) / 2));
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(-1.f, 0.f));
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(-1.f, -1.f) * (sqrt(2) / 2));
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(0.f, -1.f));
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(1.f, -1.f) * (sqrt(2) / 2));
	v = Dcm2f(M_PI / 4.f) * v;
	EXPECT_EQ(v, Vector2f(1.f, 0.f));
}
