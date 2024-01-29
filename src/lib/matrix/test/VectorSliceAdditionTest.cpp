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

/**
 * @brief Test the addition functionality between a Vector and Slice object
 *
 * Tests whether addition between Slice and Vector works.
 */
TEST(VectorSliceAdditionTest, Slice)
{
	Vector2f A{1.0f, 5.0f};
	Vector3f B{-1.0f, 4.0f, 0.0f};

	Vector2f Sum{0.0f, 9.0f};

	// Vector2 + Slice<2, 1, .., ..>
	// Vector + Slice is supported by default, as Slice gets converted into a Vector2, then gets added
	EXPECT_EQ(A + B.xy(), Sum);
}

TEST(SliceVectorAdditionTest, Slice)
{
	Vector2f A{1.0f, 5.0f};
	Vector3f B{-1.0f, 4.0f, 0.0f};

	Vector2f Sum{0.0f, 9.0f};

	// Slice<2, 1, .., ..> + Vector2
	// This should provoke the operator+() function to return Vector2 when we do : Slice + Vector
	EXPECT_EQ(B.xy() + A, Sum);
}
