/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file Vector3.cpp
 *
 * math vector
 */

#include "test/test.hpp"

#include "Vector3.hpp"

namespace math
{

Vector3::Vector3() :
	Vector(3)
{
}

Vector3::Vector3(const Vector &right) :
	Vector(right)
{
#ifdef VECTOR_ASSERT
	ASSERT(right.getRows() == 3);
#endif
}

Vector3::Vector3(float x, float y, float z) :
	Vector(3)
{
	setX(x);
	setY(y);
	setZ(z);
}

Vector3::Vector3(const float *data) :
	Vector(3, data)
{
}

Vector3::~Vector3()
{
}

Vector3 Vector3::cross(const Vector3 &b) const
{
	const Vector3 &a = *this;
	Vector3 result;
	result(0) = a(1) * b(2) - a(2) * b(1);
	result(1) = a(2) * b(0) - a(0) * b(2);
	result(2) = a(0) * b(1) - a(1) * b(0);
	return result;
}

int __EXPORT vector3Test()
{
	printf("Test Vector3\t\t: ");
	// test float ctor
	Vector3 v(1, 2, 3);
	ASSERT(equal(v(0), 1));
	ASSERT(equal(v(1), 2));
	ASSERT(equal(v(2), 3));
	printf("PASS\n");
	return 0;
}

} // namespace math
