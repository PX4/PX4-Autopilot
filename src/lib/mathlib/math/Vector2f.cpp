/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file Vector2f.cpp
 *
 * math vector
 */

#include "test/test.hpp"

#include "Vector2f.hpp"

namespace math
{

Vector2f::Vector2f() :
	Vector(2)
{
}

Vector2f::Vector2f(const Vector &right) :
	Vector(right)
{
#ifdef VECTOR_ASSERT
	ASSERT(right.getRows() == 2);
#endif
}

Vector2f::Vector2f(float x, float y) :
	Vector(2)
{
	setX(x);
	setY(y);
}

Vector2f::Vector2f(const float *data) :
	Vector(2, data)
{
}

Vector2f::~Vector2f()
{
}

float Vector2f::cross(const Vector2f &b) const
{
	const Vector2f &a = *this;
	return a(0)*b(1) - a(1)*b(0);
}

float Vector2f::operator %(const Vector2f &v) const
{
	return cross(v);
}
    
float Vector2f::operator *(const Vector2f &v) const
{
    return dot(v);
}

int __EXPORT vector2fTest()
{
	printf("Test Vector2f\t\t: ");
	// test float ctor
	Vector2f v(1, 2);
	ASSERT(equal(v(0), 1));
	ASSERT(equal(v(1), 2));
	printf("PASS\n");
	return 0;
}

} // namespace math
