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
 * @file Vector2f.hpp
 *
 * math 3 vector
 */

#pragma once

#include "Vector.hpp"

namespace math
{

class __EXPORT Vector2f :
	public Vector
{
public:
	Vector2f();
	Vector2f(const Vector &right);
	Vector2f(float x, float y);
	Vector2f(const float *data);
	virtual ~Vector2f();
	float cross(const Vector2f &b) const;
	float operator %(const Vector2f &v) const;
    float operator *(const Vector2f &v) const;
    inline Vector2f operator*(const float &right) const {
		return Vector::operator*(right);
	}

	/**
	 * accessors
	 */
	void setX(float x) { (*this)(0) = x; }
	void setY(float y) { (*this)(1) = y; }
	const float &getX() const { return (*this)(0); }
	const float &getY() const { return (*this)(1); }
};
    
class __EXPORT Vector2 :
	public Vector2f
{
};

int __EXPORT vector2fTest();
} // math

