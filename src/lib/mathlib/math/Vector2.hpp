/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Will Perone <will.perone@gmail.com>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 * @file Vector2.hpp
 *
 * 2D Vector
 */

#ifndef VECTOR2_HPP
#define VECTOR2_HPP

#include <math.h>

namespace math
{

template <typename T>
class Vector2 {
public:
	T x, y;

	/**
	 * trivial ctor
	 */
	Vector2<T>() {
	}

	/**
	 * setting ctor
	 */
	Vector2<T>(const T x0, const T y0): x(x0), y(y0) {
	}

	/**
	 * setter
	 */
	void set(const T x0, const T y0) {
		x = x0;
		y = y0;
	}

	/**
	 * access to elements by index
	 */
	T operator ()(unsigned int i) {
		return *(&x + i);
	}

	/**
	 * access to elements by index
	 */
	const T operator ()(unsigned int i) const {
		return *(&x + i);
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Vector2<T> &v) {
		return (x == v.x && y == v.y);
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Vector2<T> &v) {
		return (x != v.x || y != v.y);
	}

	/**
	 * set to value
	 */
	const Vector2<T> &operator =(const Vector2<T> &v) {
		x = v.x;
		y = v.y;
		return *this;
	}

	/**
	 * negation
	 */
	const Vector2<T> operator -(void) const {
		return Vector2<T>(-x, -y);
	}

	/**
	 * addition
	 */
	const Vector2<T> operator +(const Vector2<T> &v) const {
		return Vector2<T>(x + v.x, y + v.y);
	}

	/**
	 * subtraction
	 */
	const Vector2<T> operator -(const Vector2<T> &v) const {
		return Vector2<T>(x - v.x, y - v.y);
	}

	/**
	 * uniform scaling
	 */
	const Vector2<T> operator *(const T num) const {
		Vector2<T> temp(*this);
		return temp *= num;
	}

	/**
	 * uniform scaling
	 */
	const Vector2<T> operator /(const T num) const {
		Vector2<T> temp(*this);
		return temp /= num;
	}

	/**
	 * addition
	 */
	const Vector2<T> &operator +=(const Vector2<T> &v) {
		x += v.x;
		y += v.y;
		return *this;
	}

	/**
	 * subtraction
	 */
	const Vector2<T> &operator -=(const Vector2<T> &v) {
		x -= v.x;
		y -= v.y;
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector2<T> &operator *=(const T num) {
		x *= num;
		y *= num;
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector2<T> &operator /=(const T num) {
		x /= num;
		y /= num;
		return *this;
	}

	/**
	 * dot product
	 */
	T operator *(const Vector2<T> &v) const {
		return x * v.x + y * v.y;
	}

	/**
	 * cross product
	 */
	const float operator %(const Vector2<T> &v) const {
		return x * v.y - y * v.x;
	}

	/**
	 * gets the length of this vector squared
	 */
	float length_squared() const {
		return (*this * *this);
	}

	/**
	 * gets the length of this vector
	 */
	float length() const {
		return (T)sqrt(*this * *this);
	}

	/**
	 * normalizes this vector
	 */
	void normalize() {
		*this /= length();
	}

	/**
	 * returns the normalized version of this vector
	 */
	Vector2<T> normalized() const {
		return *this / length();
	}

	/**
	 * reflects this vector about n
	 */
	void reflect(const Vector2<T> &n)
	{
		Vector2<T> orig(*this);
		project(n);
		*this = *this * 2 - orig;
	}

	/**
	 * projects this vector onto v
	 */
	void project(const Vector2<T> &v) {
		*this = v * (*this * v) / (v * v);
	}

	/**
	 * returns this vector projected onto v
	 */
	Vector2<T> projected(const Vector2<T> &v) {
		return v * (*this * v) / (v * v);
	}

	/**
	 * computes the angle between 2 arbitrary vectors
	 */
	static inline float angle(const Vector2<T> &v1, const Vector2<T> &v2) {
		return acosf((v1 * v2) / (v1.length() * v2.length()));
	}

	/**
	 * computes the angle between 2 arbitrary normalized vectors
	 */
	static inline float angle_normalized(const Vector2<T> &v1, const Vector2<T> &v2) {
		return acosf(v1 * v2);
	}
};

typedef Vector2<float> Vector2f;
}

#endif // VECTOR2_HPP
