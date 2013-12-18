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
 * @file Vector.hpp
 *
 * Generic Vector
 */

#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <math.h>
#include "../CMSIS/Include/arm_math.h"

namespace math
{

template <unsigned int N>
class Vector {
public:
	float data[N];
	arm_matrix_instance_f32 arm_col;

	/**
	 * trivial ctor
	 */
	Vector<N>() {
		arm_col = {N, 1, &data[0]};
	}

	/**
	 * setting ctor
	 */
	Vector<N>(const float *d) {
		memcpy(data, d, sizeof(data));
		arm_col = {N, 1, &data[0]};
	}

	/**
	 * access to elements by index
	 */
	inline float &operator ()(unsigned int i) {
		return data[i];
	}

	/**
	 * access to elements by index
	 */
	inline const float &operator ()(unsigned int i) const {
		return data[i];
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Vector<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			if (data[i] != v(i))
 				return false;
 		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Vector<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			if (data[i] != v(i))
 				return true;
 		return false;
	}

	/**
	 * set to value
	 */
	const Vector<N> &operator =(const Vector<N> &v) {
		memcpy(data, v.data, sizeof(data));
		return *this;
	}

	/**
	 * negation
	 */
	const Vector<N> operator -(void) const {
		Vector<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res[i] = -data[i];
 		return res;
	}

	/**
	 * addition
	 */
	const Vector<N> operator +(const Vector<N> &v) const {
		Vector<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res[i] = data[i] + v(i);
 		return res;
	}

	/**
	 * subtraction
	 */
	const Vector<N> operator -(const Vector<N> &v) const {
		Vector<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res[i] = data[i] - v(i);
 		return res;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> operator *(const float num) const {
		Vector<N> temp(*this);
		return temp *= num;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> operator /(const float num) const {
		Vector<N> temp(*this);
		return temp /= num;
	}

	/**
	 * addition
	 */
	const Vector<N> &operator +=(const Vector<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] += v(i);
		return *this;
	}

	/**
	 * subtraction
	 */
	const Vector<N> &operator -=(const Vector<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] -= v(i);
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> &operator *=(const float num) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] *= num;
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> &operator /=(const float num) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] /= num;
		return *this;
	}

	/**
	 * dot product
	 */
	float operator *(const Vector<N> &v) const {
		float res;
 		for (unsigned int i = 0; i < N; i++)
 			res += data[i] * v(i);
 		return res;
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
		return sqrtf(*this * *this);
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
	Vector<N> normalized() const {
		return *this / length();
	}
};

}

#endif // VECTOR_HPP
