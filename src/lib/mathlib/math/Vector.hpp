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
class VectorBase {
public:
	float data[N];
	arm_matrix_instance_f32 arm_col;

	/**
	 * trivial ctor
	 */
	VectorBase<N>() {
		arm_col = {N, 1, &data[0]};
	}

	/**
	 * setting ctor
	 */
//	VectorBase<N>(const float *d) {
	//	memcpy(data, d, sizeof(data));
		//arm_col = {N, 1, &data[0]};
	//}

	/**
	 * setting ctor
	 */
	VectorBase<N>(const float d[]) : data(d) {
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
	bool operator ==(const VectorBase<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			if (data[i] != v(i))
 				return false;
 		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const VectorBase<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			if (data[i] != v(i))
 				return true;
 		return false;
	}

	/**
	 * set to value
	 */
	const VectorBase<N> &operator =(const VectorBase<N> &v) {
		memcpy(data, v.data, sizeof(data));
		return *this;
	}

	/**
	 * negation
	 */
	const VectorBase<N> operator -(void) const {
		VectorBase<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res[i] = -data[i];
 		return res;
	}

	/**
	 * addition
	 */
	const VectorBase<N> operator +(const VectorBase<N> &v) const {
		VectorBase<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res[i] = data[i] + v(i);
 		return res;
	}

	/**
	 * subtraction
	 */
	const VectorBase<N> operator -(const VectorBase<N> &v) const {
		VectorBase<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res[i] = data[i] - v(i);
 		return res;
	}

	/**
	 * uniform scaling
	 */
	const VectorBase<N> operator *(const float num) const {
		VectorBase<N> temp(*this);
		return temp *= num;
	}

	/**
	 * uniform scaling
	 */
	const VectorBase<N> operator /(const float num) const {
		VectorBase<N> temp(*this);
		return temp /= num;
	}

	/**
	 * addition
	 */
	const VectorBase<N> &operator +=(const VectorBase<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] += v(i);
		return *this;
	}

	/**
	 * subtraction
	 */
	const VectorBase<N> &operator -=(const VectorBase<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] -= v(i);
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const VectorBase<N> &operator *=(const float num) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] *= num;
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const VectorBase<N> &operator /=(const float num) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] /= num;
		return *this;
	}

	/**
	 * dot product
	 */
	float operator *(const VectorBase<N> &v) const {
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
	VectorBase<N> normalized() const {
		return *this / length();
	}
};

template <unsigned int N>
class Vector : public VectorBase<N> {
public:
	/**
	 * set to value
	 */
	const Vector<N> &operator =(const Vector<N> &v) {
		memcpy(this->data, v.data, sizeof(this->data));
		return *this;
	}
};

template <>
class Vector<3> : public VectorBase<3> {
public:
	Vector<3>() {
		arm_col = {3, 1, &this->data[0]};
	}

	Vector<3>(const float x, const float y, const float z) {
		data[0] = x;
		data[1] = y;
		data[2] = z;
		arm_col = {3, 1, &this->data[0]};
	}

	/**
	 * set to value
	 */
	const Vector<3> &operator =(const Vector<3> &v) {
		data[0] = v.data[0];
		data[1] = v.data[1];
		data[2] = v.data[2];
		return *this;
	}
};

}

#endif // VECTOR_HPP
