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

#include <stdio.h>
#include <math.h>
#include "../CMSIS/Include/arm_math.h"

namespace math
{

template <unsigned int N>
class Vector;

template <unsigned int N>
class VectorBase {
public:
	/**
	 * vector data
	 */
	float data[N];

	/**
	 * struct for using arm_math functions, represents column vector
	 */
	arm_matrix_instance_f32 arm_col;

	/**
	 * trivial ctor
	 */
	VectorBase() {
		arm_col = {N, 1, &data[0]};
	}

	/**
	 * copy ctor
	 */
	VectorBase(const VectorBase<N> &v) {
		arm_col = {N, 1, &data[0]};
		memcpy(data, v.data, sizeof(data));
	}

	/**
	 * setting ctor
	 */
	VectorBase(const float *d) {
		arm_col = {N, 1, &data[0]};
		memcpy(data, d, sizeof(data));
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

	unsigned int getRows() {
		return N;
	}

	unsigned int getCols() {
		return 1;
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
		return *reinterpret_cast<const Vector<N>*>(this);
	}

	/**
	 * negation
	 */
	const Vector<N> operator -(void) const {
		Vector<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res.data[i] = -data[i];
 		return res;
	}

	/**
	 * addition
	 */
	const Vector<N> operator +(const Vector<N> &v) const {
		Vector<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res.data[i] = data[i] + v(i);
 		return res;
	}

	/**
	 * subtraction
	 */
	const Vector<N> operator -(const Vector<N> &v) const {
		Vector<N> res;
 		for (unsigned int i = 0; i < N; i++)
 			res.data[i] = data[i] - v(i);
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
		Vector<N> temp(*reinterpret_cast<const Vector<N>*>(this));
		return temp /= num;
	}

	/**
	 * addition
	 */
	const Vector<N> &operator +=(const Vector<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] += v(i);
		return *reinterpret_cast<const Vector<N>*>(this);
	}

	/**
	 * subtraction
	 */
	const Vector<N> &operator -=(const Vector<N> &v) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] -= v(i);
		return *reinterpret_cast<const Vector<N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> &operator *=(const float num) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] *= num;
		return *reinterpret_cast<const Vector<N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> &operator /=(const float num) {
 		for (unsigned int i = 0; i < N; i++)
 			data[i] /= num;
		return *reinterpret_cast<const Vector<N>*>(this);
	}

	/**
	 * dot product
	 */
	float operator *(const Vector<N> &v) const {
		float res = 0.0f;
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
		return sqrtf(*this * *reinterpret_cast<const Vector<N>*>(this));
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

	void print(void) {
		printf("[ ");
 		for (unsigned int i = 0; i < N; i++)
			printf("%.3f\t", data[i]);
		printf("]\n");
	}
};

template <unsigned int N>
class Vector : public VectorBase<N> {
public:
	using VectorBase<N>::operator *;

	Vector() : VectorBase<N>() {
	}

	Vector(const float d[]) : VectorBase<N>(d) {
	}

	Vector(const Vector<N> &v) : VectorBase<N>(v) {
	}

	Vector(const VectorBase<N> &v) : VectorBase<N>(v) {
	}

	/**
	 * set to value
	 */
	const Vector<N> &operator =(const Vector<N> &v) {
		this->arm_col = {N, 1, &this->data[0]};
		memcpy(this->data, v.data, sizeof(this->data));
		return *this;
	}
};

template <>
class Vector<2> : public VectorBase<2> {
public:
	Vector() : VectorBase<2>() {
	}

	Vector(const float x, const float y) : VectorBase() {
		data[0] = x;
		data[1] = y;
	}

	Vector(const Vector<2> &v) : VectorBase() {
		data[0] = v.data[0];
		data[1] = v.data[1];
	}

	Vector(const VectorBase<2> &v) : VectorBase() {
		data[0] = v.data[0];
		data[1] = v.data[1];
	}

	/**
	 * set to value
	 */
	const Vector<2> &operator =(const Vector<2> &v) {
		data[0] = v.data[0];
		data[1] = v.data[1];
		return *this;
	}

	float cross(const Vector<2> &b) const {
	        return data[0]*b.data[1] - data[1]*b.data[0];
	}

	float operator %(const Vector<2> &v) const {
	        return cross(v);
	}

};

template <>
class Vector<3> : public VectorBase<3> {
public:
	Vector() {
		arm_col = {3, 1, &this->data[0]};
	}

	Vector(const float x, const float y, const float z) {
		arm_col = {3, 1, &this->data[0]};
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}

	Vector(const Vector<3> &v) : VectorBase<3>() {
		data[0] = v.data[0];
		data[1] = v.data[1];
		data[2] = v.data[2];
	}

	/**
	 * setting ctor
	 */
	Vector(const float d[]) {
		arm_col = {3, 1, &this->data[0]};
		data[0] = d[0];
		data[1] = d[1];
		data[2] = d[2];
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

template <>
class Vector<4> : public VectorBase<4> {
public:
	Vector() : VectorBase() {}

	Vector(const float x, const float y, const float z, const float t) : VectorBase() {
		data[0] = x;
		data[1] = y;
		data[2] = z;
		data[3] = t;
	}

	Vector(const Vector<4> &v) : VectorBase() {
		data[0] = v.data[0];
		data[1] = v.data[1];
		data[2] = v.data[2];
		data[3] = v.data[3];
	}

	Vector(const VectorBase<4> &v) : VectorBase() {
		data[0] = v.data[0];
		data[1] = v.data[1];
		data[2] = v.data[2];
		data[3] = v.data[3];
	}

	/**
	 * setting ctor
	 */
	/*
	Vector(const float d[]) {
		arm_col = {3, 1, &this->data[0]};
		data[0] = d[0];
		data[1] = d[1];
		data[2] = d[2];
	}
*/
	/**
	 * set to value
	 */
	/*
	const Vector<3> &operator =(const Vector<3> &v) {
		data[0] = v.data[0];
		data[1] = v.data[1];
		data[2] = v.data[2];
		return *this;
	}
	*/
};

}

#endif // VECTOR_HPP
