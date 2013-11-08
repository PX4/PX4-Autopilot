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
 * @file Vector.h
 *
 * math vector
 */

#pragma once

#include <inttypes.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "../Vector.hpp"

namespace math
{

class __EXPORT Vector
{
public:
	// constructor
	Vector(size_t rows) :
		_rows(rows),
		_data((float *)calloc(rows, sizeof(float))) {
	}
	Vector(size_t rows, const float *data) :
		_rows(rows),
		_data((float *)malloc(getSize())) {
		memcpy(getData(), data, getSize());
	}
	// deconstructor
	virtual ~Vector() {
		delete [] getData();
	}
	// copy constructor (deep)
	Vector(const Vector &right) :
		_rows(right.getRows()),
		_data((float *)malloc(getSize())) {
		memcpy(getData(), right.getData(),
		       right.getSize());
	}
	// assignment
	inline Vector &operator=(const Vector &right) {
#ifdef VECTOR_ASSERT
		ASSERT(getRows() == right.getRows());
#endif

		if (this != &right) {
			memcpy(getData(), right.getData(),
			       right.getSize());
		}

		return *this;
	}
	// element accessors
	inline float &operator()(size_t i) {
#ifdef VECTOR_ASSERT
		ASSERT(i < getRows());
#endif
		return getData()[i];
	}
	inline const float &operator()(size_t i) const {
#ifdef VECTOR_ASSERT
		ASSERT(i < getRows());
#endif
		return getData()[i];
	}
	// output
	inline void print() const {
		for (size_t i = 0; i < getRows(); i++) {
			float sig;
			int exp;
			float num = (*this)(i);
			float2SigExp(num, sig, exp);
			printf("%6.3fe%03.3d,", (double)sig, exp);
		}

		printf("\n");
	}
	// boolean ops
	inline bool operator==(const Vector &right) const {
		for (size_t i = 0; i < getRows(); i++) {
			if (fabsf(((*this)(i) - right(i))) > 1e-30f)
				return false;
		}

		return true;
	}
	// scalar ops
	inline Vector operator+(const float &right) const {
		Vector result(getRows());

		for (size_t i = 0; i < getRows(); i++) {
			result(i) = (*this)(i) + right;
		}

		return result;
	}
	inline Vector operator-(const float &right) const {
		Vector result(getRows());

		for (size_t i = 0; i < getRows(); i++) {
			result(i) = (*this)(i) - right;
		}

		return result;
	}
	inline Vector operator*(const float &right) const {
		Vector result(getRows());

		for (size_t i = 0; i < getRows(); i++) {
			result(i) = (*this)(i) * right;
		}

		return result;
	}
	inline Vector operator/(const float &right) const {
		Vector result(getRows());

		for (size_t i = 0; i < getRows(); i++) {
			result(i) = (*this)(i) / right;
		}

		return result;
	}
	// vector ops
	inline Vector operator+(const Vector &right) const {
#ifdef VECTOR_ASSERT
		ASSERT(getRows() == right.getRows());
#endif
		Vector result(getRows());

		for (size_t i = 0; i < getRows(); i++) {
			result(i) = (*this)(i) + right(i);
		}

		return result;
	}
	inline Vector operator-(const Vector &right) const {
#ifdef VECTOR_ASSERT
		ASSERT(getRows() == right.getRows());
#endif
		Vector result(getRows());

		for (size_t i = 0; i < getRows(); i++) {
			result(i) = (*this)(i) - right(i);
		}

		return result;
	}
	inline Vector operator-(void) const {
		Vector result(getRows());

		for (size_t i = 0; i < getRows(); i++) {
			result(i) = -((*this)(i));
		}

		return result;
	}
	// other functions
	inline float dot(const Vector &right) const {
		float result = 0;

		for (size_t i = 0; i < getRows(); i++) {
			result += (*this)(i) * (*this)(i);
		}

		return result;
	}
	inline float norm() const {
		return sqrtf(dot(*this));
	}
	inline float length() const {
		return norm();
	}
	inline Vector unit() const {
		return (*this) / norm();
	}
	inline Vector normalized() const {
		return unit();
	}
	inline void normalize() {
		(*this) = (*this) / norm();
	}
	inline static Vector zero(size_t rows) {
		Vector result(rows);
		// calloc returns zeroed memory
		return result;
	}
	inline void setAll(const float &val) {
		for (size_t i = 0; i < getRows(); i++) {
			(*this)(i) = val;
		}
	}
	inline void set(const float *data) {
		memcpy(getData(), data, getSize());
	}
	inline size_t getRows() const { return _rows; }
protected:
	inline size_t getSize() const { return sizeof(float) * getRows(); }
	inline float *getData() { return _data; }
	inline const float *getData() const { return _data; }
	inline void setData(float *data) { _data = data; }
private:
	size_t _rows;
	float *_data;
};

} // math
