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
 * @file Matrix.h
 *
 * matrix code
 */

#pragma once


#include <inttypes.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "../Vector.hpp"
#include "../Matrix.hpp"

// arm specific
#include "../../CMSIS/Include/arm_math.h"

namespace math
{

class __EXPORT Matrix
{
public:
	// constructor
	Matrix(size_t rows, size_t cols) :
		_matrix() {
		arm_mat_init_f32(&_matrix,
				 rows, cols,
				 (float *)calloc(rows * cols, sizeof(float)));
	}
	Matrix(size_t rows, size_t cols, const float *data) :
		_matrix() {
		arm_mat_init_f32(&_matrix,
				 rows, cols,
				 (float *)malloc(rows * cols * sizeof(float)));
		memcpy(getData(), data, getSize());
	}
	// deconstructor
	virtual ~Matrix() {
		delete [] _matrix.pData;
	}
	// copy constructor (deep)
	Matrix(const Matrix &right) :
		_matrix() {
		arm_mat_init_f32(&_matrix,
				 right.getRows(), right.getCols(),
				 (float *)malloc(right.getRows()*
						 right.getCols()*sizeof(float)));
		memcpy(getData(), right.getData(),
		       getSize());
	}
	// assignment
	inline Matrix &operator=(const Matrix &right) {
#ifdef MATRIX_ASSERT
		ASSERT(getRows() == right.getRows());
		ASSERT(getCols() == right.getCols());
#endif

		if (this != &right) {
			memcpy(getData(), right.getData(),
			       right.getSize());
		}

		return *this;
	}
	// element accessors
	inline float &operator()(size_t i, size_t j) {
#ifdef MATRIX_ASSERT
		ASSERT(i < getRows());
		ASSERT(j < getCols());
#endif
		return getData()[i * getCols() + j];
	}
	inline const float &operator()(size_t i, size_t j) const {
#ifdef MATRIX_ASSERT
		ASSERT(i < getRows());
		ASSERT(j < getCols());
#endif
		return getData()[i * getCols() + j];
	}
	// output
	inline void print() const {
		for (size_t i = 0; i < getRows(); i++) {
			for (size_t j = 0; j < getCols(); j++) {
				float sig;
				int exponent;
				float num = (*this)(i, j);
				float2SigExp(num, sig, exponent);
				printf("%6.3fe%03d ", (double)sig, exponent);
			}

			printf("\n");
		}
	}
	// boolean ops
	inline bool operator==(const Matrix &right) const {
		for (size_t i = 0; i < getRows(); i++) {
			for (size_t j = 0; j < getCols(); j++) {
				if (fabsf((*this)(i, j) - right(i, j)) > 1e-30f)
					return false;
			}
		}

		return true;
	}
	// scalar ops
	inline Matrix operator+(float right) const {
		Matrix result(getRows(), getCols());
		arm_offset_f32((float *)getData(), right,
			       (float *)result.getData(), getRows()*getCols());
		return result;
	}
	inline Matrix operator-(float right) const {
		Matrix result(getRows(), getCols());
		arm_offset_f32((float *)getData(), -right,
			       (float *)result.getData(), getRows()*getCols());
		return result;
	}
	inline Matrix operator*(float right) const {
		Matrix result(getRows(), getCols());
		arm_mat_scale_f32(&_matrix, right,
				  &(result._matrix));
		return result;
	}
	inline Matrix operator/(float right) const {
		Matrix result(getRows(), getCols());
		arm_mat_scale_f32(&_matrix, 1.0f / right,
				  &(result._matrix));
		return result;
	}
	// vector ops
	inline Vector operator*(const Vector &right) const {
#ifdef MATRIX_ASSERT
		ASSERT(getCols() == right.getRows());
#endif
		Matrix resultMat = (*this) *
				   Matrix(right.getRows(), 1, right.getData());
		return Vector(getRows(), resultMat.getData());
	}
	// matrix ops
	inline Matrix operator+(const Matrix &right) const {
#ifdef MATRIX_ASSERT
		ASSERT(getRows() == right.getRows());
		ASSERT(getCols() == right.getCols());
#endif
		Matrix result(getRows(), getCols());
		arm_mat_add_f32(&_matrix, &(right._matrix),
				&(result._matrix));
		return result;
	}
	inline Matrix operator-(const Matrix &right) const {
#ifdef MATRIX_ASSERT
		ASSERT(getRows() == right.getRows());
		ASSERT(getCols() == right.getCols());
#endif
		Matrix result(getRows(), getCols());
		arm_mat_sub_f32(&_matrix, &(right._matrix),
				&(result._matrix));
		return result;
	}
	inline Matrix operator*(const Matrix &right) const {
#ifdef MATRIX_ASSERT
		ASSERT(getCols() == right.getRows());
#endif
		Matrix result(getRows(), right.getCols());
		arm_mat_mult_f32(&_matrix, &(right._matrix),
				 &(result._matrix));
		return result;
	}
	inline Matrix operator/(const Matrix &right) const {
#ifdef MATRIX_ASSERT
		ASSERT(right.getRows() == right.getCols());
		ASSERT(getCols() == right.getCols());
#endif
		return (*this) * right.inverse();
	}
	// other functions
	inline Matrix transpose() const {
		Matrix result(getCols(), getRows());
		arm_mat_trans_f32(&_matrix, &(result._matrix));
		return result;
	}
	inline void swapRows(size_t a, size_t b) {
		if (a == b) return;

		for (size_t j = 0; j < getCols(); j++) {
			float tmp = (*this)(a, j);
			(*this)(a, j) = (*this)(b, j);
			(*this)(b, j) = tmp;
		}
	}
	inline void swapCols(size_t a, size_t b) {
		if (a == b) return;

		for (size_t i = 0; i < getRows(); i++) {
			float tmp = (*this)(i, a);
			(*this)(i, a) = (*this)(i, b);
			(*this)(i, b) = tmp;
		}
	}
	/**
	 * inverse based on LU factorization with partial pivotting
	 */
	Matrix inverse() const {
#ifdef MATRIX_ASSERT
		ASSERT(getRows() == getCols());
#endif
		Matrix result(getRows(), getCols());
		Matrix work = (*this);
		arm_mat_inverse_f32(&(work._matrix),
				    &(result._matrix));
		return result;
	}
	inline void setAll(const float &val) {
		for (size_t i = 0; i < getRows(); i++) {
			for (size_t j = 0; j < getCols(); j++) {
				(*this)(i, j) = val;
			}
		}
	}
	inline void set(const float *data) {
		memcpy(getData(), data, getSize());
	}
	inline size_t getRows() const { return _matrix.numRows; }
	inline size_t getCols() const { return _matrix.numCols; }
	inline static Matrix identity(size_t size) {
		Matrix result(size, size);

		for (size_t i = 0; i < size; i++) {
			result(i, i) = 1.0f;
		}

		return result;
	}
	inline static Matrix zero(size_t size) {
		Matrix result(size, size);
		result.setAll(0.0f);
		return result;
	}
	inline static Matrix zero(size_t m, size_t n) {
		Matrix result(m, n);
		result.setAll(0.0f);
		return result;
	}
protected:
	inline size_t getSize() const { return sizeof(float) * getRows() * getCols(); }
	inline float *getData() { return _matrix.pData; }
	inline const float *getData() const { return _matrix.pData; }
	inline void setData(float *data) { _matrix.pData = data; }
private:
	arm_matrix_instance_f32 _matrix;
};

} // namespace math
