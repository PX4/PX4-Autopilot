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
 * @file Matrix3.hpp
 *
 * 3x3 Matrix
 */

#ifndef MATRIX_HPP
#define MATRIX_HPP

#include "../CMSIS/Include/arm_math.h"

namespace math
{

template <unsigned int N, unsigned int M>
class Matrix;

// MxN matrix with float elements
template <unsigned int N, unsigned int M>
class MatrixBase {
public:
	/**
	 * matrix data[row][col]
	 */
	float data[M][N];

	/**
	 * struct for using arm_math functions
	 */
	arm_matrix_instance_f32 arm_mat;

	/**
	 * trivial ctor
	 * note that this ctor will not initialize elements
	 */
	MatrixBase() {
		//arm_mat_init_f32(&arm_mat, M, N, data);
		arm_mat = {M, N, &data[0][0]};
	}

	/**
	 * access by index
	 */
	inline float &operator ()(unsigned int row, unsigned int col) {
		return data[row][col];
	}

	/**
	 * access by index
	 */
	inline const float &operator ()(unsigned int row, unsigned int col) const {
		return data[row][col];
	}

	unsigned int getRows() {
		return M;
	}

	unsigned int getCols() {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const MatrixBase<M, N> &m) {
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				if (data[i][j] != m(i, j))
 					return false;
		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const MatrixBase<M, N> &m) {
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				if (data[i][j] != m(i, j))
 					return true;
		return false;
	}

	/**
	 * set to value
	 */
	const MatrixBase<M, N> &operator =(const MatrixBase<M, N> &m) {
		memcpy(data, m.data, sizeof(data));
		return *this;
	}

	/**
	 * negation
	 */
	MatrixBase<M, N> operator -(void) const {
		MatrixBase<M, N> res;
 		for (unsigned int i = 0; i < N; i++)
 			for (unsigned int j = 0; j < M; j++)
 				res[i][j] = -data[i][j];
		return res;
	}

	/**
	 * addition
	 */
	MatrixBase<M, N> operator +(const MatrixBase<M, N> &m) const {
		MatrixBase<M, N> res;
 		for (unsigned int i = 0; i < N; i++)
 			for (unsigned int j = 0; j < M; j++)
 				res[i][j] = data[i][j] + m(i, j);
		return res;
	}

	MatrixBase<M, N> &operator +=(const MatrixBase<M, N> &m) {
		return *this = *this + m;
	}

	/**
	 * subtraction
	 */
	MatrixBase<M, N> operator -(const MatrixBase<M, N> &m) const {
		MatrixBase<M, N> res;
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				res[i][j] = data[i][j] - m(i, j);
		return res;
	}

	MatrixBase<M, N> &operator -=(const MatrixBase<M, N> &m) {
		return *this = *this - m;
	}

	/**
	 * uniform scaling
	 */
	MatrixBase<M, N> operator *(const float num) const {
		MatrixBase<M, N> res;
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				res[i][j] = data[i][j] * num;
		return res;
	}

	MatrixBase<M, N> &operator *=(const float num) {
		return *this = *this * num;
	}

	MatrixBase<M, N> operator /(const float num) const {
		MatrixBase<M, N> res;
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				res[i][j] = data[i][j] / num;
		return res;
	}

	MatrixBase<M, N> &operator /=(const float num) {
		return *this = *this / num;
	}

	/**
	 * multiplication by another matrix
	 */
	template <unsigned int P>
	Matrix<N, P> operator *(const Matrix<M, P> &m) const {
		Matrix<N, P> res;
		arm_mat_mult_f32(&arm_mat, &m.arm_mat, &res.arm_mat);
		return res;
	}

	/**
	 * setup the identity matrix
	 */
	void identity(void) {
		memset(data, 0, sizeof(data));
		for (unsigned int i = 0; i < M; i++)
			data[i][i] = 1;
	}

	void dump(void) {
 		for (unsigned int i = 0; i < M; i++) {
 			for (unsigned int j = 0; j < N; j++)
				printf("%.3f\t", data[i][j]);
			printf("\n");
		}
	}
};

template <unsigned int M, unsigned int N>
class Matrix : public MatrixBase<M, N> {
public:
	/**
	 * set to value
	 */
	const Matrix<M, N> &operator =(const Matrix<M, N> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	/**
	 * multiplication by a vector
	 */
	/*
	Vector<N> operator *(const Vector<N> &v) const {
	    Vector<M> res;
	    arm_mat_mult_f32(&this->arm_mat, &v.arm_col, &res.arm_col);
	    return res;
	}
	*/
};

template <>
class Matrix<3, 3> : public MatrixBase<3, 3> {
public:
	/**
	 * set to value
	 */
	const Matrix<3, 3> &operator =(const Matrix<3, 3> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	/**
	 * multiplication by a vector
	 */
	/*
	Vector<3> operator *(const Vector<3> &v) const {
	    Vector<3> res;
	    res(0) = data[0][0] * v(0) + data[0][1] * v(1) + data[0][2] * v(2);
	    res(1) = data[1][0] * v(0) + data[1][1] * v(1) + data[1][2] * v(2);
	    res(2) = data[2][0] * v(0) + data[2][1] * v(1) + data[2][2] * v(2);
	    return res;
	}
	*/
};

}

#endif // MATRIX_HPP
