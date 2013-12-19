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
 * @file Matrix.hpp
 *
 * Generic Matrix
 */

#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <stdio.h>
#include "../CMSIS/Include/arm_math.h"

namespace math
{

template <unsigned int M, unsigned int N>
class Matrix;

class Quaternion;

// MxN matrix with float elements
template <unsigned int M, unsigned int N>
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
		arm_mat = {M, N, &data[0][0]};
	}

	MatrixBase(const float *d) {
		arm_mat = {M, N, &data[0][0]};
		memcpy(data, d, sizeof(data));
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
	bool operator ==(const Matrix<M, N> &m) {
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				if (data[i][j] != m.data[i][j])
 					return false;
		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Matrix<M, N> &m) {
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				if (data[i][j] != m.data[i][j])
 					return true;
		return false;
	}

	/**
	 * set to value
	 */
	const Matrix<M, N> &operator =(const Matrix<M, N> &m) {
		memcpy(data, m.data, sizeof(data));
		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * negation
	 */
	Matrix<M, N> operator -(void) const {
		Matrix<M, N> res;
 		for (unsigned int i = 0; i < N; i++)
 			for (unsigned int j = 0; j < M; j++)
 				res.data[i][j] = -data[i][j];
		return res;
	}

	/**
	 * addition
	 */
	Matrix<M, N> operator +(const Matrix<M, N> &m) const {
		Matrix<M, N> res;
 		for (unsigned int i = 0; i < N; i++)
 			for (unsigned int j = 0; j < M; j++)
 				res.data[i][j] = data[i][j] + m.data[i][j];
		return res;
	}

	Matrix<M, N> &operator +=(const Matrix<M, N> &m) {
 		for (unsigned int i = 0; i < N; i++)
 			for (unsigned int j = 0; j < M; j++)
 				data[i][j] += m.data[i][j];
		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * subtraction
	 */
	Matrix<M, N> operator -(const Matrix<M, N> &m) const {
		Matrix<M, N> res;
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				res.data[i][j] = data[i][j] - m.data[i][j];
		return res;
	}

	Matrix<M, N> &operator -=(const Matrix<M, N> &m) {
 		for (unsigned int i = 0; i < N; i++)
 			for (unsigned int j = 0; j < M; j++)
 				data[i][j] -= m.data[i][j];
		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	Matrix<M, N> operator *(const float num) const {
		Matrix<M, N> res;
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				res.data[i][j] = data[i][j] * num;
		return res;
	}

	Matrix<M, N> &operator *=(const float num) {
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				data[i][j] *= num;
		return *static_cast<Matrix<M, N>*>(this);
	}

	Matrix<M, N> operator /(const float num) const {
		Matrix<M, N> res;
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				res[i][j] = data[i][j] / num;
		return res;
	}

	Matrix<M, N> &operator /=(const float num) {
 		for (unsigned int i = 0; i < M; i++)
 			for (unsigned int j = 0; j < N; j++)
 				data[i][j] /= num;
		return *this;
	}

	/**
	 * multiplication by another matrix
	 */
	template <unsigned int P>
	Matrix<M, P> operator *(const Matrix<N, P> &m) const {
		Matrix<M, P> res;
		arm_mat_mult_f32(&arm_mat, &m.arm_mat, &res.arm_mat);
		return res;
	}

	/**
	 * transpose the matrix
	 */
	Matrix<N, M> transposed(void) const {
        Matrix<N, M> res;
        arm_mat_trans_f32(&this->arm_mat, &res.arm_mat);
        return res;
	}

	/**
	 * invert the matrix
	 */
	Matrix<M, N> inversed(void) const {
        Matrix<M, N> res;
        arm_mat_inverse_f32(&this->arm_mat, &res.arm_mat);
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

	void print(void) {
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
	using MatrixBase<M, N>::operator *;

	Matrix() : MatrixBase<M, N>() {
	}

	Matrix(const float *d) : MatrixBase<M, N>(d) {
	}

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
	Vector<M> operator *(const Vector<N> &v) const {
	    Vector<M> res;
	    arm_mat_mult_f32(&this->arm_mat, &v.arm_col, &res.arm_col);
	    return res;
	}
};
}

#include "Quaternion.hpp"

namespace math {
template <>
class Matrix<3, 3> : public MatrixBase<3, 3> {
public:
	using MatrixBase<3, 3>::operator *;

	Matrix() : MatrixBase<3, 3>() {
	}

	Matrix(const float *d) : MatrixBase<3, 3>(d) {
	}

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
	Vector<3> operator *(const Vector<3> &v) const {
	    Vector<3> res(data[0][0] * v.data[0] + data[0][1] * v.data[1] + data[0][2] * v.data[2],
	    			  data[1][0] * v.data[0] + data[1][1] * v.data[1] + data[1][2] * v.data[2],
	    			  data[2][0] * v.data[0] + data[2][1] * v.data[1] + data[2][2] * v.data[2]);
	    return res;
	}

	/**
	 * create a rotation matrix from given quaternion
	 */
	void from_quaternion(const Quaternion &q) {
		float aSq = q.data[0] * q.data[0];
		float bSq = q.data[1] * q.data[1];
		float cSq = q.data[2] * q.data[2];
		float dSq = q.data[3] * q.data[3];
		data[0][0] = aSq + bSq - cSq - dSq;
		data[0][1] = 2.0f * (q.data[1] * q.data[2] - q.data[0] * q.data[3]);
		data[0][2] = 2.0f * (q.data[0] * q.data[2] + q.data[1] * q.data[3]);
		data[1][0] = 2.0f * (q.data[1] * q.data[2] + q.data[0] * q.data[3]);
		data[1][1] = aSq - bSq + cSq - dSq;
		data[1][2] = 2.0f * (q.data[2] * q.data[3] - q.data[0] * q.data[1]);
		data[2][0] = 2.0f * (q.data[1] * q.data[3] - q.data[0] * q.data[2]);
		data[2][1] = 2.0f * (q.data[0] * q.data[1] + q.data[2] * q.data[3]);
		data[2][2] = aSq - bSq - cSq + dSq;
	}

	/**
	 * create a rotation matrix from given euler angles
	 * based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
	 */
	void from_euler(float roll, float pitch, float yaw) {
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		data[0][0] = cp * cy;
		data[0][1] = (sr * sp * cy) - (cr * sy);
		data[0][2] = (cr * sp * cy) + (sr * sy);
		data[1][0] = cp * sy;
		data[1][1] = (sr * sp * sy) + (cr * cy);
		data[1][2] = (cr * sp * sy) - (sr * cy);
		data[2][0] = -sp;
		data[2][1] = sr * cp;
		data[2][2] = cr * cp;
	}

	Vector<3> to_euler(void) const {
		Vector<3> euler;
		euler.data[1] = asinf(-data[2][0]);

		if (fabsf(euler.data[1] - M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) + euler.data[0];

		} else if (fabsf(euler.data[1] + M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) - euler.data[0];

		} else {
			euler.data[0] = atan2f(data[2][1], data[2][2]);
			euler.data[2] = atan2f(data[1][0], data[0][0]);
		}
	}
};

}

#endif // MATRIX_HPP
