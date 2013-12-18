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

#ifndef MATRIX3_HPP
#define MATRIX3_HPP

#include "Vector3.hpp"
#include "../CMSIS/Include/arm_math.h"

namespace math
{

// 3x3 matrix with elements of type T
template <typename T>
class Matrix3 {
public:
	/**
	 * matrix data[row][col]
	 */
	T data[3][3];

	/**
	 * struct for using arm_math functions
	 */
	arm_matrix_instance_f32 arm_mat;

	/**
	 * trivial ctor
	 * note that this ctor will not initialize elements
	 */
	Matrix3<T>() {
		arm_mat = {3, 3, &data[0][0]};
	}

	/**
	 * setting ctor
	 */
	Matrix3<T>(const T d[3][3]) {
		arm_mat = {3, 3, &data[0][0]};
		memcpy(data, d, sizeof(data));
	}

	/**
	 * setting ctor
	 */
	Matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz) {
		arm_mat = {3, 3, &data[0][0]};
		data[0][0] = ax;
		data[0][1] = ay;
		data[0][2] = az;
		data[1][0] = bx;
		data[1][1] = by;
		data[1][2] = bz;
		data[2][0] = cx;
		data[2][1] = cy;
		data[2][2] = cz;
	}

	/**
	 * casting from a vector3f to a matrix is the tilde operator
	 */
	Matrix3<T>(const Vector3<T> &v) {
		arm_mat = {3, 3, &data[0][0]};
		data[0][0] = 0;
		data[0][1] = -v.z;
		data[0][2] = v.y;
		data[1][0] = v.z;
		data[1][1] = 0;
		data[1][2] = -v.x;
		data[2][0] = -v.y;
		data[2][1] = v.x;
		data[2][2] = 0;
	}

	/**
	 * access by index
	 */
	inline T &operator ()(unsigned int row, unsigned int col) {
		return data[row][col];
	}

	/**
	 * access to elements by index
	 */
	inline const T &operator ()(unsigned int row, unsigned int col) const {
		return data[row][col];
	}

	/**
	 * set to value
	 */
	const Matrix3<T> &operator =(const Matrix3<T> &m) {
		memcpy(data, m.data, sizeof(data));
		return *this;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Matrix3<T> &m) {
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				if (data[i][j] != m(i, j))
 					return false;
		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Matrix3<T> &m) {
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				if (data[i][j] != m(i, j))
 					return true;
		return false;
	}

	/**
	 * negation
	 */
	Matrix3<T> operator -(void) const {
		Matrix3<T> res;
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				res[i][j] = -data[i][j];
		return res;
	}

	/**
	 * addition
	 */
	Matrix3<T> operator +(const Matrix3<T> &m) const {
		Matrix3<T> res;
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				res[i][j] = data[i][j] + m(i, j);
		return res;
	}

	Matrix3<T> &operator +=(const Matrix3<T> &m) {
		return *this = *this + m;
	}

	/**
	 * subtraction
	 */
	Matrix3<T> operator -(const Matrix3<T> &m) const {
		Matrix3<T> res;
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				res[i][j] = data[i][j] - m(i, j);
		return res;
	}

	Matrix3<T> &operator -=(const Matrix3<T> &m) {
		return *this = *this - m;
	}

	/**
	 * uniform scaling
	 */
	Matrix3<T> operator *(const T num) const {
		Matrix3<T> res;
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				res[i][j] = data[i][j] * num;
		return res;
	}

	Matrix3<T> &operator *=(const T num) {
		return *this = *this * num;
	}

	Matrix3<T> operator /(const T num) const {
		Matrix3<T> res;
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				res[i][j] = data[i][j] / num;
		return res;
	}

	Matrix3<T> &operator /=(const T num) {
		return *this = *this / num;
	}

	/**
	 * multiplication by a vector
	 */
	Vector3<T> operator *(const Vector3<T> &v) const {
	    return Vector3<T>(
	    		data[0][0] * v.x + data[0][1] * v.y + data[0][2] * v.z,
	    		data[1][0] * v.x + data[1][1] * v.y + data[1][2] * v.z,
	    		data[2][0] * v.x + data[2][1] * v.y + data[2][2] * v.z);
	}

	/**
	 * multiplication of transpose by a vector
	 */
	Vector3<T> mul_transpose(const Vector3<T> &v) const {
	    return Vector3<T>(
	    		data[0][0] * v.x + data[1][0] * v.y + data[2][0] * v.z,
	    		data[0][1] * v.x + data[1][1] * v.y + data[2][1] * v.z,
	    		data[0][2] * v.x + data[1][2] * v.y + data[2][2] * v.z);
	}

	/**
	 * multiplication by another matrix
	 */
	Matrix3<T> operator *(const Matrix3<T> &m) const {
#if defined(CONFIG_ARCH_CORTEXM4) && defined(CONFIG_ARCH_FPU)
		Matrix3<T> res;
		arm_mat_mult_f32(&arm_mat, &m.arm_mat, &res.arm_mat);
		return res;
#else
	    return Matrix3<T>(data[0][0] * m(0, 0) + data[0][1] * m(1, 0) + data[0][2] * m(2, 0),
	                     data[0][0] * m(0, 1) + data[0][1] * m(1, 1) + data[0][2] * m(2, 1),
	                     data[0][0] * m(0, 2) + data[0][1] * m(1, 2) + data[0][2] * m(2, 2),
	                     data[1][0] * m(0, 0) + data[1][1] * m(1, 0) + data[1][2] * m(2, 0),
	                     data[1][0] * m(0, 1) + data[1][1] * m(1, 1) + data[1][2] * m(2, 1),
	                     data[1][0] * m(0, 2) + data[1][1] * m(1, 2) + data[1][2] * m(2, 2),
	                     data[2][0] * m(0, 0) + data[2][1] * m(1, 0) + data[2][2] * m(2, 0),
	                     data[2][0] * m(0, 1) + data[2][1] * m(1, 1) + data[2][2] * m(2, 1),
	                     data[2][0] * m(0, 2) + data[2][1] * m(1, 2) + data[2][2] * m(2, 2));
#endif
	}

	Matrix3<T> &operator *=(const Matrix3<T> &m) {
		return *this = *this * m;
	}

	/**
	 * transpose the matrix
	 */
	Matrix3<T> transposed(void) const {
#if defined(CONFIG_ARCH_CORTEXM4) && defined(CONFIG_ARCH_FPU) && T == float
        Matrix3<T> res;
        arm_mat_trans_f32(&arm_mat, &res.arm_mat);
        return res;
#else
		return Matrix3<T>(data[0][0], data[1][0], data[2][0],
						  data[0][1], data[1][1], data[2][1],
						  data[0][2], data[1][2], data[2][2]);
#endif
	}

	/**
	 * inverse the matrix
	 */
	Matrix3<T> inversed(void) const {
        Matrix3<T> res;
        arm_mat_inverse_f32(&arm_mat, &res.arm_mat);
        return res;
	}

	/**
	 * zero the matrix
	 */
	void zero(void) {
		memset(data, 0, sizeof(data));
	}

	/**
	 * setup the identity matrix
	 */
	void identity(void) {
		memset(data, 0, sizeof(data));
		data[0][0] = 1;
		data[1][1] = 1;
		data[2][2] = 1;
	}

	/**
	 * check if any elements are NAN
	 */
 	bool is_nan(void) {
 		for (int i = 0; i < 3; i++)
 			for (int j = 0; j < 3; j++)
 				if (isnan(data[i][j]))
 					return true;
 		return false;
	}

	/**
	 * create a rotation matrix from given euler angles
	 * based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
	 */
	void from_euler(T roll, T pitch, T yaw) {
		T cp = (T)cosf(pitch);
		T sp = (T)sinf(pitch);
		T sr = (T)sinf(roll);
		T cr = (T)cosf(roll);
		T sy = (T)sinf(yaw);
		T cy = (T)cosf(yaw);

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

    // create eulers from a rotation matrix
	//void to_euler(float *roll, float *pitch, float *yaw);

    // apply an additional rotation from a body frame gyro vector
    // to a rotation matrix.
	//void rotate(const Vector3<T> &g);
};

typedef Matrix3<float> Matrix3f;
}

#endif // MATRIX3_HPP
