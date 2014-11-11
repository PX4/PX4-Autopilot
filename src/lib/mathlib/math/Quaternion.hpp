/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
 *           Pavel Kirienko <pavel.kirienko@gmail.com>
 *           Lorenz Meier <lm@inf.ethz.ch>
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
 * @file Quaternion.hpp
 *
 * Quaternion class
 */

#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <math.h>

#include "Vector.hpp"
#include "Matrix.hpp"

namespace math
{

class __EXPORT Quaternion : public Vector<4>
{
public:
	/**
	 * trivial ctor
	 */
	Quaternion() : Vector<4>() {}

	/**
	 * copy ctor
	 */
	Quaternion(const Quaternion &q) : Vector<4>(q) {}

	/**
	 * casting from vector
	 */
	Quaternion(const Vector<4> &v) : Vector<4>(v) {}

	/**
	 * setting ctor
	 */
	Quaternion(const float d[4]) : Vector<4>(d) {}

	/**
	 * setting ctor
	 */
	Quaternion(const float a0, const float b0, const float c0, const float d0): Vector<4>(a0, b0, c0, d0) {}

	using Vector<4>::operator *;

	/**
	 * multiplication
	 */
	const Quaternion operator *(const Quaternion &q) const {
		return Quaternion(
			       data[0] * q.data[0] - data[1] * q.data[1] - data[2] * q.data[2] - data[3] * q.data[3],
			       data[0] * q.data[1] + data[1] * q.data[0] + data[2] * q.data[3] - data[3] * q.data[2],
			       data[0] * q.data[2] - data[1] * q.data[3] + data[2] * q.data[0] + data[3] * q.data[1],
			       data[0] * q.data[3] + data[1] * q.data[2] - data[2] * q.data[1] + data[3] * q.data[0]);
	}

	/**
	 * derivative
	 */
	const Quaternion derivative(const Vector<3> &w) {
		float dataQ[] = {
			data[0], -data[1], -data[2], -data[3],
			data[1],  data[0], -data[3],  data[2],
			data[2],  data[3],  data[0], -data[1],
			data[3], -data[2],  data[1],  data[0]
		};
		Matrix<4, 4> Q(dataQ);
		Vector<4> v(0.0f, w.data[0], w.data[1], w.data[2]);
		return Q * v * 0.5f;
	}

	/**
	 * imaginary part of quaternion
	 */
	Vector<3> imag(void) {
		return Vector<3>(&data[1]);
	}

	/**
	 * set quaternion to rotation defined by euler angles
	 */
	void from_euler(float roll, float pitch, float yaw) {
		double cosPhi_2 = cos(double(roll) / 2.0);
		double sinPhi_2 = sin(double(roll) / 2.0);
		double cosTheta_2 = cos(double(pitch) / 2.0);
		double sinTheta_2 = sin(double(pitch) / 2.0);
		double cosPsi_2 = cos(double(yaw) / 2.0);
		double sinPsi_2 = sin(double(yaw) / 2.0);
		data[0] = cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2;
		data[1] = sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2;
		data[2] = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
		data[3] = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;
	}

	void from_dcm(const Matrix<3, 3> &m) {
		// avoiding singularities by not using division equations
		data[0] = 0.5f * sqrtf(1.0f + m.data[0][0] + m.data[1][1] + m.data[2][2]);
		data[1] = 0.5f * sqrtf(1.0f + m.data[0][0] - m.data[1][1] - m.data[2][2]);
		data[2] = 0.5f * sqrtf(1.0f - m.data[0][0] + m.data[1][1] - m.data[2][2]);
		data[3] = 0.5f * sqrtf(1.0f - m.data[0][0] - m.data[1][1] + m.data[2][2]);
	}

	/**
	 * create rotation matrix for the quaternion
	 */
	Matrix<3, 3> to_dcm(void) const {
		Matrix<3, 3> R;
		float aSq = data[0] * data[0];
		float bSq = data[1] * data[1];
		float cSq = data[2] * data[2];
		float dSq = data[3] * data[3];
		R.data[0][0] = aSq + bSq - cSq - dSq;
		R.data[0][1] = 2.0f * (data[1] * data[2] - data[0] * data[3]);
		R.data[0][2] = 2.0f * (data[0] * data[2] + data[1] * data[3]);
		R.data[1][0] = 2.0f * (data[1] * data[2] + data[0] * data[3]);
		R.data[1][1] = aSq - bSq + cSq - dSq;
		R.data[1][2] = 2.0f * (data[2] * data[3] - data[0] * data[1]);
		R.data[2][0] = 2.0f * (data[1] * data[3] - data[0] * data[2]);
		R.data[2][1] = 2.0f * (data[0] * data[1] + data[2] * data[3]);
		R.data[2][2] = aSq - bSq - cSq + dSq;
		return R;
	}
};

}

#endif // QUATERNION_HPP
