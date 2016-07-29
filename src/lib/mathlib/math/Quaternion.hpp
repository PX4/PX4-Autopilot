/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
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
	 * division
	 */
	Quaternion operator /(const Quaternion &q) const {
		float norm = q.length_squared();
		return Quaternion(
				(  data[0] * q.data[0] + data[1] * q.data[1] + data[2] * q.data[2] + data[3] * q.data[3]) / norm,
			    (- data[0] * q.data[1] + data[1] * q.data[0] - data[2] * q.data[3] + data[3] * q.data[2]) / norm,
			    (- data[0] * q.data[2] + data[1] * q.data[3] + data[2] * q.data[0] - data[3] * q.data[1]) / norm,
			    (- data[0] * q.data[3] - data[1] * q.data[2] + data[2] * q.data[1] + data[3] * q.data[0]) / norm
		);
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
	 * conjugate
	 */
	Quaternion conjugated() const {
		return Quaternion(data[0], -data[1], -data[2], -data[3]);
	}

	/**
	 * inversed
	 */
	Quaternion inversed() const {
		float norm = length_squared();
		return Quaternion(data[0] / norm, -data[1] / norm, -data[2] / norm, -data[3] / norm);
	}

	/**
	 * conjugation
	 */
	Vector<3> conjugate(const Vector<3> &v) const {
		float q0q0 = data[0] * data[0];
		float q1q1 = data[1] * data[1];
		float q2q2 = data[2] * data[2];
		float q3q3 = data[3] * data[3];

		return Vector<3>(
				v.data[0] * (q0q0 + q1q1 - q2q2 - q3q3) +
				v.data[1] * 2.0f * (data[1] * data[2] - data[0] * data[3]) +
				v.data[2] * 2.0f * (data[0] * data[2] + data[1] * data[3]),

				v.data[0] * 2.0f * (data[1] * data[2] + data[0] * data[3]) +
				v.data[1] * (q0q0 - q1q1 + q2q2 - q3q3) +
				v.data[2] * 2.0f * (data[2] * data[3] - data[0] * data[1]),

				v.data[0] * 2.0f * (data[1] * data[3] - data[0] * data[2]) +
				v.data[1] * 2.0f * (data[0] * data[1] + data[2] * data[3]) +
				v.data[2] * (q0q0 - q1q1 - q2q2 + q3q3)
		);
	}

	/**
	 * conjugation with inversed quaternion
	 */
	Vector<3> conjugate_inversed(const Vector<3> &v) const {
		float q0q0 = data[0] * data[0];
		float q1q1 = data[1] * data[1];
		float q2q2 = data[2] * data[2];
		float q3q3 = data[3] * data[3];

		return Vector<3>(
				v.data[0] * (q0q0 + q1q1 - q2q2 - q3q3) +
				v.data[1] * 2.0f * (data[1] * data[2] + data[0] * data[3]) +
				v.data[2] * 2.0f * (data[1] * data[3] - data[0] * data[2]),

				v.data[0] * 2.0f * (data[1] * data[2] - data[0] * data[3]) +
				v.data[1] * (q0q0 - q1q1 + q2q2 - q3q3) +
				v.data[2] * 2.0f * (data[2] * data[3] + data[0] * data[1]),

				v.data[0] * 2.0f * (data[1] * data[3] + data[0] * data[2]) +
				v.data[1] * 2.0f * (data[2] * data[3] - data[0] * data[1]) +
				v.data[2] * (q0q0 - q1q1 - q2q2 + q3q3)
		);
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

		/* operations executed in double to avoid loss of precision through
		 * consecutive multiplications. Result stored as float.
		 */
		data[0] = static_cast<float>(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
		data[1] = static_cast<float>(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
		data[2] = static_cast<float>(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
		data[3] = static_cast<float>(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
	}

	/**
	 * simplified version of the above method to create quaternion representing rotation only by yaw
	 */
	void from_yaw(float yaw) {
		data[0] = cosf(yaw / 2.0f);
		data[1] = 0.0f;
		data[2] = 0.0f;
		data[3] = sinf(yaw / 2.0f);
	}

	/**
	 * create Euler angles vector from the quaternion
	 */
	Vector<3> to_euler() const {
		return Vector<3>(
			atan2f(2.0f * (data[0] * data[1] + data[2] * data[3]), 1.0f - 2.0f * (data[1] * data[1] + data[2] * data[2])),
			asinf(2.0f * (data[0] * data[2] - data[3] * data[1])),
			atan2f(2.0f * (data[0] * data[3] + data[1] * data[2]), 1.0f - 2.0f * (data[2] * data[2] + data[3] * data[3]))
		);
	}

	/**
	 * set quaternion to rotation by DCM
	 * Reference: Shoemake, Quaternions, http://www.cs.ucr.edu/~vbz/resources/quatut.pdf
	 */
	void from_dcm(const Matrix<3, 3> &dcm) {
		float tr = dcm.data[0][0] + dcm.data[1][1] + dcm.data[2][2];
		if (tr > 0.0f) {
			float s = sqrtf(tr + 1.0f);
			data[0] = s * 0.5f;
			s = 0.5f / s;
			data[1] = (dcm.data[2][1] - dcm.data[1][2]) * s;
			data[2] = (dcm.data[0][2] - dcm.data[2][0]) * s;
			data[3] = (dcm.data[1][0] - dcm.data[0][1]) * s;
		} else {
			/* Find maximum diagonal element in dcm
			* store index in dcm_i */
			int dcm_i = 0;
			for (int i = 1; i < 3; i++) {
				if (dcm.data[i][i] > dcm.data[dcm_i][dcm_i]) {
					dcm_i = i;
				}
			}
			int dcm_j = (dcm_i + 1) % 3;
			int dcm_k = (dcm_i + 2) % 3;
			float s = sqrtf((dcm.data[dcm_i][dcm_i] - dcm.data[dcm_j][dcm_j] -
			dcm.data[dcm_k][dcm_k]) + 1.0f);
			data[dcm_i + 1] = s * 0.5f;
			s = 0.5f / s;
			data[dcm_j + 1] = (dcm.data[dcm_i][dcm_j] + dcm.data[dcm_j][dcm_i]) * s;
			data[dcm_k + 1] = (dcm.data[dcm_k][dcm_i] + dcm.data[dcm_i][dcm_k]) * s;
			data[0] = (dcm.data[dcm_k][dcm_j] - dcm.data[dcm_j][dcm_k]) * s;
		}
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
