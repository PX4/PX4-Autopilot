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
 * @file Quaternion.hpp
 *
 * Quaternion
 */

#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <math.h>
#include "../CMSIS/Include/arm_math.h"
#include "Vector.hpp"
#include "Matrix.hpp"

namespace math
{

template <unsigned int N, unsigned int M>
class Matrix;

class Quaternion : public Vector<4> {
public:
	/**
	 * trivial ctor
	 */
	Quaternion() {
	}

	/**
	 * setting ctor
	 */
	Quaternion(const float a0, const float b0, const float c0, const float d0): Vector(a0, b0, c0, d0) {
	}

	Quaternion(const Vector<4> &v) : Vector(v) {
	}

	Quaternion(const float *v) : Vector(v) {
	}

	/**
	 * access to elements by index
	 */
	/*
	inline float &operator ()(unsigned int i) {
		return *(&a + i);
	}
	*/

	/**
	 * access to elements by index
	 */
	/*
	inline const float &operator ()(unsigned int i) const {
		return *(&a + i);
	}
	*/

	/**
	 * addition
	 */
	/*
	const Quaternion operator +(const Quaternion &q) const {
		return Quaternion(a + q.a, b + q.b, c + q.c, d + q.d);
	}
	*/

	/**
	 * subtraction
	 */
	/*
	const Quaternion operator -(const Quaternion &q) const {
		return Quaternion(a - q.a, b - q.b, c - q.c, d - q.d);
	}
	*/

	Quaternion derivative(const Vector<3> &w) {
	        float dataQ[] = {
	                data[0], -data[1], -data[2], -data[3],
	                data[1],  data[0], -data[3],  data[2],
	                data[2],  data[3],  data[0], -data[1],
	                data[3], -data[2],  data[1],  data[0]
	        };
	        Matrix<4,4> Q(dataQ);
	        Vector<4> v(0.0f, w.data[0], w.data[1], w.data[2]);
	        return Q * v * 0.5f;
	}

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
};
}

#endif // QUATERNION_HPP
