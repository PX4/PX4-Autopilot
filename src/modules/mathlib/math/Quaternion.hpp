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
 * @file Quaternion.hpp
 *
 * math quaternion lib
 */

#pragma once

#include "Vector.hpp"
#include "Matrix.hpp"

namespace math
{

class Dcm;
class EulerAngles;

class __EXPORT Quaternion : public Vector
{
public:

	/**
	 * default ctor
	 */
	Quaternion();

	/**
	 * ctor from floats
	 */
	Quaternion(float a, float b, float c, float d);

	/**
	 * ctor from data
	 */
	Quaternion(const float *data);

	/**
	 * ctor from Vector
	 */
	Quaternion(const Vector &v);

	/**
	 * ctor from EulerAngles
	 */
	Quaternion(const EulerAngles &euler);

	/**
	 * ctor from Dcm
	 */
	Quaternion(const Dcm &dcm);

	/**
	 * deep copy ctor
	 */
	Quaternion(const Quaternion &right);

	/**
	 * dtor
	 */
	virtual ~Quaternion();

	/**
	 * derivative
	 */
	Vector derivative(const Vector &w);

	/**
	 * accessors
	 */
	void setA(float a) { (*this)(0) = a; }
	void setB(float b) { (*this)(1) = b; }
	void setC(float c) { (*this)(2) = c; }
	void setD(float d) { (*this)(3) = d; }
	const float &getA() const { return (*this)(0); }
	const float &getB() const { return (*this)(1); }
	const float &getC() const { return (*this)(2); }
	const float &getD() const { return (*this)(3); }
};

int __EXPORT quaternionTest();
} // math

