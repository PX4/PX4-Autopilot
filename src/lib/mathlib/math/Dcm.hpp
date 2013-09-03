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
 * @file Dcm.hpp
 *
 * math direction cosine matrix
 */

//#pragma once

#include "Vector.hpp"
#include "Matrix.hpp"

namespace math
{

class Quaternion;
class EulerAngles;

/**
 * This is a Tait Bryan, Body 3-2-1 sequence.
 * (yaw)-(pitch)-(roll)
 * The Dcm transforms a vector in the body frame
 * to the navigation frame, typically represented
 * as C_nb. C_bn can be obtained through use
 * of the transpose() method.
 */
class __EXPORT Dcm : public Matrix
{
public:
	/**
	 * default ctor
	 */
	Dcm();

	/**
	 * scalar ctor
	 */
	Dcm(float c00, float c01, float c02,
	    float c10, float c11, float c12,
	    float c20, float c21, float c22);

	/**
	 * data ctor
	 */
	Dcm(const float *data);

	/**
	 * array ctor
	 */
	Dcm(const float data[3][3]);

	/**
	 * quaternion ctor
	 */
	Dcm(const Quaternion &q);

	/**
	 * euler angles ctor
	 */
	Dcm(const EulerAngles &euler);

	/**
	 * copy ctor (deep)
	 */
	Dcm(const Dcm &right);

	/**
	 * dtor
	 */
	virtual ~Dcm();
};

int __EXPORT dcmTest();

} // math

