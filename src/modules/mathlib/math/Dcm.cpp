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
 * @file Dcm.cpp
 *
 * math direction cosine matrix
 */

#include <mathlib/math/test/test.hpp>

#include "Dcm.hpp"
#include "Quaternion.hpp"
#include "EulerAngles.hpp"
#include "Vector3.hpp"

namespace math
{

Dcm::Dcm() :
	Matrix(Matrix::identity(3))
{
}

Dcm::Dcm(float c00, float c01, float c02,
	 float c10, float c11, float c12,
	 float c20, float c21, float c22) :
	Matrix(3, 3)
{
	Dcm &dcm = *this;
	dcm(0, 0) = c00;
	dcm(0, 1) = c01;
	dcm(0, 2) = c02;
	dcm(1, 0) = c10;
	dcm(1, 1) = c11;
	dcm(1, 2) = c12;
	dcm(2, 0) = c20;
	dcm(2, 1) = c21;
	dcm(2, 2) = c22;
}

Dcm::Dcm(const float data[3][3]) :
	Matrix(3, 3)
{
	Dcm &dcm = *this;
	/* set rotation matrix */
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
		dcm(i, j) = data[i][j];
}

Dcm::Dcm(const float *data) :
	Matrix(3, 3, data)
{
}

Dcm::Dcm(const Quaternion &q) :
	Matrix(3, 3)
{
	Dcm &dcm = *this;
	double a = q.getA();
	double b = q.getB();
	double c = q.getC();
	double d = q.getD();
	double aSq = a * a;
	double bSq = b * b;
	double cSq = c * c;
	double dSq = d * d;
	dcm(0, 0) = aSq + bSq - cSq - dSq;
	dcm(0, 1) = 2.0 * (b * c - a * d);
	dcm(0, 2) = 2.0 * (a * c + b * d);
	dcm(1, 0) = 2.0 * (b * c + a * d);
	dcm(1, 1) = aSq - bSq + cSq - dSq;
	dcm(1, 2) = 2.0 * (c * d - a * b);
	dcm(2, 0) = 2.0 * (b * d - a * c);
	dcm(2, 1) = 2.0 * (a * b + c * d);
	dcm(2, 2) = aSq - bSq - cSq + dSq;
}

Dcm::Dcm(const EulerAngles &euler) :
	Matrix(3, 3)
{
	Dcm &dcm = *this;
	double cosPhi = cos(euler.getPhi());
	double sinPhi = sin(euler.getPhi());
	double cosThe = cos(euler.getTheta());
	double sinThe = sin(euler.getTheta());
	double cosPsi = cos(euler.getPsi());
	double sinPsi = sin(euler.getPsi());

	dcm(0, 0) = cosThe * cosPsi;
	dcm(0, 1) = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
	dcm(0, 2) = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

	dcm(1, 0) = cosThe * sinPsi;
	dcm(1, 1) = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
	dcm(1, 2) = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

	dcm(2, 0) = -sinThe;
	dcm(2, 1) = sinPhi * cosThe;
	dcm(2, 2) = cosPhi * cosThe;
}

Dcm::Dcm(const Dcm &right) :
	Matrix(right)
{
}

Dcm::~Dcm()
{
}

int __EXPORT dcmTest()
{
	printf("Test DCM\t\t: ");
	// default ctor
	ASSERT(matrixEqual(Dcm(),
			   Matrix::identity(3)));
	// quaternion ctor
	ASSERT(matrixEqual(
		       Dcm(Quaternion(0.983347f, 0.034271f, 0.106021f, 0.143572f)),
		       Dcm(0.9362934f, -0.2750958f,  0.2183507f,
			   0.2896295f,  0.9564251f, -0.0369570f,
			   -0.1986693f,  0.0978434f,  0.9751703f)));
	// euler angle ctor
	ASSERT(matrixEqual(
		       Dcm(EulerAngles(0.1f, 0.2f, 0.3f)),
		       Dcm(0.9362934f, -0.2750958f,  0.2183507f,
			   0.2896295f,  0.9564251f, -0.0369570f,
			   -0.1986693f,  0.0978434f,  0.9751703f)));
	// rotations
	Vector3 vB(1, 2, 3);
	ASSERT(vectorEqual(Vector3(-2.0f, 1.0f, 3.0f),
			   Dcm(EulerAngles(0.0f, 0.0f, M_PI_2_F))*vB));
	ASSERT(vectorEqual(Vector3(3.0f, 2.0f, -1.0f),
			   Dcm(EulerAngles(0.0f, M_PI_2_F, 0.0f))*vB));
	ASSERT(vectorEqual(Vector3(1.0f, -3.0f, 2.0f),
			   Dcm(EulerAngles(M_PI_2_F, 0.0f, 0.0f))*vB));
	ASSERT(vectorEqual(Vector3(3.0f, 2.0f, -1.0f),
			   Dcm(EulerAngles(
				       M_PI_2_F, M_PI_2_F, M_PI_2_F))*vB));
	printf("PASS\n");
	return 0;
}
} // namespace math
