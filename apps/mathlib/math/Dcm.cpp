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

#include "math/test/test.hpp"

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

Dcm::Dcm(const float *data) :
	Matrix(3, 3, data)
{
}

Dcm::Dcm(const Quaternion &q) :
	Matrix(3, 3)
{
	Dcm &dcm = *this;
	float a = q.getA();
	float b = q.getB();
	float c = q.getC();
	float d = q.getD();
	float aSq = a * a;
	float bSq = b * b;
	float cSq = c * c;
	float dSq = d * d;
	dcm(0, 0) = aSq + bSq - cSq - dSq;
	dcm(0, 1) = 2 * (b * c - a * d);
	dcm(0, 2) = 2 * (a * c + b * d);
	dcm(1, 0) = 2 * (b * c + a * d);
	dcm(1, 1) = aSq - bSq + cSq - dSq;
	dcm(1, 2) = 2 * (c * d - a * b);
	dcm(2, 0) = 2 * (b * d - a * c);
	dcm(2, 1) = 2 * (a * b + c * d);
	dcm(2, 2) = aSq - bSq - cSq + dSq;
}

Dcm::Dcm(const EulerAngles &euler) :
	Matrix(3, 3)
{
	Dcm &dcm = *this;
	float cosPhi = cosf(euler.getPhi());
	float sinPhi = sinf(euler.getPhi());
	float cosThe = cosf(euler.getTheta());
	float sinThe = sinf(euler.getTheta());
	float cosPsi = cosf(euler.getPsi());
	float sinPsi = sinf(euler.getPsi());

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
	Vector3 vB(1, 2, 3);
	ASSERT(matrixEqual(Dcm(Quaternion(1, 0, 0, 0)),
			   Matrix::identity(3)));
	ASSERT(matrixEqual(Dcm(EulerAngles(0, 0, 0)),
			   Matrix::identity(3)));
	ASSERT(vectorEqual(Vector3(-2, 1, 3),
			   Dcm(EulerAngles(0, 0, M_PI_2_F))*vB));
	ASSERT(vectorEqual(Vector3(3, 2, -1),
			   Dcm(EulerAngles(0, M_PI_2_F, 0))*vB));
	ASSERT(vectorEqual(Vector3(1, -3, 2),
			   Dcm(EulerAngles(M_PI_2_F, 0, 0))*vB));
	ASSERT(vectorEqual(Vector3(3, 2, -1),
			   Dcm(EulerAngles(
				       M_PI_2_F, M_PI_2_F, M_PI_2_F))*vB));
	printf("PASS\n");
	return 0;
}
} // namespace math
