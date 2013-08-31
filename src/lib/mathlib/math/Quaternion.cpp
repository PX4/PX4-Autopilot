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
 * @file Quaternion.cpp
 *
 * math vector
 */

#include "test/test.hpp"


#include "Quaternion.hpp"
#include "Dcm.hpp"
#include "EulerAngles.hpp"

namespace math
{

Quaternion::Quaternion() :
	Vector(4)
{
	setA(1.0f);
	setB(0.0f);
	setC(0.0f);
	setD(0.0f);
}

Quaternion::Quaternion(float a, float b,
		       float c, float d) :
	Vector(4)
{
	setA(a);
	setB(b);
	setC(c);
	setD(d);
}

Quaternion::Quaternion(const float *data) :
	Vector(4, data)
{
}

Quaternion::Quaternion(const Vector &v) :
	Vector(v)
{
}

Quaternion::Quaternion(const Dcm &dcm) :
	Vector(4)
{
	// avoiding singularities by not using
	// division equations
	setA(0.5 * sqrt(1.0 +
			double(dcm(0, 0) + dcm(1, 1) + dcm(2, 2))));
	setB(0.5 * sqrt(1.0 +
			double(dcm(0, 0) - dcm(1, 1) - dcm(2, 2))));
	setC(0.5 * sqrt(1.0 +
			double(-dcm(0, 0) + dcm(1, 1) - dcm(2, 2))));
	setD(0.5 * sqrt(1.0 +
			double(-dcm(0, 0) - dcm(1, 1) + dcm(2, 2))));
}

Quaternion::Quaternion(const EulerAngles &euler) :
	Vector(4)
{
	double cosPhi_2 = cos(double(euler.getPhi()) / 2.0);
	double sinPhi_2 = sin(double(euler.getPhi()) / 2.0);
	double cosTheta_2 = cos(double(euler.getTheta()) / 2.0);
	double sinTheta_2 = sin(double(euler.getTheta()) / 2.0);
	double cosPsi_2 = cos(double(euler.getPsi()) / 2.0);
	double sinPsi_2 = sin(double(euler.getPsi()) / 2.0);
	setA(cosPhi_2 * cosTheta_2 * cosPsi_2 +
	     sinPhi_2 * sinTheta_2 * sinPsi_2);
	setB(sinPhi_2 * cosTheta_2 * cosPsi_2 -
	     cosPhi_2 * sinTheta_2 * sinPsi_2);
	setC(cosPhi_2 * sinTheta_2 * cosPsi_2 +
	     sinPhi_2 * cosTheta_2 * sinPsi_2);
	setD(cosPhi_2 * cosTheta_2 * sinPsi_2 -
	     sinPhi_2 * sinTheta_2 * cosPsi_2);
}

Quaternion::Quaternion(const Quaternion &right) :
	Vector(right)
{
}

Quaternion::~Quaternion()
{
}

Vector Quaternion::derivative(const Vector &w)
{
#ifdef QUATERNION_ASSERT
	ASSERT(w.getRows() == 3);
#endif
	float dataQ[] = {
		getA(), -getB(), -getC(), -getD(),
		getB(),  getA(), -getD(),  getC(),
		getC(),  getD(),  getA(), -getB(),
		getD(), -getC(),  getB(),  getA()
	};
	Vector v(4);
	v(0) = 0.0f;
	v(1) = w(0);
	v(2) = w(1);
	v(3) = w(2);
	Matrix Q(4, 4, dataQ);
	return Q * v * 0.5f;
}

int __EXPORT quaternionTest()
{
	printf("Test Quaternion\t\t: ");
	// test default ctor
	Quaternion q;
	ASSERT(equal(q.getA(), 1.0f));
	ASSERT(equal(q.getB(), 0.0f));
	ASSERT(equal(q.getC(), 0.0f));
	ASSERT(equal(q.getD(), 0.0f));
	// test float ctor
	q = Quaternion(0.1825742f, 0.3651484f, 0.5477226f, 0.7302967f);
	ASSERT(equal(q.getA(), 0.1825742f));
	ASSERT(equal(q.getB(), 0.3651484f));
	ASSERT(equal(q.getC(), 0.5477226f));
	ASSERT(equal(q.getD(), 0.7302967f));
	// test euler ctor
	q = Quaternion(EulerAngles(0.1f, 0.2f, 0.3f));
	ASSERT(vectorEqual(q, Quaternion(0.983347f, 0.034271f, 0.106021f, 0.143572f)));
	// test dcm ctor
	q = Quaternion(Dcm());
	ASSERT(vectorEqual(q, Quaternion(1.0f, 0.0f, 0.0f, 0.0f)));
	// TODO test derivative
	// test accessors
	q.setA(0.1f);
	q.setB(0.2f);
	q.setC(0.3f);
	q.setD(0.4f);
	ASSERT(vectorEqual(q, Quaternion(0.1f, 0.2f, 0.3f, 0.4f)));
	printf("PASS\n");
	return 0;
}

} // namespace math
