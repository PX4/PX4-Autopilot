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
 * @file Vector.cpp
 *
 * math vector
 */

#include "test/test.hpp"

#include "EulerAngles.hpp"
#include "Quaternion.hpp"
#include "Dcm.hpp"
#include "Vector3.hpp"

namespace math
{

EulerAngles::EulerAngles() :
	Vector(3)
{
	setPhi(0.0f);
	setTheta(0.0f);
	setPsi(0.0f);
}

EulerAngles::EulerAngles(float phi, float theta, float psi) :
	Vector(3)
{
	setPhi(phi);
	setTheta(theta);
	setPsi(psi);
}

EulerAngles::EulerAngles(const Quaternion &q) :
	Vector(3)
{
	(*this) = EulerAngles(Dcm(q));
}

EulerAngles::EulerAngles(const Dcm &dcm) :
	Vector(3)
{
	setTheta(asinf(-dcm(2, 0)));

	if (fabsf(getTheta() - M_PI_2_F) < 1.0e-3f) {
		setPhi(0.0f);
		setPsi(atan2f(dcm(1, 2) - dcm(0, 1),
			      dcm(0, 2) + dcm(1, 1)) + getPhi());

	} else if (fabsf(getTheta() + M_PI_2_F) < 1.0e-3f) {
		setPhi(0.0f);
		setPsi(atan2f(dcm(1, 2) - dcm(0, 1),
			      dcm(0, 2) + dcm(1, 1)) - getPhi());

	} else {
		setPhi(atan2f(dcm(2, 1), dcm(2, 2)));
		setPsi(atan2f(dcm(1, 0), dcm(0, 0)));
	}
}

EulerAngles::~EulerAngles()
{
}

int __EXPORT eulerAnglesTest()
{
	printf("Test EulerAngles\t: ");
	EulerAngles euler(0.1f, 0.2f, 0.3f);

	// test ctor
	ASSERT(vectorEqual(Vector3(0.1f, 0.2f, 0.3f), euler));
	ASSERT(equal(euler.getPhi(), 0.1f));
	ASSERT(equal(euler.getTheta(), 0.2f));
	ASSERT(equal(euler.getPsi(), 0.3f));

	// test dcm ctor
	euler = Dcm(EulerAngles(0.1f, 0.2f, 0.3f));
	ASSERT(vectorEqual(Vector3(0.1f, 0.2f, 0.3f), euler));

	// test quat ctor
	euler = Quaternion(EulerAngles(0.1f, 0.2f, 0.3f));
	ASSERT(vectorEqual(Vector3(0.1f, 0.2f, 0.3f), euler));

	// test assignment
	euler.setPhi(0.4f);
	euler.setTheta(0.5f);
	euler.setPsi(0.6f);
	ASSERT(vectorEqual(Vector3(0.4f, 0.5f, 0.6f), euler));

	printf("PASS\n");
	return 0;
}

} // namespace math
