/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <RLSIdentification.hpp>

using namespace matrix;

class RLSIdentificationBasicTest : public ::testing::Test
{
public:
	RLSIdentificationBasicTest()
	{
		const float x_init[4] = {1.2f, 0.1f, 0.f, 0.f};
		const float x_confidence[4] = {0.001f, 0.01f, 1000.f, 1000.f};
		const float R_diag[5] = {1.f, 1.f, 10.f, 1.f, 1.f};
		const VehicleParameters params{1.04f, math::radians(31.f), 8, 0.1f, 1.041f, 0.25f, 0.035f, 0.020f};
		_rls_identification.initialize(x_init, x_confidence, R_diag, params);
	}

	RLSIdentification _rls_identification;

	float _dt;
	Vector<float, 8> _actuator_output{};
	Vector3f _acc{};
	Quatf _q;

};


TEST_F(RLSIdentificationBasicTest, Test1)
{
	_actuator_output.setAll(1500.f);
	_actuator_output(0) = 1400.f;
	_actuator_output(1) = 1400.f;
	_actuator_output(2) = 1540.f;
	_actuator_output(3) = 1500.f;
	_actuator_output(4) = 1540.f;
	_actuator_output(5) = 1540.f;
	_actuator_output(6) = 1400.f;
	_actuator_output(7) = 1355.f;
	_dt = (0.004f);
	_acc(0) = 0.4f;
	_acc(1) = 0.f;
	_acc(2) = -9.80665f;
	_q = Quatf(1.f,0.f,0.f,0.f);


	Vector2f x ;
	Vector3f fi ;
	for (size_t i = 0; i < 100000; i++)
	{
	_rls_identification.updateThrust(_acc*1.04f, _actuator_output*0.83333f, _dt, false);
	_rls_identification.updateOffset(_q, false);
	x = _rls_identification.getEstimationThrust();
	fi = _rls_identification.getActuatorForceVector();
	}

	EXPECT_EQ(x(0), x(1));
	EXPECT_EQ(fi(0), fi(2));
	EXPECT_EQ(fi(1), fi(2));

}
