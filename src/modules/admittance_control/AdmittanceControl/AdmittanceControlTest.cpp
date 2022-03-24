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
#include <AdmittanceControl.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <px4_defines.h>

using namespace matrix;


class AdmittanceControlBasicTest : public ::testing::Test
{
public:
	AdmittanceControlBasicTest()
	{
		_admittance_control.initialize(_bell_params);
	}

	AdmittanceControl _admittance_control;
	vehicle_local_position_setpoint_s _input_setpoint{};
	vehicle_local_position_setpoint_s _output_setpoint{};
	vehicle_attitude_s _attitude{};



	BellParameters _bell_params{};
	float _dt;
	Vector<float, 4> _wrench;
	Vector<float, 8> _actuator_output{};
	float _dist;

};


TEST_F(AdmittanceControlBasicTest, BellTest)
{
	_input_setpoint.x = 0.f;
	_input_setpoint.y = 0.f;
	_input_setpoint.z = -10.f;

	Quatf q{cosf(-M_PI/8),0.f,0.f,sinf(-M_PI/8)};
	// Quatf q{cosf(0.f),0.f,0.f,sinf(0.f)};

	_bell_params.A[0] = 2.5f;
	_bell_params.A[1] = 2.5f;
	_bell_params.A[2] = 2.5f;
	_bell_params.A[3] = 2.5f;

	_bell_params.B1[0] = 5.f;
	_bell_params.B1[1] = 5.f;
	_bell_params.B1[2] = 5.f;
	_bell_params.B1[3] = 5.f;

	_bell_params.B2[0] = 3.5f;
	_bell_params.B2[1] = 3.5f;
	_bell_params.B2[2] = 3.5f;
	_bell_params.B2[3] = 3.5f;

	_bell_params.B3[0] = 2.f;
	_bell_params.B3[1] = 2.f;
	_bell_params.B3[2] = 2.f;
	_bell_params.B3[3] = 2.f;

	_bell_params.M_min[0] = .1f;
	_bell_params.M_min[1] = .1f;
	_bell_params.M_min[2] = .1f;
	_bell_params.M_min[3] = .1f;

	_bell_params.K_min[0] = 1.f;
	_bell_params.K_min[1] = 1.f;
	_bell_params.K_min[2] = 1.f;
	_bell_params.K_min[3] = 1.f;

	_bell_params.M_max[0] = 1.f;
	_bell_params.M_max[1] = 1.f;
	_bell_params.M_max[2] = 1.f;
	_bell_params.M_max[3] = 1.f;

	_bell_params.K_max[0] = 10.f;
	_bell_params.K_max[1] = 10.f;
	_bell_params.K_max[2] = 10.f;
	_bell_params.K_max[3] = 10.f;

	_bell_params.lpf_sat_factor = 1.f;

	_dt = (0.01f);
	_wrench(0) = (0.f);
	_wrench(1) = (1.f);
	_wrench(2) = (2.f);
	_wrench(3) = (5.f);
	_actuator_output.setAll(1500);

	_admittance_control.initialize(_bell_params);
	_admittance_control.update(_dt,_wrench,_actuator_output,0.f,q,_input_setpoint);

	AdmittanceParameters params = _admittance_control.getAdmittanceParameters();

	EXPECT_EQ(params.K[0], 10);
	EXPECT_GT(params.K[1], 9.4584);
	EXPECT_LE(params.K[1], 9.459);
	EXPECT_GT(params.K[2], 6.952);
	EXPECT_LE(params.K[2], 6.953);
	EXPECT_EQ(params.K[3], 2.0);

	EXPECT_EQ(params.M[0], 1);
	EXPECT_GT(params.M[1], .94584);
	EXPECT_LE(params.M[1], .9459);
	EXPECT_GT(params.M[2], .6952);
	EXPECT_LE(params.M[2], .6953);
	EXPECT_GT(params.M[3], .199);
	EXPECT_LE(params.M[3], .201);

	_input_setpoint.x = 0.f;
	_input_setpoint.y = 0.f;
	_input_setpoint.z = 0.f;

	_bell_params.M_min[0] = .5f;
	_bell_params.M_max[0] = 1.f;
	_bell_params.K_min[0] = 1.f;
	_bell_params.K_max[0] = 2.f;

	_wrench(0) = (0.5f);
	_wrench(1) = (1.f);

	_admittance_control.initialize(_bell_params);

	for (size_t i = 0; i < 500; i++)
	{
	_admittance_control.update(_dt,_wrench,_actuator_output,0.f,q,_input_setpoint);
	_output_setpoint = _admittance_control.getSetpoints();
	printf("x : %.6f\n",(double) _output_setpoint.x);
	// printf("y : %.6f\n",(double) _output_setpoint.y);
	}

	// _wrench(0) = -0.3f;
	// _wrench(1) = 0.f;
	// _wrench(2) = -30.f;
	// _wrench(3) = 0.01f;

	// 	printf("x : %.6f\n",(double) _wrench(0));
	// 	printf("y : %.6f\n",(double) _wrench(1));
	// 	printf("z : %.6f\n",(double) _wrench(2));
	// 	printf("yaw : %.6f\n",(double) _wrench(3));

	// //Saturate
	// _wrench(0) = math::constrain(_wrench(0), -2.5f, 2.5f);
	// _wrench(1) = math::constrain(_wrench(1), -2.5f, 2.5f);
	// _wrench(2) = math::constrain(_wrench(2), -15.f, 15.f);
	// _wrench(3) = math::constrain(_wrench(3), -0.5f, 0.5f);

	// 	printf("x constraint : %.6f\n",(double) _wrench(0));
	// 	printf("y constraint : %.6f\n",(double) _wrench(1));
	// 	printf("z constraint : %.6f\n",(double) _wrench(2));
	// 	printf("yaw constraint : %.6f\n",(double) _wrench(3));

	// 	//Deadzone
	// 	_wrench(0) = (abs(_wrench(0)) > 0.25f) ? (_wrench(0)) : (0.f);
	// 	_wrench(1) = (abs(_wrench(1)) > 0.2f) ? (_wrench(1)) : (0.f);
	// 	_wrench(2) = (abs(_wrench(2)) > 0.2f) ? (_wrench(2)) : (0.f);
	// 	_wrench(3) = (abs(_wrench(3)) > 0.035f) ? (_wrench(3)) : (0.f);


	// 	printf("x dz: %.6f\n",(double) _wrench(0));
	// 	printf("y dz: %.6f\n",(double) _wrench(1));
	// 	printf("z dz: %.6f\n",(double) _wrench(2));
	// 	printf("yaw dz: %.6f\n",(double) _wrench(3));

	EXPECT_EQ(0,cosf(-M_PI/4));

}
