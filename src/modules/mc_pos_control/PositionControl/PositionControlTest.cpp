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
#include <PositionControl.hpp>
#include <px4_defines.h>

using namespace matrix;

TEST(PositionControlTest, EmptySetpoint)
{
	PositionControl position_control;

	vehicle_local_position_setpoint_s output_setpoint{};
	position_control.getLocalPositionSetpoint(output_setpoint);
	EXPECT_EQ(output_setpoint.x, 0.f);
	EXPECT_EQ(output_setpoint.y, 0.f);
	EXPECT_EQ(output_setpoint.z, 0.f);
	EXPECT_EQ(output_setpoint.yaw, 0.f);
	EXPECT_EQ(output_setpoint.yawspeed, 0.f);
	EXPECT_EQ(output_setpoint.vx, 0.f);
	EXPECT_EQ(output_setpoint.vy, 0.f);
	EXPECT_EQ(output_setpoint.vz, 0.f);
	EXPECT_EQ(Vector3f(output_setpoint.acceleration), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(output_setpoint.jerk), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(output_setpoint.thrust), Vector3f(0, 0, 0));

	vehicle_attitude_setpoint_s attitude{};
	position_control.getAttitudeSetpoint(attitude);
	EXPECT_EQ(attitude.roll_body, 0.f);
	EXPECT_EQ(attitude.pitch_body, 0.f);
	EXPECT_EQ(attitude.yaw_body, 0.f);
	EXPECT_EQ(attitude.yaw_sp_move_rate, 0.f);
	EXPECT_EQ(Quatf(attitude.q_d), Quatf(1.f, 0.f, 0.f, 0.f));
	//EXPECT_EQ(attitude.q_d_valid, false); // TODO should not be true when there was no control
	EXPECT_EQ(Vector3f(attitude.thrust_body), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(attitude.roll_reset_integral, false);
	EXPECT_EQ(attitude.pitch_reset_integral, false);
	EXPECT_EQ(attitude.yaw_reset_integral, false);
	EXPECT_EQ(attitude.fw_control_yaw, false);
	EXPECT_EQ(attitude.apply_flaps, 0.f);//vehicle_attitude_setpoint_s::FLAPS_OFF); // TODO why no reference?
}

class PositionControlBasicTest : public ::testing::Test
{
public:
	PositionControlBasicTest()
	{
		_position_control.setPositionGains(Vector3f(1.f, 1.f, 1.f));
		_position_control.setVelocityGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f));
		_position_control.setVelocityLimits(1.f, 1.f, 1.f);
		_position_control.setThrustLimits(0.1f, 0.9f);
		_position_control.setTiltLimit(1.f);
		_position_control.setHoverThrust(.5f);

		_contraints.tilt = 1.f;
		_contraints.speed_xy = NAN;
		_contraints.speed_up = NAN;
		_contraints.speed_down = NAN;

		_input_setpoint.x = NAN;
		_input_setpoint.y = NAN;
		_input_setpoint.z = NAN;
		_input_setpoint.yaw = NAN;
		_input_setpoint.yawspeed = NAN;
		_input_setpoint.vx = NAN;
		_input_setpoint.vy = NAN;
		_input_setpoint.vz = NAN;
		Vector3f(NAN, NAN, NAN).copyTo(_input_setpoint.acceleration);
		Vector3f(NAN, NAN, NAN).copyTo(_input_setpoint.thrust);
	}

	void runController()
	{
		_position_control.setConstraints(_contraints);
		_position_control.setInputSetpoint(_input_setpoint);
		_position_control.update(.1f);
		_position_control.getLocalPositionSetpoint(_output_setpoint);
		_position_control.getAttitudeSetpoint(_attitude);
	}

	PositionControl _position_control;
	vehicle_constraints_s _contraints{};
	vehicle_local_position_setpoint_s _input_setpoint{};
	vehicle_local_position_setpoint_s _output_setpoint{};
	vehicle_attitude_setpoint_s _attitude{};
};

class PositionControlBasicDirectionTest : public PositionControlBasicTest
{
public:
	void checkDirection()
	{
		Vector3f thrust(_output_setpoint.thrust);
		EXPECT_GT(thrust(0), 0.f);
		EXPECT_GT(thrust(1), 0.f);
		EXPECT_LT(thrust(2), 0.f);

		Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
		EXPECT_LT(body_z(0), 0.f);
		EXPECT_LT(body_z(1), 0.f);
		EXPECT_GT(body_z(2), 0.f);
	}
};

TEST_F(PositionControlBasicDirectionTest, PositionDirection)
{
	_input_setpoint.x = .1f;
	_input_setpoint.y = .1f;
	_input_setpoint.z = -.1f;
	runController();
	checkDirection();
}

TEST_F(PositionControlBasicDirectionTest, VelocityDirection)
{
	_input_setpoint.vx = .1f;
	_input_setpoint.vy = .1f;
	_input_setpoint.vz = -.1f;
	runController();
	checkDirection();
}

TEST_F(PositionControlBasicTest, TiltLimit)
{
	_input_setpoint.x = 10.f;
	_input_setpoint.y = 10.f;
	_input_setpoint.z = -0.f;

	runController();
	Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
	float angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, 1.f);

	_contraints.tilt = .5f;
	runController();
	body_z = Quatf(_attitude.q_d).dcm_z();
	angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, .50001f);
}

TEST_F(PositionControlBasicTest, VelocityLimit)
{
	_input_setpoint.x = 10.f;
	_input_setpoint.y = 10.f;
	_input_setpoint.z = -10.f;

	runController();
	Vector2f velocity_xy(_output_setpoint.vx, _output_setpoint.vy);
	EXPECT_LE(velocity_xy.norm(), 1.f);
	EXPECT_LE(abs(_output_setpoint.vz), 1.f);
}

TEST_F(PositionControlBasicTest, ThrustLimit)
{
	_input_setpoint.x = 10.f;
	_input_setpoint.y = 10.f;
	_input_setpoint.z = -10.f;

	runController();
	EXPECT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_LT(_attitude.thrust_body[2], -.1f);
	EXPECT_GE(_attitude.thrust_body[2], -0.9f);
}

TEST_F(PositionControlBasicTest, FailsafeInput)
{
	_input_setpoint.vz = .7f;
	_input_setpoint.acceleration[0] = _input_setpoint.acceleration[1] = 0.f;

	runController();
	EXPECT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_LT(_output_setpoint.thrust[2], -.1f);
	EXPECT_GT(_output_setpoint.thrust[2], -.5f);
	EXPECT_GT(_attitude.thrust_body[2], -.5f);
	EXPECT_LE(_attitude.thrust_body[2], -.1f);
}

TEST_F(PositionControlBasicTest, InputCombinationsPosition)
{
	_input_setpoint.x = .1f;
	_input_setpoint.y = .2f;
	_input_setpoint.z = .3f;

	runController();
	EXPECT_EQ(_output_setpoint.x, .1f);
	EXPECT_EQ(_output_setpoint.y, .2f);
	EXPECT_EQ(_output_setpoint.z, .3f);
	EXPECT_FALSE(isnan(_output_setpoint.vx));
	EXPECT_FALSE(isnan(_output_setpoint.vy));
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}

TEST_F(PositionControlBasicTest, InputCombinationsPositionVelocity)
{
	_input_setpoint.vx = .1f;
	_input_setpoint.vy = .2f;
	_input_setpoint.z = .3f; // altitude

	runController();
	// EXPECT_TRUE(isnan(_output_setpoint.x));
	// EXPECT_TRUE(isnan(_output_setpoint.y));
	EXPECT_EQ(_output_setpoint.z, .3f);
	EXPECT_EQ(_output_setpoint.vx, .1f);
	EXPECT_EQ(_output_setpoint.vy, .2f);
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}
