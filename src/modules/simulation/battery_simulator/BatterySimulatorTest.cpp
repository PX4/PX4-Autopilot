/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#include <uORB/Publication.hpp>
#include "BatterySimulator.hpp"

// to run: make tests TESTFILTER=BatterySimulator

using namespace time_literals;

class TestBatterySimulator : public BatterySimulator
{
public:
	using BatterySimulator::current;
	using BatterySimulator::updateMotorLoad;
	using BatterySimulator::updateParams;
};

class BatterySimulatorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);
		param_reset_all();
	}

	void setParam(const char *name, const float value) { param_set(param_find(name), &value); }
	void setParam(const char *name, const int32_t value) { param_set(param_find(name), &value); }

	// Publish the given motor commands, unused motors are NaN like in the real allocation output
	void publishMotors(std::initializer_list<float> controls, const hrt_abstime timestamp)
	{
		actuator_motors_s actuator_motors{};

		for (float &control : actuator_motors.control) {
			control = NAN;
		}

		size_t i = 0;

		for (const float control : controls) {
			actuator_motors.control[i++] = control;
		}

		actuator_motors.timestamp = timestamp;
		_actuator_motors_pub.publish(actuator_motors);
		_simulator.updateMotorLoad();
	}

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	TestBatterySimulator _simulator;
};

TEST_F(BatterySimulatorTest, ThrottleCurrentFollowsMotorCommands)
{
	// GIVEN: the throttle model with 1 A idle and 4 A per motor at full throttle
	setParam("SIM_BAT_MODEL", (int32_t)1);
	setParam("SIM_BAT_I_IDLE", 1.f);
	setParam("SIM_BAT_I_MOT", 4.f);
	_simulator.updateParams();

	// WHEN: 4 motors run at 25% thrust
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({0.25f, 0.25f, 0.25f, 0.25f}, now);

	// THEN: each motor draws 4 A * 0.25^1.5 = 0.5 A on top of the idle current
	EXPECT_NEAR(_simulator.current(0, now), 1.f + 4.f * 0.5f, 1e-4f);

	// WHEN: one motor goes to full throttle and another one is reversed
	publishMotors({1.f, -0.25f, 0.25f, 0.25f}, now);

	// THEN: the full throttle motor draws the full motor current, the reversed one the same as forward
	EXPECT_NEAR(_simulator.current(0, now), 1.f + 4.f * (1.f + 3.f * 0.125f), 1e-4f);

	// WHEN: the motors are stopped
	publishMotors({0.f, 0.f, 0.f, 0.f}, now);

	// THEN: only the idle current is drawn
	EXPECT_NEAR(_simulator.current(0, now), 1.f, 1e-4f);
}

TEST_F(BatterySimulatorTest, DefaultsMatchAverageCurrentInHover)
{
	// GIVEN: the throttle model with default currents
	setParam("SIM_BAT_MODEL", (int32_t)1);
	_simulator.updateParams();

	// WHEN: a quadcopter hovers at 60% thrust
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({0.6f, 0.6f, 0.6f, 0.6f}, now);

	// THEN: the current is close to the default average multicopter current SIM_BAT_I_MC
	float i_mc = 0.f;
	param_get(param_find("SIM_BAT_I_MC"), &i_mc);
	EXPECT_NEAR(_simulator.current(0, now), i_mc, 0.2f);
}

TEST_F(BatterySimulatorTest, ThrottleFallsBackToAverageWithoutMotorCommands)
{
	// GIVEN: the throttle model and the average multicopter current
	setParam("SIM_BAT_MODEL", (int32_t)1);
	setParam("SIM_BAT_I_MC", 8.f);
	setParam("SIM_BAT_I_IDLE", 1.f);
	setParam("SIM_BAT_I_MOT", 4.f);
	_simulator.updateParams();

	// WHEN: no motor commands were received yet
	// THEN: the average current is used
	EXPECT_FLOAT_EQ(_simulator.current(0, hrt_absolute_time()), 8.f);

	// WHEN: the motors stop at a given time
	const hrt_abstime motors_time = hrt_absolute_time();
	publishMotors({0.f, 0.f, 0.f, 0.f}, motors_time);

	// THEN: the throttle model is used while the commands are recent
	EXPECT_FLOAT_EQ(_simulator.current(0, motors_time + 100_ms), 1.f);

	// AND: the average current is used again once they are stale
	EXPECT_FLOAT_EQ(_simulator.current(0, motors_time + 1_s), 8.f);
}

TEST_F(BatterySimulatorTest, AverageModelIgnoresMotorCommands)
{
	// GIVEN: the average model
	setParam("SIM_BAT_MODEL", (int32_t)0);
	setParam("SIM_BAT_I_MC", 8.f);
	_simulator.updateParams();

	// WHEN: the motors run at full throttle
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({1.f, 1.f, 1.f, 1.f}, now);

	// THEN: the average current is used
	EXPECT_FLOAT_EQ(_simulator.current(0, now), 8.f);
}

TEST_F(BatterySimulatorTest, PerBatteryThrottleOverrides)
{
	// GIVEN: the throttle model with shared currents, overridden for battery 2 only
	setParam("SIM_BAT_MODEL", (int32_t)1);
	setParam("SIM_BAT_I_IDLE", 1.f);
	setParam("SIM_BAT_I_MOT", 4.f);
	setParam("SIM_BAT2_I_IDLE", 0.f);
	setParam("SIM_BAT2_I_MOT", 2.f);
	_simulator.updateParams();

	// WHEN: one motor runs at full throttle
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({1.f}, now);

	// THEN: batteries 1 and 3 use the shared currents
	EXPECT_FLOAT_EQ(_simulator.current(0, now), 5.f);
	EXPECT_FLOAT_EQ(_simulator.current(2, now), 5.f);

	// AND: battery 2 uses its own, including a zero idle current
	EXPECT_FLOAT_EQ(_simulator.current(1, now), 2.f);
}
