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

TEST_F(BatterySimulatorTest, CurrentFollowsMotorCommands)
{
	// GIVEN: 1 A idle, a = 0.5 A, b = 0.02 A/V², and a 10 V battery
	setParam("SIM_BAT_I_IDLE", 1.f);
	setParam("SIM_BAT_I_MOT_A", 0.5f);
	setParam("SIM_BAT_I_MOT_B", 0.02f);
	_simulator.updateParams();
	const float voltage = 10.f;

	// WHEN: 4 motors run at 50% thrust
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({0.5f, 0.5f, 0.5f, 0.5f}, now);

	// THEN: I = 1 + 0.5 * (4 * 0.5) + 0.02 * (4 * 0.125) * 10²
	EXPECT_NEAR(_simulator.current(0, now, voltage), 1.f + 1.f + 1.f, 1e-4f);

	// WHEN: one motor goes to full throttle and another one is reversed
	publishMotors({1.f, -0.5f, 0.5f, 0.5f}, now);

	// THEN: the reversed motor draws the same as forward
	EXPECT_NEAR(_simulator.current(0, now, voltage), 1.f + 0.5f * 2.5f + 0.02f * 1.375f * 100.f, 1e-4f);

	// WHEN: the motors are stopped
	publishMotors({0.f, 0.f, 0.f, 0.f}, now);

	// THEN: only the idle current is drawn
	EXPECT_NEAR(_simulator.current(0, now, voltage), 1.f, 1e-4f);
}

TEST_F(BatterySimulatorTest, CubicTermScalesWithVoltageSquared)
{
	// GIVEN: only the cubic term
	setParam("SIM_BAT_I_IDLE", 0.f);
	setParam("SIM_BAT_I_MOT_A", 0.f);
	setParam("SIM_BAT_I_MOT_B", 0.02f);
	_simulator.updateParams();

	// WHEN: a motor runs at full throttle
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({1.f}, now);

	// THEN: the current is b·V²
	EXPECT_NEAR(_simulator.current(0, now, 10.f), 2.f, 1e-4f);

	// AND: halving the voltage quarters it
	EXPECT_NEAR(_simulator.current(0, now, 5.f), 0.5f, 1e-4f);

	// AND: without voltage there is no cubic current
	EXPECT_FLOAT_EQ(_simulator.current(0, now, 0.f), 0.f);
}

TEST_F(BatterySimulatorTest, DefaultsDrawRealisticHoverCurrent)
{
	// GIVEN: the default currents and a fully charged 4S battery
	_simulator.updateParams();
	const float voltage = 4.f * 4.2f;

	// WHEN: a quadcopter hovers at 60% thrust
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({0.6f, 0.6f, 0.6f, 0.6f}, now);

	// THEN: the current is about 8 A, i.e. a 5000 mAh battery lasts close to 40 min
	EXPECT_NEAR(_simulator.current(0, now, voltage), 8.f, 0.3f);
}

TEST_F(BatterySimulatorTest, IdleCurrentWithoutMotorCommands)
{
	// GIVEN: 1 A idle, a = 1 A, b = 0.04 A/V², and a 10 V battery
	setParam("SIM_BAT_I_IDLE", 1.f);
	setParam("SIM_BAT_I_MOT_A", 1.f);
	setParam("SIM_BAT_I_MOT_B", 0.04f);
	_simulator.updateParams();
	const float voltage = 10.f;

	// WHEN: no motor commands were received yet
	// THEN: only the idle current is drawn
	EXPECT_FLOAT_EQ(_simulator.current(0, hrt_absolute_time(), voltage), 1.f);

	// WHEN: the motors run at full throttle at a given time
	const hrt_abstime motors_time = hrt_absolute_time();
	publishMotors({1.f, 1.f, 1.f, 1.f}, motors_time);

	// THEN: the motor current is drawn while the commands are recent: 1 + 4 * 1 + 4 * 0.04 * 10²
	EXPECT_FLOAT_EQ(_simulator.current(0, motors_time + 100_ms, voltage), 21.f);

	// AND: only the idle current once they are stale
	EXPECT_FLOAT_EQ(_simulator.current(0, motors_time + 1_s, voltage), 1.f);
}

TEST_F(BatterySimulatorTest, PerBatteryOverrides)
{
	// GIVEN: shared coefficients, overridden for battery 2 only
	setParam("SIM_BAT_I_IDLE", 1.f);
	setParam("SIM_BAT_I_MOT_A", 1.f);
	setParam("SIM_BAT_I_MOT_B", 0.03f);
	setParam("SIM_BAT2_I_IDLE", 0.f);
	setParam("SIM_BAT2_I_MOT_A", 0.f);
	setParam("SIM_BAT2_I_MOT_B", 0.02f);
	_simulator.updateParams();
	const float voltage = 10.f;

	// WHEN: one motor runs at full throttle
	const hrt_abstime now = hrt_absolute_time();
	publishMotors({1.f}, now);

	// THEN: batteries 1 and 3 use the shared coefficients
	EXPECT_FLOAT_EQ(_simulator.current(0, now, voltage), 5.f);
	EXPECT_FLOAT_EQ(_simulator.current(2, now, voltage), 5.f);

	// AND: battery 2 uses its own, including zero idle and linear currents
	EXPECT_FLOAT_EQ(_simulator.current(1, now, voltage), 2.f);
}
