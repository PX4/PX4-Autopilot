/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
#include <array>

#include <parameters/param.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include "mixer_module.hpp"

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#define PARAM_PREFIX "PWM_MAIN"
#else
#define PARAM_PREFIX "HIL_ACT"
#endif

static constexpr int max_num_outputs = 8;

static constexpr int disarmed_value = 900;
static constexpr int failsafe_value = 800;
static constexpr int min_value = 1000;
static constexpr int max_value = 2000;

class MixerModuleTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		int32_t v = 1;
		param_set(param_find("SYS_CTRL_ALLOC"), &v);
	}

	int update(MixingOutput &mixing_output)
	{
		mixing_output.update();
		// make sure output_limit switches to ON (if outputs enabled)
		px4_usleep(50000 * 2);
		mixing_output.update();
		mixing_output.update();
		return 3; // expected number of output updates
	}

};

class OutputModuleTest : public OutputModuleInterface
{
public:
	OutputModuleTest() : OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default) {};

	void Run() override
	{
		was_scheduled = true;
	}

	bool updateOutputs(bool stop_motors, uint16_t outputs_[MAX_ACTUATORS],
			   unsigned num_outputs_, unsigned num_control_groups_updated) override
	{
		memcpy(outputs, outputs_, sizeof(outputs));
		num_outputs = num_outputs_;
		++num_updates;
		return true;
	}

	void mixerChanged() override
	{
		mixer_changed = true;
	}

	void configureFunctions(const std::array<int32_t, max_num_outputs> &functions)
	{
		for (int i = 0; i < max_num_outputs; ++i) {
			char buffer[17];

			snprintf(buffer, sizeof(buffer), "%s_FUNC%u", PARAM_PREFIX, i + 1);
			param_set(param_find(buffer), &functions[i]);
		}

		updateParams();
	}

	void sendMotors(const std::array<float, actuator_motors_s::NUM_CONTROLS> &motors, uint16_t reversible = 0)
	{
		actuator_motors_s actuator_motors{};
		actuator_motors.timestamp = hrt_absolute_time();
		actuator_motors.reversible_flags = reversible;

		for (unsigned i = 0; i < motors.size(); ++i) {
			actuator_motors.control[i] = motors[i];
		}

		_actuator_motors_pub.publish(actuator_motors);
	}

	void sendServos(const std::array<float, actuator_servos_s::NUM_CONTROLS> &servos)
	{
		actuator_servos_s actuator_servos{};
		actuator_servos.timestamp = hrt_absolute_time();

		for (unsigned i = 0; i < servos.size(); ++i) {
			actuator_servos.control[i] = servos[i];
		}

		_actuator_servos_pub.publish(actuator_servos);
	}

	void sendActuatorMotorTest(int function, float value, bool release_control)
	{
		actuator_test_s actuator_test{};
		actuator_test.timestamp = hrt_absolute_time();
		actuator_test.function = function;
		actuator_test.value = value;
		actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
		actuator_test.timeout_ms = 0;
		_actuator_test_pub.publish(actuator_test);
	}

	void sendActuatorArmed(bool armed, bool force_failsafe = false, bool manual_lockdown = false, bool prearm = false)
	{
		actuator_armed_s actuator_armed{};
		actuator_armed.timestamp = hrt_absolute_time();
		actuator_armed.armed = armed;
		actuator_armed.force_failsafe = force_failsafe;
		actuator_armed.manual_lockdown = manual_lockdown;
		actuator_armed.prearmed = prearm;
		_actuator_armed_pub.publish(actuator_armed);
	}

	void reset()
	{
		memset(outputs, 0, sizeof(outputs));
		num_outputs = 0;
		num_updates = 0;
		mixer_changed = false;
	}

	uint16_t outputs[MAX_ACTUATORS] {};
	int num_outputs{0};
	int num_updates{0};
	bool was_scheduled{false};
	bool mixer_changed{false};

private:
	uORB::Publication<actuator_test_s> _actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_armed_s> _actuator_armed_pub{ORB_ID(actuator_armed)};
};

TEST_F(MixerModuleTest, basic)
{
	OutputModuleTest test_module;
	test_module.configureFunctions({});
	MixingOutput mixing_output{PARAM_PREFIX, max_num_outputs, test_module, MixingOutput::SchedulingPolicy::Disabled, false, false};
	mixing_output.setAllDisarmedValues(disarmed_value);
	mixing_output.setAllFailsafeValues(failsafe_value);
	mixing_output.setAllMinValues(min_value);
	mixing_output.setAllMaxValues(max_value);
	EXPECT_EQ(test_module.num_updates, 0);

	// all functions disabled: not expected to get an update
	mixing_output.update();
	mixing_output.updateSubscriptions(false);
	mixing_output.update();
	EXPECT_EQ(test_module.num_updates, 0);
	test_module.reset();

	// configure motor, ensure all still disarmed
	test_module.configureFunctions({(int)OutputFunction::Motor1});
	mixing_output.updateSubscriptions(false);
	EXPECT_TRUE(test_module.mixer_changed);
	EXPECT_EQ(test_module.num_updates, update(mixing_output));
	EXPECT_EQ(test_module.num_outputs, max_num_outputs);

	for (int i = 0; i < test_module.num_outputs; ++i) {
		EXPECT_EQ(test_module.outputs[i], disarmed_value);
	}

	test_module.reset();

	// send motors -> still disarmed
	test_module.sendMotors({1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f});
	test_module.configureFunctions({
		0,
		(int)OutputFunction::Motor3,
		(int)OutputFunction::Motor1,
		(int)OutputFunction::Motor5});

	mixing_output.updateSubscriptions(false);
	EXPECT_EQ(test_module.num_updates, update(mixing_output));
	EXPECT_EQ(test_module.num_outputs, max_num_outputs);

	for (int i = 0; i < test_module.num_outputs; ++i) {
		EXPECT_EQ(test_module.outputs[i], disarmed_value);
	}

	test_module.reset();


	// actuator test
	test_module.sendActuatorMotorTest((int)OutputFunction::Motor5, 1.f, false);
	test_module.sendMotors({1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f});
	mixing_output.updateSubscriptions(false);
	EXPECT_EQ(test_module.num_updates, update(mixing_output));
	EXPECT_EQ(test_module.num_outputs, max_num_outputs);

	for (int i = 0; i < test_module.num_outputs; ++i) {
		if (i == 3) {
			EXPECT_EQ(test_module.outputs[i], max_value);

		} else {
			EXPECT_EQ(test_module.outputs[i], disarmed_value);
		}
	}

	test_module.reset();

	// stop
	test_module.sendActuatorMotorTest((int)OutputFunction::Motor5, 0.f, true);
	test_module.sendMotors({1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f});
	mixing_output.updateSubscriptions(false);
	EXPECT_EQ(test_module.num_updates, update(mixing_output));
	EXPECT_EQ(test_module.num_outputs, max_num_outputs);

	for (int i = 0; i < test_module.num_outputs; ++i) {
		EXPECT_EQ(test_module.outputs[i], disarmed_value);
	}

	test_module.reset();

	EXPECT_FALSE(test_module.was_scheduled);
}

TEST_F(MixerModuleTest, arming)
{
	OutputModuleTest test_module;
	test_module.configureFunctions({
		0,
		(int)OutputFunction::Motor3,
		(int)OutputFunction::Motor1,
		(int)OutputFunction::Motor5,
		(int)OutputFunction::Servo3});
	MixingOutput mixing_output{PARAM_PREFIX, max_num_outputs, test_module, MixingOutput::SchedulingPolicy::Disabled, false, false};
	mixing_output.setAllDisarmedValues(disarmed_value);
	mixing_output.setAllFailsafeValues(failsafe_value);
	mixing_output.setAllMinValues(min_value);
	mixing_output.setAllMaxValues(max_value);

	test_module.sendMotors({1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f});
	test_module.sendActuatorArmed(false);

	// ensure all disarmed
	mixing_output.updateSubscriptions(false);
	EXPECT_EQ(test_module.num_updates, update(mixing_output));
	EXPECT_EQ(test_module.num_outputs, max_num_outputs);

	for (int i = 0; i < max_num_outputs; ++i) {
		EXPECT_EQ(test_module.outputs[i], disarmed_value);
	}

	test_module.reset();

	// arming
	test_module.sendMotors({0.5f, 1.f, 0.1f, 0.2f, 1.f, 1.f, 1.f, 1.f});
	test_module.sendActuatorArmed(true);

	EXPECT_EQ(test_module.num_updates, update(mixing_output));
	EXPECT_EQ(test_module.num_outputs, max_num_outputs);

	for (int i = 0; i < max_num_outputs; ++i) {
		if (i == 1) {
			EXPECT_EQ(test_module.outputs[i], (max_value - min_value) * 0.1f + min_value);

		} else if (i == 2) {
			EXPECT_EQ(test_module.outputs[i], (max_value - min_value) * 0.5f + min_value);

		} else if (i == 3) {
			EXPECT_EQ(test_module.outputs[i], max_value);

		} else {
			EXPECT_EQ(test_module.outputs[i], disarmed_value);
		}
	}

	test_module.reset();

	// update motors
	test_module.sendMotors({0.9f, 1.f, 0.24f, 0.2f, 0.f, 1.f, 1.f, 1.f});
	mixing_output.updateSubscriptions(false);
	mixing_output.update();

	for (int i = 0; i < max_num_outputs; ++i) {
		if (i == 1) {
			EXPECT_EQ(test_module.outputs[i], (max_value - min_value) * 0.24f + min_value);

		} else if (i == 2) {
			EXPECT_EQ(test_module.outputs[i], (max_value - min_value) * 0.9f + min_value);

		} else if (i == 3) {
			EXPECT_EQ(test_module.outputs[i], min_value);

		} else {
			EXPECT_EQ(test_module.outputs[i], disarmed_value);
		}
	}

	test_module.reset();

	// failsafe
	test_module.sendActuatorArmed(true, true);
	test_module.sendMotors({0.5f, 1.f, 0.1f, 0.2f, 1.f, 1.f, 1.f, 1.f});
	mixing_output.update();

	for (int i = 0; i < max_num_outputs; ++i) {
		EXPECT_EQ(test_module.outputs[i], failsafe_value);
	}

	test_module.reset();

	// restore
	test_module.sendActuatorArmed(true, false);
	test_module.sendMotors({1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f});
	mixing_output.update();

	for (int i = 0; i < max_num_outputs; ++i) {
		if (i >= 1 && i <= 3) {
			EXPECT_EQ(test_module.outputs[i], max_value);

		} else {
			EXPECT_EQ(test_module.outputs[i], disarmed_value);
		}
	}

	test_module.reset();

	// manual lockdown
	test_module.sendActuatorArmed(true, false, true);
	test_module.sendMotors({0.5f, 1.f, 0.1f, 0.2f, 1.f, 1.f, 1.f, 1.f});
	mixing_output.update();

	for (int i = 0; i < max_num_outputs; ++i) {
		EXPECT_EQ(test_module.outputs[i], disarmed_value);
	}

	test_module.reset();

	// restore
	test_module.sendActuatorArmed(true, false);
	test_module.sendMotors({0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f});
	mixing_output.update();

	for (int i = 0; i < max_num_outputs; ++i) {
		if (i >= 1 && i <= 3) {
			EXPECT_EQ(test_module.outputs[i], min_value);

		} else {
			EXPECT_EQ(test_module.outputs[i], disarmed_value);
		}
	}

	test_module.reset();

	// set motor 5 reversible: expect output to be in center when commanding to 0
	test_module.sendActuatorArmed(true, false);
	test_module.sendMotors({0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f}, 1u << 4);
	mixing_output.update();
	EXPECT_EQ(mixing_output.reversibleOutputs(), 1u << 3);

	for (int i = 0; i < max_num_outputs; ++i) {
		if (i == 1) {
			EXPECT_EQ(test_module.outputs[i], min_value);

		} else if (i == 2) {
			EXPECT_EQ(test_module.outputs[i], min_value);

		} else if (i == 3) {
			EXPECT_EQ(test_module.outputs[i], (max_value - min_value) * 0.5f + min_value);

		} else {
			EXPECT_EQ(test_module.outputs[i], disarmed_value);
		}
	}

	test_module.reset();

	// disarm
	test_module.sendActuatorArmed(false);
	test_module.sendMotors({0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f}, 1u << 4);
	mixing_output.update();

	for (int i = 0; i < max_num_outputs; ++i) {
		EXPECT_EQ(test_module.outputs[i], disarmed_value);
	}

	test_module.reset();

	EXPECT_FALSE(test_module.was_scheduled);
}

TEST_F(MixerModuleTest, prearm)
{
	OutputModuleTest test_module;
	test_module.configureFunctions({
		(int)OutputFunction::Motor1,
		(int)OutputFunction::Servo1});
	MixingOutput mixing_output{PARAM_PREFIX, max_num_outputs, test_module, MixingOutput::SchedulingPolicy::Disabled, false, false};
	mixing_output.setAllDisarmedValues(disarmed_value);
	mixing_output.setAllFailsafeValues(failsafe_value);
	mixing_output.setAllMinValues(min_value);
	mixing_output.setAllMaxValues(max_value);

	test_module.sendMotors({1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f});
	test_module.sendServos({1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f});
	test_module.sendActuatorArmed(false, false, false, true);

	// ensure all disarmed, except the servo
	mixing_output.updateSubscriptions(false);
	EXPECT_EQ(test_module.num_updates, update(mixing_output));
	EXPECT_EQ(test_module.num_outputs, max_num_outputs);

	for (int i = 0; i < max_num_outputs; ++i) {
		if (i == 1) {
			EXPECT_EQ(test_module.outputs[i], max_value);

		} else {
			EXPECT_EQ(test_module.outputs[i], disarmed_value);
		}
	}

	test_module.reset();

	EXPECT_FALSE(test_module.was_scheduled);
}
