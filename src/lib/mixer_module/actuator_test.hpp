/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include "functions.hpp"

#include <drivers/drv_pwm_output.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/Subscription.hpp>

static_assert(actuator_test_s::FUNCTION_MOTOR1 == (int)OutputFunction::Motor1, "define mismatch");
static_assert(actuator_test_s::MAX_NUM_MOTORS == (int)OutputFunction::MotorMax - (int)OutputFunction::Motor1 + 1,
	      "count mismatch");
static_assert(actuator_test_s::FUNCTION_SERVO1 == (int)OutputFunction::Servo1, "define mismatch");
static_assert(actuator_test_s::MAX_NUM_SERVOS == (int)OutputFunction::ServoMax - (int)OutputFunction::Servo1 + 1,
	      "count mismatch");

class ActuatorTest
{
public:
	static constexpr int MAX_ACTUATORS = PWM_OUTPUT_MAX_CHANNELS;

	ActuatorTest(const OutputFunction function_assignments[MAX_ACTUATORS]);

	void reset();

	void update(int num_outputs, float thrust_curve);

	void overrideValues(float outputs[MAX_ACTUATORS], int num_outputs);

	bool inTestMode() const { return _in_test_mode; }

private:

	uORB::Subscription _actuator_test_sub{ORB_ID(actuator_test)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	bool _in_test_mode{false};
	hrt_abstime _next_timeout{0};

	float _current_outputs[MAX_ACTUATORS];
	bool _output_overridden[MAX_ACTUATORS];
	const OutputFunction *_function_assignments;
};
