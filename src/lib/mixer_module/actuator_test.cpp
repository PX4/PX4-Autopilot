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

#include "actuator_test.hpp"

#include "functions/FunctionMotors.hpp"

using namespace time_literals;

ActuatorTest::ActuatorTest(const OutputFunction function_assignments[MAX_ACTUATORS])
	: _function_assignments(function_assignments)
{
	reset();
}

void ActuatorTest::update(int num_outputs, float thrust_curve)
{
	const hrt_abstime now = hrt_absolute_time();

	actuator_test_s actuator_test;

	while (_actuator_test_sub.update(&actuator_test)) {
		if (actuator_test.timestamp == 0 ||
		    hrt_elapsed_time(&actuator_test.timestamp) > 100_ms) {
			continue;
		}

		for (int i = 0; i < num_outputs; ++i) {
			if ((int)_function_assignments[i] == actuator_test.function) {
				bool in_test_mode = actuator_test.action == actuator_test_s::ACTION_DO_CONTROL;

				// entering test mode?
				if (!_in_test_mode && in_test_mode) {
					reset();
					_in_test_mode = true;
				}

				_output_overridden[i] = in_test_mode;

				if (in_test_mode) {
					if (actuator_test.timeout_ms > 0) {
						_next_timeout = now + actuator_test.timeout_ms * 1000;
					}

					float value = actuator_test.value;

					// handle motors
					if ((int)OutputFunction::Motor1 <= actuator_test.function && actuator_test.function <= (int)OutputFunction::MotorMax) {
						actuator_motors_s motors;
						motors.reversible_flags = 0;
						_actuator_motors_sub.copy(&motors);
						int motor_idx = actuator_test.function - (int)OutputFunction::Motor1;
						FunctionMotors::updateValues(motors.reversible_flags >> motor_idx, thrust_curve, &value, 1);
					}

					// handle servos: add trim
					if ((int)OutputFunction::Servo1 <= actuator_test.function && actuator_test.function <= (int)OutputFunction::ServoMax) {
						actuator_servos_trim_s trim{};
						_actuator_servos_trim_sub.copy(&trim);
						int idx = actuator_test.function - (int)OutputFunction::Servo1;

						if (idx < actuator_servos_trim_s::NUM_CONTROLS) {
							value += trim.trim[idx];
						}
					}

					_current_outputs[i] = value;

				} else {
					_current_outputs[i] = NAN;
				}
			}
		}
	}

	if (_in_test_mode) {
		// check if all disabled
		bool any_overridden = false;

		for (int i = 0; i < num_outputs; ++i) {
			any_overridden |= _output_overridden[i];
		}

		if (!any_overridden) {
			_in_test_mode = false;
		}
	}

	// check for timeout
	if (_in_test_mode && _next_timeout != 0 && now > _next_timeout) {
		_in_test_mode = false;
	}
}

void ActuatorTest::overrideValues(float outputs[MAX_ACTUATORS], int num_outputs)
{
	if (_in_test_mode) {
		for (int i = 0; i < num_outputs; ++i) {
			outputs[i] = _current_outputs[i];
		}
	}
}

void ActuatorTest::reset()
{
	_in_test_mode = false;
	_next_timeout = 0;

	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		_current_outputs[i] = NAN;
		_output_overridden[i] = false;
	}
}
