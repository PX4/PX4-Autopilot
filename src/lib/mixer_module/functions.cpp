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

#include "functions.hpp"

FunctionMotors::FunctionMotors(const Context &context)
	: _topic(&context.work_item, ORB_ID(actuator_motors)),
	  _reversible_motors(context.reversible_motors)
{
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		_data.control[i] = NAN;
	}
}
void FunctionMotors::update()
{
	bool updated = _topic.update(&_data);

	if (updated && !_reversible_motors) {
		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			if (_data.control[i] < -FLT_EPSILON) {
				_data.control[i] = NAN;

			} else {
				// remap from [0, 1] to [-1, 1]
				_data.control[i] = _data.control[i] * 2.f - 1.f;
			}
		}
	}
}

FunctionActuatorSet::FunctionActuatorSet()
{
	for (int i = 0; i < max_num_actuators; ++i) {
		_data[i] = NAN;
	}
}

void FunctionActuatorSet::update()
{
	vehicle_command_s vehicle_command;

	while (_topic.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR) {
			int index = (int)(vehicle_command.param7 + 0.5f);

			if (index == 0) {
				_data[0] = vehicle_command.param1;
				_data[1] = vehicle_command.param2;
				_data[2] = vehicle_command.param3;
				_data[3] = vehicle_command.param4;
				_data[4] = vehicle_command.param5;
				_data[5] = vehicle_command.param6;
			}
		}
	}
}
