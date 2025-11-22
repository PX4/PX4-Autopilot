/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file TurtelMode.cpp
 *
 * Flight task for turtle mode - allows manual control for flipping a crashed drone
 */

#include "TurtelMode.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <parameters/param.h>
#include <inttypes.h>

using namespace matrix;

TurtleMode::~TurtleMode()
{
	// Set DSHOT_3D_ENABLE parameter to zero when exiting turtle mode
	param_t param = param_find("DSHOT_3D_ENABLE");
	
	if (param != PARAM_INVALID) {
		int32_t value = 0;
		param_set(param, &value);
	}
}

bool TurtleMode::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);

	// Set DSHOT_3D_ENABLE to 1 for turtle mode
	param_t param = param_find("DSHOT_3D_ENABLE");
	
	if (param != PARAM_INVALID) {
		int32_t value = 1;
		param_set(param, &value);
	}
	
	return ret;
}

bool TurtleMode::update()
{
	bool ret = FlightTaskManualAltitude::update();



	// In turtle mode, we want full manual control without altitude lock
	// The base class handles most of the work, but we ensure no position locking

	_publishMotorOutputs();
	return ret;
}


void TurtleMode::_publishMotorOutputs(){

	actuator_motors_s motor_outputs;
	motor_outputs.timestamp = hrt_absolute_time();
	motor_outputs.control[0] = 0.2f;
	motor_outputs.control[1] = 0.2f;
	motor_outputs.control[2] = 0.2f;
	motor_outputs.control[3] = 0.2f;

	PX4_INFO("Publishing motor outputs: %f, %f, %f, %f", (double)motor_outputs.control[0], (double)motor_outputs.control[1], (double)motor_outputs.control[2], (double)motor_outputs.control[3]);
	_actuator_motors_pub.publish(motor_outputs);
}


