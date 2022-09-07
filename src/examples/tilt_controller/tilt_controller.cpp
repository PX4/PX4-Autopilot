/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file tilt_controller.cpp
 *
 * Simple controller aligning the payload and vehicle pitch angles
 * The control loop is triggered by a new payload_attitude estimate being published by EKF2
 * Setup:
 * - external IMU set as "payload" on a tilted mechanism controlled by a servo
 *   connected to a PMW output linked to the gimbal pitch output
 */

#include <mathlib/mathlib.h>

#include "tilt_controller.hpp"

using namespace time_literals;

TiltController::TiltController() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
	reset();
}

TiltController::~TiltController()
{
	perf_free(_cycle_perf);
}

bool TiltController::init()
{
	if (!_payload_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void TiltController::reset()
{
	_tilt_setpoint = 0.f;
}

void TiltController::updateParams()
{
	ModuleParams::updateParams();

	// additional checks
}

void TiltController::Run()
{
	if (should_exit()) {
		_payload_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_begin(_cycle_perf);

	vehicle_attitude_s pyld_attitude;

	if (!_payload_attitude_sub.update(&pyld_attitude)) {
		return;
	}

	const float dt = (pyld_attitude.timestamp - _timestamp_last) * 1e-6f;
	_timestamp_last = pyld_attitude.timestamp;

	if ((dt > 0.001f) && (dt < 1.f)) {

		vehicle_attitude_s vehicle_attitude;

		if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
			// Use the vehicle pitch as the setpoint
			_tilt_setpoint = matrix::Eulerf(matrix::Quatf(vehicle_attitude.q)).theta();
		}

		const float tilt = matrix::Eulerf(matrix::Quatf(pyld_attitude.q)).theta();
		const float error = _tilt_setpoint - tilt;
		const float control_speed = _param_tc_p.get() * error;
		_control_output = math::constrain(_control_output + control_speed * dt, -1.f, 1.f);

		publishControl(pyld_attitude.timestamp);
	}

	perf_end(_cycle_perf);
}

void TiltController::publishControl(const hrt_abstime &timestamp_sample)
{
	actuator_controls_s control{};

	control.timestamp_sample = timestamp_sample;
	control.timestamp = hrt_absolute_time();

	control.control[actuator_controls_s::INDEX_PITCH] = _control_output;

	_actuator_controls_pub.publish(control);
}

int TiltController::task_spawn(int argc, char *argv[])
{
	TiltController *instance = new TiltController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int TiltController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TiltController::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int TiltController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tilt_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int tilt_controller_main(int argc, char *argv[])
{
	return TiltController::main(argc, argv);
}
