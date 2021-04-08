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

#include "ManualControl.hpp"

#include <drivers/drv_hrt.h>

namespace manual_control
{

ManualControl::ManualControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ManualControl::~ManualControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ManualControl::init()
{
	ScheduleNow();
	return true;
}

void ManualControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_stick_arm_hysteresis.set_hysteresis_time_from(false, _param_rc_arm_hyst.get() * 1_ms);
		_stick_disarm_hysteresis.set_hysteresis_time_from(false, _param_rc_arm_hyst.get() * 1_ms);

		_selector.set_rc_in_mode(_param_com_rc_in_mode.get());
		_selector.set_timeout(_param_com_rc_loss_t.get() * 1_s);
	}

	bool found_at_least_one = false;
	const hrt_abstime now = hrt_absolute_time();

	for (int i = 0; i < MAX_MANUAL_INPUT_COUNT; i++) {
		manual_control_input_s manual_control_input;

		if (_manual_control_input_subs[i].update(&manual_control_input)) {

			found_at_least_one = true;

			_selector.update_manual_control_input(now, manual_control_input);

		}
	}

	if (!found_at_least_one) {
		_selector.update_time_only(now);
	}

	if (_selector.setpoint().valid) {
		_published_invalid_once = false;

		// user arm/disarm gesture
		const bool right_stick_centered = (fabsf(_selector.setpoint().x) < 0.1f) && (fabsf(_selector.setpoint().y) < 0.1f);
		const bool stick_lower_left = (_selector.setpoint().z < 0.1f) && (_selector.setpoint().r < -0.9f);
		const bool stick_lower_right = (_selector.setpoint().z < 0.1f) && (_selector.setpoint().r > 0.9f);

		_stick_arm_hysteresis.set_state_and_update(stick_lower_right && right_stick_centered, _selector.setpoint().timestamp);
		_stick_disarm_hysteresis.set_state_and_update(stick_lower_left && right_stick_centered, _selector.setpoint().timestamp);
		_selector.setpoint().arm_gesture = _stick_arm_hysteresis.get_state();
		_selector.setpoint().disarm_gesture = _stick_disarm_hysteresis.get_state();


		// user wants override
		//const float minimum_stick_change = 0.01f * _param_com_rc_stick_ov.get();

		// FIXME: we need previous
		//const bool rpy_moved = (fabsf(_selector.setpoint().x - _manual_control_input[i].x) > minimum_stick_change)
		//		       || (fabsf(_selector.setpoint().y - _manual_control_input[i].y) > minimum_stick_change)
		//		       || (fabsf(_selector.setpoint().r - _manual_control_input[i].r) > minimum_stick_change);

		// Throttle change value doubled to achieve the same scaling even though the range is [0,1] instead of [-1,1]
		//const bool throttle_moved = (fabsf(_selector.setpoint().z - _manual_control_input[i].z) * 2.f > minimum_stick_change);
		//const bool use_throttle = !(_param_rc_override.get() & OverrideBits::OVERRIDE_IGNORE_THROTTLE_BIT);

		//_selector.setpoint().user_override = rpy_moved || (use_throttle && throttle_moved);


		_selector.setpoint().timestamp = now;
		_manual_control_setpoint_pub.publish(_selector.setpoint());

	} else {
		_published_invalid_once = true;
		_manual_control_setpoint_pub.publish(_selector.setpoint());
	}

	// FIXME: get from selector
	//if ((selected_manual_input >= 0) && (selected_manual_input != _selected_manual_input)) {
	//	if (_selected_manual_input >= 0) {
	//		PX4_INFO("selected manual_control_input changed %d -> %d", _selected_manual_input, selected_manual_input);
	//	}

	//	_manual_control_input_subs[selected_manual_input].registerCallback();
	//	_selected_manual_input = selected_manual_input;
	//}

	// reschedule timeout
	ScheduleDelayed(200_ms);

	perf_end(_loop_perf);
}

int ManualControl::task_spawn(int argc, char *argv[])
{
	ManualControl *instance = new ManualControl();

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

int ManualControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int ManualControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ManualControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("manual_control", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

}; // namespace manual_control

extern "C" __EXPORT int manual_control_main(int argc, char *argv[])
{
	return manual_control::ManualControl::main(argc, argv);
}
