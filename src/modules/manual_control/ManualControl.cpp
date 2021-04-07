/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
	}

	const hrt_abstime rc_timeout = _param_com_rc_loss_t.get() * 1_s;

	bool publish_once = false;
	int selected_manual_input = -1;

	for (int i = 0; i < MAX_MANUAL_INPUT_COUNT; i++) {
		manual_control_input_s manual_control_input;

		if (_manual_control_input_subs[i].update(&manual_control_input)) {

			bool publish = false;

			if ((_param_com_rc_in_mode.get() == 0)
			    && (manual_control_input.data_source == manual_control_input_s::SOURCE_RC)) {

				publish = true;

			} else if ((_param_com_rc_in_mode.get() == 1)
				   && (manual_control_input.data_source >= manual_control_input_s::SOURCE_MAVLINK_0)) {

				publish = true;

			} else {
				// otherwise, first come, first serve (REVIEW)
				publish = true;
			}


			bool available = (hrt_elapsed_time(&manual_control_input.timestamp) < rc_timeout)
					 && (hrt_elapsed_time(&_manual_control_input[i].timestamp) < rc_timeout);

			if (publish && available && !publish_once) {
				const float dt_inv = 1e6f / static_cast<float>(manual_control_input.timestamp_sample -
						     _manual_control_input[i].timestamp_sample);

				manual_control_setpoint_s manual_control_setpoint{};

				manual_control_setpoint.timestamp_sample = manual_control_input.timestamp_sample;
				manual_control_setpoint.x = manual_control_input.x;
				manual_control_setpoint.y = manual_control_input.y;
				manual_control_setpoint.z = manual_control_input.z;
				manual_control_setpoint.r = manual_control_input.r;
				manual_control_setpoint.vx = (manual_control_input.x - _manual_control_input[i].x) * dt_inv;
				manual_control_setpoint.vy = (manual_control_input.y - _manual_control_input[i].y) * dt_inv;
				manual_control_setpoint.vz = (manual_control_input.z - _manual_control_input[i].z) * dt_inv;
				manual_control_setpoint.vr = (manual_control_input.r - _manual_control_input[i].r) * dt_inv;
				manual_control_setpoint.flaps = manual_control_input.flaps;
				manual_control_setpoint.aux1 = manual_control_input.aux1;
				manual_control_setpoint.aux2 = manual_control_input.aux2;
				manual_control_setpoint.aux3 = manual_control_input.aux3;
				manual_control_setpoint.aux4 = manual_control_input.aux4;
				manual_control_setpoint.aux5 = manual_control_input.aux5;
				manual_control_setpoint.aux6 = manual_control_input.aux6;
				manual_control_setpoint.data_source = manual_control_input.data_source;


				// user arm/disarm gesture
				const bool right_stick_centered = (fabsf(manual_control_input.x) < 0.1f) && (fabsf(manual_control_input.y) < 0.1f);
				const bool stick_lower_left = (manual_control_input.z < 0.1f) && (manual_control_input.r < -0.9f);
				const bool stick_lower_right = (manual_control_input.z < 0.1f) && (manual_control_input.r > 0.9f);

				_stick_arm_hysteresis.set_state_and_update(stick_lower_right && right_stick_centered, manual_control_input.timestamp);
				_stick_disarm_hysteresis.set_state_and_update(stick_lower_left && right_stick_centered, manual_control_input.timestamp);
				manual_control_setpoint.arm_gesture = _stick_arm_hysteresis.get_state();
				manual_control_setpoint.disarm_gesture = _stick_disarm_hysteresis.get_state();


				// user wants override
				const float minimum_stick_change = 0.01f * _param_com_rc_stick_ov.get();
				const bool rpy_moved = (fabsf(manual_control_input.x - _manual_control_input[i].x) > minimum_stick_change)
						       || (fabsf(manual_control_input.y - _manual_control_input[i].y) > minimum_stick_change)
						       || (fabsf(manual_control_input.r - _manual_control_input[i].r) > minimum_stick_change);

				// Throttle change value doubled to achieve the same scaling even though the range is [0,1] instead of [-1,1]
				const bool throttle_moved = (fabsf(manual_control_input.z - _manual_control_input[i].z) * 2.f > minimum_stick_change);
				const bool use_throttle = !(_param_rc_override.get() & OverrideBits::OVERRIDE_IGNORE_THROTTLE_BIT);

				manual_control_setpoint.user_override = rpy_moved || (use_throttle && throttle_moved);


				manual_control_setpoint.timestamp = hrt_absolute_time();
				_manual_control_setpoint_pub.publish(manual_control_setpoint);
				publish_once = true;

				selected_manual_input = i;
			}

			_manual_control_input[i] = manual_control_input;
			_available[i] = available;
		}
	}

	if ((selected_manual_input >= 0) && (selected_manual_input != _selected_manual_input)) {
		if (_selected_manual_input >= 0) {
			PX4_INFO("selected manual_control_input changed %d -> %d", _selected_manual_input, selected_manual_input);
		}

		_manual_control_input_subs[selected_manual_input].registerCallback();
		_selected_manual_input = selected_manual_input;
	}

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
