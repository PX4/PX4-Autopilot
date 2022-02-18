/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * Gimbal/mount driver.
 * Supported inputs:
 * - RC
 * - MAVLink gimbal protocol v1
 * - MAVLink gimbal protocol v2
 * - Test CLI commands
 * Supported outputs:
 * - PWM
 * - MAVLink gimbal protocol v1
 * - MAVLink gimbal protocol v2
 */

#include <stdlib.h>
#include <string.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>

#include "gimbal_params.h"
#include "input_mavlink.h"
#include "input_rc.h"
#include "input_test.h"
#include "output_rc.h"
#include "output_mavlink.h"

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/atomic.h>

using namespace time_literals;
using namespace gimbal;

static px4::atomic<bool> thread_should_exit {false};
static px4::atomic<bool> thread_running {false};

static constexpr int input_objs_len_max = 3;

struct ThreadData {
	InputBase *input_objs[input_objs_len_max] = {nullptr, nullptr, nullptr};
	int input_objs_len = 0;
	int last_input_active = -1;
	OutputBase *output_obj = nullptr;
	InputTest *test_input = nullptr;
};
static ThreadData *g_thread_data = nullptr;


static void usage();
static void update_params(ParameterHandles &param_handles, Parameters &params);
static bool initialize_params(ParameterHandles &param_handles, Parameters &params);

static int gimbal_thread_main(int argc, char *argv[]);
extern "C" __EXPORT int gimbal_main(int argc, char *argv[]);

static int gimbal_thread_main(int argc, char *argv[])
{
	ParameterHandles param_handles;
	Parameters params {};
	ThreadData thread_data;

	if (!initialize_params(param_handles, params)) {
		PX4_ERR("could not get mount parameters!");
		delete g_thread_data->test_input;
		return -1;
	}

	uORB::SubscriptionInterval parameter_update_sub{ORB_ID(parameter_update), 1_s};
	thread_running.store(true);
	ControlData control_data {};
	g_thread_data = &thread_data;

	thread_data.test_input = new InputTest(params);

	bool alloc_failed = false;

	thread_data.input_objs[thread_data.input_objs_len++] = thread_data.test_input;

	switch (params.mnt_mode_in) {
	case 0:
		// Automatic
		// MAVLINK_V2 as well as RC input are supported together.
		// Whichever signal is updated last, gets control, for RC there is a deadzone
		// to avoid accidental activation.
		thread_data.input_objs[thread_data.input_objs_len++] = new InputMavlinkGimbalV2(params);

		thread_data.input_objs[thread_data.input_objs_len++] = new InputRC(params);
		break;

	case 1: // RC only
		thread_data.input_objs[thread_data.input_objs_len++] = new InputRC(params);
		break;

	case 2: // MAVLINK_ROI commands only (to be deprecated)
		thread_data.input_objs[thread_data.input_objs_len++] = new InputMavlinkROI(params);
		break;

	case 3: // MAVLINK_DO_MOUNT commands only (to be deprecated)
		thread_data.input_objs[thread_data.input_objs_len++] = new InputMavlinkCmdMount(params);
		break;

	case 4: //MAVLINK_V2
		thread_data.input_objs[thread_data.input_objs_len++] = new InputMavlinkGimbalV2(params);
		break;

	default:
		PX4_ERR("invalid input mode %" PRId32, params.mnt_mode_in);
		break;
	}

	for (int i = 0; i < thread_data.input_objs_len; ++i) {
		if (!thread_data.input_objs[i]) {
			alloc_failed = true;
		}
	}

	if (alloc_failed) {
		PX4_ERR("input objs memory allocation failed");
		thread_should_exit.store(true);
	}

	if (!alloc_failed) {
		for (int i = 0; i < thread_data.input_objs_len; ++i) {
			if (thread_data.input_objs[i]->initialize() != 0) {
				PX4_ERR("Input %d failed", i);
				thread_should_exit.store(true);
			}
		}
	}

	switch (params.mnt_mode_out) {
	case 0: //AUX
		thread_data.output_obj = new OutputRC(params);

		if (!thread_data.output_obj) { alloc_failed = true; }

		break;

	case 1: //MAVLink gimbal v1 protocol
		thread_data.output_obj = new OutputMavlinkV1(params);

		if (!thread_data.output_obj) { alloc_failed = true; }

		break;

	case 2: //MAVLink gimbal v2 protocol
		thread_data.output_obj = new OutputMavlinkV2(params);

		if (!thread_data.output_obj) { alloc_failed = true; }

		break;

	default:
		PX4_ERR("invalid output mode %" PRId32, params.mnt_mode_out);
		thread_should_exit.store(true);
		break;
	}

	if (alloc_failed) {
		PX4_ERR("output memory allocation failed");
		thread_should_exit.store(true);
	}

	while (!thread_should_exit.load()) {

		const bool updated = parameter_update_sub.updated();

		if (updated) {
			parameter_update_s pupdate;
			parameter_update_sub.copy(&pupdate);
			update_params(param_handles, params);
		}

		if (thread_data.last_input_active == -1) {
			// Reset control as no one is active anymore, or yet.
			control_data.sysid_primary_control = 0;
			control_data.compid_primary_control = 0;
		}

		InputBase::UpdateResult update_result = InputBase::UpdateResult::NoUpdate;

		if (thread_data.input_objs_len > 0) {

			// get input: we cannot make the timeout too large, because the output needs to update
			// periodically for stabilization and angle updates.

			for (int i = 0; i < thread_data.input_objs_len; ++i) {

				const bool already_active = (thread_data.last_input_active == i);
				// poll only on active input to reduce latency, or on all if none is active
				const unsigned int poll_timeout =
					(already_active || thread_data.last_input_active == -1) ? 20 : 0;

				update_result = thread_data.input_objs[i]->update(poll_timeout, control_data, already_active);

				bool break_loop = false;

				switch (update_result) {
				case InputBase::UpdateResult::NoUpdate:
					if (already_active) {
						// No longer active.
						thread_data.last_input_active = -1;
					}

					break;

				case InputBase::UpdateResult::UpdatedActive:
					thread_data.last_input_active = i;
					break_loop = true;
					break;

				case InputBase::UpdateResult::UpdatedActiveOnce:
					thread_data.last_input_active = -1;
					break_loop = true;
					break;

				case InputBase::UpdateResult::UpdatedNotActive:
					if (already_active) {
						// No longer active
						thread_data.last_input_active = -1;
					}

					break;
				}

				if (break_loop) {
					break;
				}
			}

			if (params.mnt_do_stab == 1) {
				thread_data.output_obj->set_stabilize(true, true, true);

			} else if (params.mnt_do_stab == 2) {
				thread_data.output_obj->set_stabilize(false, false, true);

			} else {
				thread_data.output_obj->set_stabilize(false, false, false);
			}

			// Update output
			thread_data.output_obj->update(
				control_data,
				update_result != InputBase::UpdateResult::NoUpdate);

			// Only publish the mount orientation if the mode is not mavlink v1 or v2
			// If the gimbal speaks mavlink it publishes its own orientation.
			if (params.mnt_mode_out != 1 && params.mnt_mode_out != 2) { // 1 = MAVLink v1, 2 = MAVLink v2
				thread_data.output_obj->publish();
			}

		} else {
			// We still need to wake up regularly to check for thread exit requests
			px4_usleep(1e6);
		}
	}

	g_thread_data = nullptr;

	for (int i = 0; i < input_objs_len_max; ++i) {
		if (thread_data.input_objs[i]) {
			delete (thread_data.input_objs[i]);
			thread_data.input_objs[i] = nullptr;
		}
	}

	thread_data.input_objs_len = 0;

	if (thread_data.output_obj) {
		delete (thread_data.output_obj);
		thread_data.output_obj = nullptr;
	}

	thread_running.store(false);
	return 0;
}

int gimbal_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("missing command");
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running.load()) {
			PX4_WARN("mount driver already running");
			return 1;
		}

		thread_should_exit.store(false);

		int gimbal_task = px4_task_spawn_cmd("gimbal",
						     SCHED_DEFAULT,
						     SCHED_PRIORITY_DEFAULT,
						     2100,
						     gimbal_thread_main,
						     nullptr);

		int counter = 0;

		while (!thread_running.load() && gimbal_task >= 0) {
			px4_usleep(5000);

			if (++counter >= 100) {
				break;
			}
		}

		if (gimbal_task < 0) {
			PX4_ERR("failed to start");
			return -1;
		}

		return counter < 100 || thread_should_exit.load() ? 0 : -1;
	}

	else if (!strcmp(argv[1], "stop")) {

		if (!thread_running.load()) {
			PX4_WARN("mount driver not running");
			return 0;
		}

		thread_should_exit.store(true);

		while (thread_running.load()) {
			px4_usleep(100000);
		}

		return 0;
	}

	else if (!strcmp(argv[1], "test")) {

		if (thread_running.load() && g_thread_data && g_thread_data->test_input) {

			if (argc >= 4) {
				bool found_axis = false;
				const char *axis_names[3] = {"roll", "pitch", "yaw"};
				int angles[3] = { 0, 0, 0 };

				for (int arg_i = 2 ; arg_i < (argc - 1); ++arg_i) {
					for (int axis_i = 0; axis_i < 3; ++axis_i) {
						if (!strcmp(argv[arg_i], axis_names[axis_i])) {
							int angle_deg = (int)strtol(argv[arg_i + 1], nullptr, 0);
							angles[axis_i] = angle_deg;
							found_axis = true;
						}
					}
				}

				if (!found_axis) {
					usage();
					return -1;
				}

				g_thread_data->test_input->set_test_input(angles[0], angles[1], angles[2]);
				return 0;
			}

		} else {
			PX4_WARN("not running");
			usage();
			return 1;
		}
	}

	else if (!strcmp(argv[1], "status")) {
		if (thread_running.load() && g_thread_data && g_thread_data->test_input) {

			if (g_thread_data->input_objs_len == 0) {
				PX4_INFO("Input: None");

			} else {
				PX4_INFO("Input Selected");

				for (int i = 0; i < g_thread_data->input_objs_len; ++i) {
					if (i == g_thread_data->last_input_active) {
						g_thread_data->input_objs[i]->print_status();
					}
				}

				PX4_INFO("Input not selected");

				for (int i = 0; i < g_thread_data->input_objs_len; ++i) {
					if (i != g_thread_data->last_input_active) {
						g_thread_data->input_objs[i]->print_status();
					}
				}

			}

			if (g_thread_data->output_obj) {
				g_thread_data->output_obj->print_status();

			} else {
				PX4_INFO("Output: None");
			}

		} else {
			PX4_INFO("not running");
		}

		return 0;
	}

	PX4_ERR("unrecognized command");
	usage();
	return -1;
}

void update_params(ParameterHandles &param_handles, Parameters &params)
{
	param_get(param_handles.mnt_mode_in, &params.mnt_mode_in);
	param_get(param_handles.mnt_mode_out, &params.mnt_mode_out);
	param_get(param_handles.mnt_mav_sysid_v1, &params.mnt_mav_sysid_v1);
	param_get(param_handles.mnt_mav_compid_v1, &params.mnt_mav_compid_v1);
	param_get(param_handles.mnt_ob_lock_mode, &params.mnt_ob_lock_mode);
	param_get(param_handles.mnt_ob_norm_mode, &params.mnt_ob_norm_mode);
	param_get(param_handles.mnt_man_pitch, &params.mnt_man_pitch);
	param_get(param_handles.mnt_man_roll, &params.mnt_man_roll);
	param_get(param_handles.mnt_man_yaw, &params.mnt_man_yaw);
	param_get(param_handles.mnt_do_stab, &params.mnt_do_stab);
	param_get(param_handles.mnt_range_pitch, &params.mnt_range_pitch);
	param_get(param_handles.mnt_range_roll, &params.mnt_range_roll);
	param_get(param_handles.mnt_range_yaw, &params.mnt_range_yaw);
	param_get(param_handles.mnt_off_pitch, &params.mnt_off_pitch);
	param_get(param_handles.mnt_off_roll, &params.mnt_off_roll);
	param_get(param_handles.mnt_off_yaw, &params.mnt_off_yaw);
	param_get(param_handles.mav_sysid, &params.mav_sysid);
	param_get(param_handles.mav_compid, &params.mav_compid);
	param_get(param_handles.mnt_rate_pitch, &params.mnt_rate_pitch);
	param_get(param_handles.mnt_rate_yaw, &params.mnt_rate_yaw);
	param_get(param_handles.mnt_rc_in_mode, &params.mnt_rc_in_mode);
	param_get(param_handles.mnt_lnd_p_min, &params.mnt_lnd_p_min);
	param_get(param_handles.mnt_lnd_p_max, &params.mnt_lnd_p_max);
}

bool initialize_params(ParameterHandles &param_handles, Parameters &params)
{
	param_handles.mnt_mode_in = param_find("MNT_MODE_IN");
	param_handles.mnt_mode_out = param_find("MNT_MODE_OUT");
	param_handles.mnt_mav_sysid_v1 = param_find("MNT_MAV_SYSID");
	param_handles.mnt_mav_compid_v1 = param_find("MNT_MAV_COMPID");
	param_handles.mnt_ob_lock_mode = param_find("MNT_OB_LOCK_MODE");
	param_handles.mnt_ob_norm_mode = param_find("MNT_OB_NORM_MODE");
	param_handles.mnt_man_pitch = param_find("MNT_MAN_PITCH");
	param_handles.mnt_man_roll = param_find("MNT_MAN_ROLL");
	param_handles.mnt_man_yaw = param_find("MNT_MAN_YAW");
	param_handles.mnt_do_stab = param_find("MNT_DO_STAB");
	param_handles.mnt_range_pitch = param_find("MNT_RANGE_PITCH");
	param_handles.mnt_range_roll = param_find("MNT_RANGE_ROLL");
	param_handles.mnt_range_yaw = param_find("MNT_RANGE_YAW");
	param_handles.mnt_off_pitch = param_find("MNT_OFF_PITCH");
	param_handles.mnt_off_roll = param_find("MNT_OFF_ROLL");
	param_handles.mnt_off_yaw = param_find("MNT_OFF_YAW");
	param_handles.mav_sysid = param_find("MAV_SYS_ID");
	param_handles.mav_compid = param_find("MAV_COMP_ID");
	param_handles.mnt_rate_pitch = param_find("MNT_RATE_PITCH");
	param_handles.mnt_rate_yaw = param_find("MNT_RATE_YAW");
	param_handles.mnt_rc_in_mode = param_find("MNT_RC_IN_MODE");
	param_handles.mnt_lnd_p_min = param_find("MNT_LND_P_MIN");
	param_handles.mnt_lnd_p_max = param_find("MNT_LND_P_MAX");

	if (param_handles.mnt_mode_in == PARAM_INVALID ||
	    param_handles.mnt_mode_out == PARAM_INVALID ||
	    param_handles.mnt_mav_sysid_v1 == PARAM_INVALID ||
	    param_handles.mnt_mav_compid_v1 == PARAM_INVALID ||
	    param_handles.mnt_ob_lock_mode == PARAM_INVALID ||
	    param_handles.mnt_ob_norm_mode == PARAM_INVALID ||
	    param_handles.mnt_man_pitch == PARAM_INVALID ||
	    param_handles.mnt_man_roll == PARAM_INVALID ||
	    param_handles.mnt_man_yaw == PARAM_INVALID ||
	    param_handles.mnt_do_stab == PARAM_INVALID ||
	    param_handles.mnt_range_pitch == PARAM_INVALID ||
	    param_handles.mnt_range_roll == PARAM_INVALID ||
	    param_handles.mnt_range_yaw == PARAM_INVALID ||
	    param_handles.mnt_off_pitch == PARAM_INVALID ||
	    param_handles.mnt_off_roll == PARAM_INVALID ||
	    param_handles.mnt_off_yaw == PARAM_INVALID ||
	    param_handles.mav_sysid == PARAM_INVALID ||
	    param_handles.mav_compid == PARAM_INVALID ||
	    param_handles.mnt_rate_pitch == PARAM_INVALID ||
	    param_handles.mnt_rate_yaw == PARAM_INVALID ||
	    param_handles.mnt_rc_in_mode == PARAM_INVALID ||
	    param_handles.mnt_lnd_p_min == PARAM_INVALID ||
	    param_handles.mnt_lnd_p_max == PARAM_INVALID
	   ) {
		return false;
	}

	update_params(param_handles, params);
	return true;
}

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Mount/gimbal Gimbal control driver. It maps several different input methods (eg. RC or MAVLink) to a configured
output (eg. AUX channels or MAVLink).

Documentation how to use it is on the [gimbal_control](https://docs.px4.io/master/en/advanced/gimbal_control.html) page.

### Examples
Test the output by setting a angles (all omitted axes are set to 0):
$ gimbal test pitch -45 yaw 30
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gimbal", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test the output: set a fixed angle for one or multiple axes (gimbal must be running)");
	PRINT_MODULE_USAGE_ARG("roll|pitch|yaw <angle>", "Specify an axis and an angle in degrees", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}
