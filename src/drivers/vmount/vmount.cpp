/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file vmount.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 * MAV_MOUNT driver for controlling mavlink gimbals, rc gimbals/servors and
 * future kinds of mounts.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <px4_defines.h>
#include <px4_tasks.h>

#include "input_mavlink.h"
#include "input_rc.h"
#include "input_test.h"
#include "output_rc.h"
#include "output_mavlink.h"

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>

#include <px4_config.h>

using namespace vmount;

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
struct ThreadData {
	InputBase *input_obj = nullptr;
	OutputBase *output_obj = nullptr;
};
static volatile ThreadData *g_thread_data = nullptr;

struct Parameters {
	int mnt_mode_in;
	int mnt_mode_out;
	int mnt_mav_sysid;
	int mnt_mav_compid;
	int mnt_ob_lock_mode;
	int mnt_ob_norm_mode;
	int mnt_man_control;
	int mnt_man_pitch;
	int mnt_man_roll;
	int mnt_man_yaw;

	bool operator!=(const Parameters &p)
	{
		return mnt_mode_in != p.mnt_mode_in ||
		       mnt_mode_out != p.mnt_mode_out ||
		       mnt_mav_sysid != p.mnt_mav_sysid ||
		       mnt_mav_compid != p.mnt_mav_compid ||
		       mnt_ob_lock_mode != p.mnt_ob_lock_mode ||
		       mnt_ob_norm_mode != p.mnt_ob_norm_mode ||
		       mnt_man_control != p.mnt_man_control ||
		       mnt_man_pitch != p.mnt_man_pitch ||
		       mnt_man_roll != p.mnt_man_roll ||
		       mnt_man_yaw != p.mnt_man_yaw;
	}
};

struct ParameterHandles {
	param_t mnt_mode_in;
	param_t mnt_mode_out;
	param_t mnt_mav_sysid;
	param_t mnt_mav_compid;
	param_t mnt_ob_lock_mode;
	param_t mnt_ob_norm_mode;
	param_t mnt_man_control;
	param_t mnt_man_pitch;
	param_t mnt_man_roll;
	param_t mnt_man_yaw;
};


/* functions */
static void usage();
static void update_params(ParameterHandles &param_handles, Parameters &params, bool &got_changes);
static bool get_params(ParameterHandles &param_handles, Parameters &params);

static int vmount_thread_main(int argc, char *argv[]);
extern "C" __EXPORT int vmount_main(int argc, char *argv[]);

static void usage()
{
	PX4_INFO("usage: vmount {start|stop|status|test}");
	PX4_INFO("       vmount test {roll|pitch|yaw} <angle_deg>");
}

static int vmount_thread_main(int argc, char *argv[])
{
	ParameterHandles param_handles;
	Parameters params;
	OutputConfig output_config;
	ThreadData thread_data;
	memset(&params, 0, sizeof(params));


	InputTest *test_input = nullptr;

#ifdef __PX4_NUTTX
	/* the NuttX optarg handler does not
	 * ignore argv[0] like the POSIX handler
	 * does, nor does it deal with non-flag
	 * verbs well. So we Remove the application
	 * name and the verb.
	 */
	argc -= 1;
	argv += 1;
#endif

	if (argc > 0 && !strcmp(argv[0], "test")) {
		PX4_INFO("Starting in test mode");

		const char *axis_names[3] = {"roll", "pitch", "yaw"};
		float angles[3] = { 0.f, 0.f, 0.f };

		if (argc == 3) {
			bool found_axis = false;

			for (int i = 0 ; i < 3; ++i) {
				if (!strcmp(argv[1], axis_names[i])) {
					long angle_deg = strtol(argv[2], nullptr, 0);
					angles[i] = (float)angle_deg;
					found_axis = true;
				}
			}

			if (!found_axis) {
				usage();
				return -1;
			}

			test_input = new InputTest(angles[0], angles[1], angles[2]);

			if (!test_input) {
				PX4_ERR("memory allocation failed");
				return -1;
			}

		} else {
			usage();
			return -1;
		}
	}

	if (!get_params(param_handles, params)) {
		PX4_ERR("could not get mount parameters!");
		return -1;
	}

	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	thread_running = true;
	ControlData *control_data = nullptr;
	InputRC *manual_input = nullptr;
	g_thread_data = &thread_data;

	while (!thread_should_exit) {

		if (!thread_data.input_obj && (params.mnt_mode_in != 0 || test_input)) { //need to initialize

			output_config.gimbal_normal_mode_value = params.mnt_ob_norm_mode;
			output_config.gimbal_retracted_mode_value = params.mnt_ob_lock_mode;
			output_config.mavlink_sys_id = params.mnt_mav_sysid;
			output_config.mavlink_comp_id = params.mnt_mav_compid;

			if (test_input) {
				thread_data.input_obj = test_input;

			} else {
				if (params.mnt_man_control) {
					manual_input = new InputRC(params.mnt_man_roll, params.mnt_man_pitch, params.mnt_man_yaw);

					if (!manual_input) {
						PX4_ERR("memory allocation failed");
						break;
					}
				}

				switch (params.mnt_mode_in) {
				case 1: //RC
					if (manual_input) {
						thread_data.input_obj = manual_input;
						manual_input = nullptr;

					} else {
						thread_data.input_obj = new InputRC(params.mnt_man_roll, params.mnt_man_pitch, params.mnt_man_yaw);
					}

					break;

				case 2: //MAVLINK_ROI
					thread_data.input_obj = new InputMavlinkROI(manual_input);
					break;

				case 3: //MAVLINK_DO_MOUNT
					thread_data.input_obj = new InputMavlinkCmdMount(manual_input);
					break;

				default:
					PX4_ERR("invalid input mode %i", params.mnt_mode_in);
					break;
				}
			}

			switch (params.mnt_mode_out) {
			case 0: //AUX
				thread_data.output_obj = new OutputRC(output_config);
				break;

			case 1: //MAVLINK
				thread_data.output_obj = new OutputMavlink(output_config);
				break;

			default:
				PX4_ERR("invalid output mode %i", params.mnt_mode_out);
				break;
			}

			if (!thread_data.input_obj || !thread_data.output_obj) {
				PX4_ERR("memory allocation failed");
				thread_should_exit = true;
				break;
			}

			int ret = thread_data.output_obj->initialize();

			if (ret) {
				PX4_ERR("failed to initialize output mode (%i)", ret);
				thread_should_exit = true;
				break;
			}
		}

		if (thread_data.input_obj) {

			//get input: we cannot make the timeout too large, because the output needs to update
			//periodically for stabilization and angle updates.
			int ret = thread_data.input_obj->update(50, &control_data);

			if (ret) {
				PX4_ERR("failed to read input (%i)", ret);
				break;
			}

			//update output
			ret = thread_data.output_obj->update(control_data);

			if (ret) {
				PX4_ERR("failed to write output (%i)", ret);
				break;
			}

			thread_data.output_obj->publish();

		} else {
			//wait for parameter changes. We still need to wake up regularily to check for thread exit requests
			usleep(1e6);
		}

		if (test_input && test_input->finished()) {
			thread_should_exit = true;
			break;
		}


		//check for parameter changes
		bool updated;

		if (orb_check(parameter_update_sub, &updated) == 0 && updated) {
			parameter_update_s param_update;
			orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_update);
			update_params(param_handles, params, updated);

			if (updated) {
				//re-init objects
				if (thread_data.input_obj) {
					delete (thread_data.input_obj);
					thread_data.input_obj = nullptr;
				}

				if (thread_data.output_obj) {
					delete (thread_data.output_obj);
					thread_data.output_obj = nullptr;
				}

				if (manual_input) {
					delete (manual_input);
					manual_input = nullptr;
				}
			}
		}
	}

	g_thread_data = nullptr;

	orb_unsubscribe(parameter_update_sub);

	if (thread_data.input_obj) {
		delete (thread_data.input_obj);
		thread_data.input_obj = nullptr;
	}

	if (thread_data.output_obj) {
		delete (thread_data.output_obj);
		thread_data.output_obj = nullptr;
	}

	if (manual_input) {
		delete (manual_input);
		manual_input = nullptr;
	}

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int vmount_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("missing command");
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "start") || !strcmp(argv[1], "test")) {

		/* this is not an error */
		if (thread_running) {
			PX4_WARN("mount driver already running");
			return 0;
		}

		thread_should_exit = false;
		int vmount_task = px4_task_spawn_cmd("vmount",
						     SCHED_DEFAULT,
						     SCHED_PRIORITY_DEFAULT + 40,
						     1500,
						     vmount_thread_main,
						     (char *const *)argv + 1);

		int counter = 0;

		while (!thread_running && vmount_task >= 0) {
			usleep(5000);

			if (++counter >= 100) {
				break;
			}
		}

		if (vmount_task < 0) {
			PX4_ERR("failed to start");
			return -1;
		}

		return counter < 100 || thread_should_exit ? 0 : -1;
	}

	if (!strcmp(argv[1], "stop")) {

		/* this is not an error */
		if (!thread_running) {
			PX4_WARN("mount driver not running");
			return 0;
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(100000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running && g_thread_data) {

			if (g_thread_data->input_obj) {
				g_thread_data->input_obj->print_status();

			} else {
				PX4_INFO("Input: None");
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

void update_params(ParameterHandles &param_handles, Parameters &params, bool &got_changes)
{
	Parameters prev_params = params;
	param_get(param_handles.mnt_mode_in, &params.mnt_mode_in);
	param_get(param_handles.mnt_mode_out, &params.mnt_mode_out);
	param_get(param_handles.mnt_mav_sysid, &params.mnt_mav_sysid);
	param_get(param_handles.mnt_mav_compid, &params.mnt_mav_compid);
	param_get(param_handles.mnt_ob_lock_mode, &params.mnt_ob_lock_mode);
	param_get(param_handles.mnt_ob_norm_mode, &params.mnt_ob_norm_mode);
	param_get(param_handles.mnt_man_control, &params.mnt_man_control);
	param_get(param_handles.mnt_man_pitch, &params.mnt_man_pitch);
	param_get(param_handles.mnt_man_roll, &params.mnt_man_roll);
	param_get(param_handles.mnt_man_yaw, &params.mnt_man_yaw);

	got_changes = prev_params != params;
}

bool get_params(ParameterHandles &param_handles, Parameters &params)
{
	param_handles.mnt_mode_in = param_find("MNT_MODE_IN");
	param_handles.mnt_mode_out = param_find("MNT_MODE_OUT");
	param_handles.mnt_mav_sysid = param_find("MNT_MAV_SYSID");
	param_handles.mnt_mav_compid = param_find("MNT_MAV_COMPID");
	param_handles.mnt_ob_lock_mode = param_find("MNT_OB_LOCK_MODE");
	param_handles.mnt_ob_norm_mode = param_find("MNT_OB_NORM_MODE");
	param_handles.mnt_man_control = param_find("MNT_MAN_CONTROL");
	param_handles.mnt_man_pitch = param_find("MNT_MAN_PITCH");
	param_handles.mnt_man_roll = param_find("MNT_MAN_ROLL");
	param_handles.mnt_man_yaw = param_find("MNT_MAN_YAW");

	if (param_handles.mnt_mode_in == PARAM_INVALID ||
	    param_handles.mnt_mode_out == PARAM_INVALID ||
	    param_handles.mnt_mav_sysid == PARAM_INVALID ||
	    param_handles.mnt_mav_compid == PARAM_INVALID ||
	    param_handles.mnt_ob_lock_mode == PARAM_INVALID ||
	    param_handles.mnt_ob_norm_mode == PARAM_INVALID ||
	    param_handles.mnt_man_control == PARAM_INVALID ||
	    param_handles.mnt_man_pitch == PARAM_INVALID ||
	    param_handles.mnt_man_roll == PARAM_INVALID ||
	    param_handles.mnt_man_yaw == PARAM_INVALID) {
		return false;
	}

	bool dummy;
	update_params(param_handles, params, dummy);
	return true;
}
