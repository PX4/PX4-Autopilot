/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "rs_canfd_receiver.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>
#include <sys/ioctl.h>

int RoboSubCANFDReceiver::print_status()
{
        PX4_INFO("Running");
        // TODO: print additional runtime information about the state of the module
        return 0;
}

int RoboSubCANFDReceiver::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "stop")) {
		get_instance()->exit_and_cleanup();
		return 0;
	}

	return print_usage("unknown command");
}

int RoboSubCANFDReceiver::task_spawn(int argc, char *argv[])
{
	RoboSubCANFDReceiver *instance = new RoboSubCANFDReceiver();

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

bool RoboSubCANFDReceiver::init()
{
	if (!_raw_canfd_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	PX4_DEBUG("RoboSubCANFDReceiver::init()");
	return true;
}

RoboSubCANFDReceiver::RoboSubCANFDReceiver()
        : ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	/* performance counters */
	// _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// perf_free(_loop_perf);
}


void RoboSubCANFDReceiver::Run()
{
	PX4_INFO("RoboSubCANFDReceiver::Run()");
	parameters_update();

	_raw_canfd_sub.update(&_raw_canfd_msg);

	received_id.id = _raw_canfd_msg.id; // Put the received can id in the union to parse the id.

	if (received_id.can_id_seg.module_id_src == 0x02) { // Check if the src module is 0x02 (mainbrain)
		if (received_id.can_id_seg.client_id_src == 0x04) { // check if the src client id is 0x04 (LP4 GPIO non contact water level)
			water_presence_s water_presence_msg{}; // create the temp message struct
			water_presence_msg.timestamp = hrt_absolute_time(); // set the timestamp
			water_presence_msg.water_detected = _raw_canfd_msg.data[0]; // set the water detected bit, this should be the first byte

			water_presence_pub.publish(water_presence_msg); // publish the data
		}
	}

	return;
}

void RoboSubCANFDReceiver::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
	// perf_end(_loop_perf);
}

int RoboSubCANFDReceiver::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s\n", reason);
        }
        PRINT_MODULE_DESCRIPTION(
                R"DESCRSTR(
### Description
Section that describes the provided module functionality.
This is a template for a module running as a task in the background with start/stop/status functionality.
### Implementation
Section describing the high-level implementation of this module.
### Examples
CLI usage example:
$ module start -f -p 42
)DESCRSTR");
        PRINT_MODULE_USAGE_NAME("module", "rs canfd sender");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return 0;
}

int rs_canfd_receiver_main(int argc, char *argv[])
{
        return RoboSubCANFDReceiver::main(argc, argv);
}
