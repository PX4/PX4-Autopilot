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

#include "rs_canfd_sender.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>


int RoboSubCANFDSender::print_status()
{
        PX4_INFO("Running");
        // TODO: print additional runtime information about the state of the module
        return 0;
}

int RoboSubCANFDSender::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoboSubCANFDSender::task_spawn(int argc, char *argv[])
{
	RoboSubCANFDSender *instance = new RoboSubCANFDSender();

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

bool RoboSubCANFDSender::init()
{
	if (!control_lamp_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	PX4_DEBUG("RoboSubCANFDSender::init()");
	return true;
}


RoboSubCANFDSender::RoboSubCANFDSender()
        : ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_last_sent = hrt_absolute_time();
}

void RoboSubCANFDSender::Run()
{
	PX4_INFO("RoboSubCANFDSender::Run()");

	if (should_exit()) {
		exit_and_cleanup();
		return;
	}
	// initialize parameters
	parameters_update();

	if (control_lamp_sub.update(&control_lamp_msg)) {
		send_id.can_id_seg.module_id_src = MODULE_ID;
		send_id.can_id_seg.client_id_src = 0x0A;
		send_id.can_id_seg.module_id_des = 0x11;
		send_id.can_id_seg.client_id_des = 0x06;
		send_id.can_id_seg.command_type = 0x05; // Control lamp command
		send_id.can_id_seg.rest = 0x00; // Rest of the ID is 0

		_send_raw_canfd_msg.id = (send_id.id | CAN_EFF_FLAG);
		_send_raw_canfd_msg.data[0] = 0x0A;
		memcpy(_send_raw_canfd_msg.data + 1, control_lamp_msg.color, 4);
		_send_raw_canfd_msg.len = 5;
		_send_raw_canfd_msg.timestamp = hrt_absolute_time();
		// Publish the test message
		send_raw_canfd_pub.publish(_send_raw_canfd_msg);
	}

}

void RoboSubCANFDSender::parameters_update(bool force)
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

int RoboSubCANFDSender::print_usage(const char *reason)
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

int rs_canfd_sender_main(int argc, char *argv[])
{
        return RoboSubCANFDSender::main(argc, argv);
}
