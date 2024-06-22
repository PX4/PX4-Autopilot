/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "ModeRegister.hpp"

#include <px4_platform_common/log.h>

ModeRegister::ModeRegister() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// PX4_INFO("Hello Mode Register");
	doRegister();
}

ModeRegister::~ModeRegister()
{
	perf_free(_loop_perf);
}

bool
ModeRegister::init()
{
	if (!_register_ext_component_reply_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

bool ModeRegister::doRegister()
{
//   assert(!_registration->registered());

//   if (!_skip_message_compatibility_check &&
//     !messageCompatibilityCheck(node(), {ALL_PX4_ROS2_MESSAGES}, topicNamespacePrefix()))
//   {
//     return false;
//   }

//   onAboutToRegister();
	register_ext_component_request_s request;
	strcpy(request.name, "Internal Mode");
	request.register_arming_check = true;
	request.register_mode = true;
	request.register_mode_executor = true;
	request.enable_replace_internal_mode = true;
	request.replace_internal_mode = true;
	request.activate_mode_immediately = false;
	request.px4_ros2_api_version = 1;
	request.request_id = 123;
	_register_ext_component_request_pub.publish(request);

//   _health_and_arming_checks.overrideRegistration(_registration);
//   const RegistrationSettings settings = getRegistrationSettings();
	bool ret = true;
//    = _registration->doRegister(settings);

//   if (ret) {
//     if (!onRegistered()) {
//       ret = false;
//     }
//   }

	return ret;
}

void
ModeRegister::Run()
{
	if (should_exit()) {
		_register_ext_component_reply_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	PX4_INFO("Fuuuuuuuuuck");

	if (!_requested) {
		PX4_INFO("Do Register");
		doRegister();
		_requested = true;
	}

	register_ext_component_reply_s reply;

	if (_register_ext_component_reply_sub.update(&reply)) {
		PX4_INFO("Replied!");
		PX4_INFO("  - success: %f", double(reply.success));
	}

	perf_end(_loop_perf);
}

int ModeRegister::task_spawn(int argc, char *argv[])
{

	ModeRegister *instance = new ModeRegister();

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

int ModeRegister::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ModeRegister::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements an internal mode registration.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mode_register", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mode_register_main(int argc, char *argv[])
{
	return ModeRegister::main(argc, argv);
}
