/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.cpp
 * Minimal module example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <string.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/vehicle_acceleration.h>

using namespace time_literals;

class PX4SimpleApp : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	PX4SimpleApp() : WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		// Run() at most every 200 ms (5 Hz), not on every vehicle_acceleration update
		_vehicle_acceleration_sub.set_interval_us(200_ms);
	}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

private:
	void Run() override;

	// Schedules Run() whenever vehicle_acceleration is published
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};

	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
};

ModuleBase::Descriptor PX4SimpleApp::desc{task_spawn, custom_command, print_usage};

void PX4SimpleApp::Run()
{
	if (should_exit()) {
		_vehicle_acceleration_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_acceleration_s accel;

	if (_vehicle_acceleration_sub.update(&accel)) {
		PX4_INFO("Accelerometer: %8.4f %8.4f %8.4f",
			 (double)accel.xyz[0], (double)accel.xyz[1], (double)accel.xyz[2]);

		// Publish the values, MAVLink forwards debug_vect to the ground station as DEBUG_VECT
		debug_vect_s debug_vect{};
		strncpy(debug_vect.name, "accel", sizeof(debug_vect.name));
		debug_vect.x = accel.xyz[0];
		debug_vect.y = accel.xyz[1];
		debug_vect.z = accel.xyz[2];
		debug_vect.timestamp = hrt_absolute_time();
		_debug_vect_pub.publish(debug_vect);
	}
}

int PX4SimpleApp::task_spawn(int argc, char *argv[])
{
	PX4SimpleApp *instance = new PX4SimpleApp();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	PX4_INFO("Hello Sky!");
	return PX4_OK;
}

int PX4SimpleApp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal example module: prints vehicle_acceleration at 5 Hz and publishes it as debug_vect.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("px4_simple_app", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[])
{
	return ModuleBase::main(PX4SimpleApp::desc, argc, argv);
}
