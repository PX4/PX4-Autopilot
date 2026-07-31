/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/events.h>
#include "navputer.hpp"

using namespace time_literals;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t navputer_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<Navputer *> _instance {};

Navputer::Navputer(const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config)
{
	AdvertiseTopics();
}

Navputer::~Navputer()
{

}

void Navputer::AdvertiseTopics()
{

}


int Navputer::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

	Navputer *_inst = new Navputer(px4::wq_configurations::INS0, replay_mode);

	if (_inst) {
		_instance.store(_inst);
		_inst->ScheduleNow();
		success = true;
	}

	return success ? PX4_OK : PX4_ERROR;
}

int Navputer::print_status(bool verbose)
{
	PX4_INFO_RAW("Navputer is OK\n");

	return 0;
}

void Navputer::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	if (!_callback_registered) {
		_callback_registered = _sensor_combined_sub.registerCallback();

		if (!_callback_registered) {
			ScheduleDelayed(10_ms);
			return;
		}
	}
}

int Navputer::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Navputer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	return 0;
}

extern "C" __EXPORT int navputer_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return Navputer::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		Navputer::lock_module();

		ret = Navputer::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		Navputer::unlock_module();
		return ret;
	} else if (strcmp(argv[1], "status") == 0) {
		if (Navputer::trylock_module()) {
			bool verbose_status = false;

			_instance.load()->print_status(verbose_status);

			Navputer::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		Navputer::lock_module();

		// otherwise stop everything
		bool was_running = false;

		Navputer *inst = _instance.load();

		if (inst) {
			PX4_INFO("stopping Navputer");
			was_running = true;
			inst->request_stop();
			px4_usleep(20000); // 20 ms
			delete inst;
			_instance.store(nullptr);
		}

		if (!was_running) {
			PX4_WARN("not running");
		}

		Navputer::unlock_module();
		return PX4_OK;
	}

	Navputer::lock_module(); // Lock here, as the method could access _instance.
	int ret = Navputer::custom_command(argc - 1, argv + 1);
	Navputer::unlock_module();

	return ret;
}
