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

#include "TimePersistor.hpp"

#include <errno.h>
#include <px4_platform_common/log.h>

TimePersistor::TimePersistor() :
	ModuleParams(nullptr)
{
}

TimePersistor::~TimePersistor()
{
	fclose(_file);
}

int TimePersistor::init()
{
	if ((_file = fopen(TIME_FILE_PATH, "r+")) == nullptr) {
		_file = fopen(TIME_FILE_PATH, "w+");
	}

	if (!_file) {
		PX4_ERR("error opening the time file");
		return PX4_ERROR;
	}

	time_t t;

	if (read_time(&t) == PX4_OK) {
		struct timespec ts;
		ts.tv_sec = t;
		ts.tv_nsec = 0;

		px4_clock_settime(CLOCK_REALTIME, &ts);
		return PX4_OK;
	}

	return PX4_OK;
}

void TimePersistor::run()
{
	while (!should_exit()) {
		struct timespec ts;
		px4_clock_gettime(CLOCK_REALTIME, &ts);

		if (write_time(ts.tv_sec) != PX4_OK) {
			PX4_ERR("error writing RTC to time file");
			exit_and_cleanup();
		}

		px4_usleep(1_s);
	}
}

int TimePersistor::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("time_persistor",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

TimePersistor *TimePersistor::instantiate(int argc, char *argv[])
{
	TimePersistor *instance = new TimePersistor();

	if (instance) {
		int status = instance->init();

		if (status != PX4_OK) {
			delete instance;
			return nullptr;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int TimePersistor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TimePersistor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Writes the RTC time cyclically to a file and reloads this value on startup.
This allows monotonic time on systems that only have a software RTC (that is not battery powered).
Explicitly setting the time backwards (e.g. via system_time) is still possible.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("time_persistor", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int time_persistor_main(int argc, char *argv[])
{
	return TimePersistor::main(argc, argv);
}

int TimePersistor::read_time(time_t *time)
{
	int status = PX4_OK;

	status |= fseek(_file, 0, SEEK_SET);
	status |= fread(time, sizeof(time_t), 1, _file) != 1;

	return status;
}

int TimePersistor::write_time(const time_t time)
{
	int status = PX4_OK;

	status |= fseek(_file, 0, SEEK_SET);
	status |= fwrite(&time, sizeof(time_t), 1, _file) != 1;
	status |= fflush(_file);
	status |= fsync(fileno(_file));

	return status;
}
