/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#include "LoadMon.hpp"

using namespace time_literals;

namespace load_mon
{

LoadMon::LoadMon() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

LoadMon::~LoadMon()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int LoadMon::task_spawn(int argc, char *argv[])
{
	LoadMon *obj = new LoadMon();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void LoadMon::start()
{
	ScheduleOnInterval(500_ms); // 2 Hz
}

void LoadMon::Run()
{
	perf_begin(_cycle_perf);

	cpuload();

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	perf_end(_cycle_perf);
}

void LoadMon::cpuload()
{
	tms spent_time_stamp_struct;
	clock_t total_time_stamp = times(&spent_time_stamp_struct);
	clock_t spent_time_stamp = spent_time_stamp_struct.tms_utime + spent_time_stamp_struct.tms_stime;

	if (_last_total_time_stamp == 0 || _last_spent_time_stamp == 0) {
		// Just get the time in the first iteration */
		_last_total_time_stamp = total_time_stamp;
		_last_spent_time_stamp = spent_time_stamp;
		return;
	}

	// compute system load
	const float interval = total_time_stamp - _last_total_time_stamp;
	const float interval_spent_time = spent_time_stamp - _last_spent_time_stamp;

	// get ram usage
	struct mallinfo mem = mallinfo();
	float ram_usage = (float)mem.uordblks / mem.arena;

	cpuload_s cpuload{};
	cpuload.load = interval_spent_time / interval;
	cpuload.ram_usage = ram_usage;
	cpuload.timestamp = hrt_absolute_time();

	_cpuload_pub.publish(cpuload);

	// store for next iteration
	_last_total_time_stamp = total_time_stamp;
	_last_spent_time_stamp = spent_time_stamp;
}

int LoadMon::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the low priority work queue to calculate the CPU load and RAM
usage and publish the `cpuload` topic.

This module runs on Linux platform.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("load_mon_linux", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int load_mon_linux_main(int argc, char *argv[])
{
	return LoadMon::main(argc, argv);
}

} // namespace load_mon
