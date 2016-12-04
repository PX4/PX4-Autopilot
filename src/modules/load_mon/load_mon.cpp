/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file load_mon.cpp
 *
 * @author Jonathan Challinger <jonathan@3drobotics.com>
 * @author Julian Oes <julian@oes.ch
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_defines.h>

#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/cpuload.h>
#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/low_stack.h>

extern struct system_load_s system_load;


namespace load_mon
{

extern "C" __EXPORT int load_mon_main(int argc, char *argv[]);

// Run it at 1 Hz.
const unsigned LOAD_MON_INTERVAL_US = 1000000;

class LoadMon
{
public:
	LoadMon();
	~LoadMon();

	/* Start the load monitoring
	 *
	 * @return 0 if successfull, -1 on error. */
	int start();

	/* Stop the load monitoring */
	void stop();

	/* Trampoline for the work queue. */
	static void cycle_trampoline(void *arg);

	bool isRunning() { return _taskIsRunning; }

	void printStatus();

private:
	/* Do a compute and schedule the next cycle. */
	void _cycle();

	/* Do a calculation of the CPU load and publish it. */
	void _compute();

	/* Calculate the memory usage */
	float _ram_used();

#ifdef __PX4_NUTTX
	/* Calculate stack usage */
	void _stack_usage();

	struct low_stack_s _low_stack;
	int _stack_task_index;
	orb_advert_t _low_stack_pub;
#endif

	bool _taskShouldExit;
	bool _taskIsRunning;
	struct work_s _work;

	struct cpuload_s _cpuload;
	orb_advert_t _cpuload_pub;
	hrt_abstime _last_idle_time;
	perf_counter_t _stack_perf;
};

LoadMon::LoadMon() :
#ifdef __PX4_NUTTX
	_low_stack {},
	_stack_task_index(0),
	_low_stack_pub(nullptr),
#endif
	_taskShouldExit(false),
	_taskIsRunning(false),
	_work{},
	_cpuload{},
	_cpuload_pub(nullptr),
	_last_idle_time(0),
	_stack_perf(perf_alloc(PC_ELAPSED, "stack_check"))
{}

LoadMon::~LoadMon()
{
	work_cancel(LPWORK, &_work);
	perf_free(_stack_perf);
	_taskIsRunning = false;
}

int LoadMon::start()
{
	/* Schedule a cycle to start things. */
	return work_queue(LPWORK, &_work, (worker_t)&LoadMon::cycle_trampoline, this, 0);
}

void LoadMon::stop()
{
	_taskShouldExit = true;
}

void
LoadMon::cycle_trampoline(void *arg)
{
	LoadMon *dev = reinterpret_cast<LoadMon *>(arg);

	dev->_cycle();
}

void LoadMon::_cycle()
{
	_taskIsRunning = true;

	_compute();

	if (!_taskShouldExit) {
		work_queue(LPWORK, &_work, (worker_t)&LoadMon::cycle_trampoline, this,
			   USEC2TICK(LOAD_MON_INTERVAL_US));
	}
}

void LoadMon::_compute()
{
	if (_last_idle_time == 0) {
		/* Just get the time in the first iteration */
		_last_idle_time = system_load.tasks[0].total_runtime;
		return;
	}

	/* compute system load */
	const hrt_abstime interval_idletime = system_load.tasks[0].total_runtime - _last_idle_time;
	_last_idle_time = system_load.tasks[0].total_runtime;

	_cpuload.timestamp = hrt_absolute_time();
	_cpuload.load = 1.0f - (float)interval_idletime / (float)LOAD_MON_INTERVAL_US;
	_cpuload.ram_usage = _ram_used();

#ifdef __PX4_NUTTX
	perf_begin(_stack_perf);
	sched_lock();
	_stack_usage();
	sched_unlock();
	perf_end(_stack_perf);
#endif

	if (_cpuload_pub == nullptr) {
		_cpuload_pub = orb_advertise(ORB_ID(cpuload), &_cpuload);

	} else {
		orb_publish(ORB_ID(cpuload), _cpuload_pub, &_cpuload);
	}
}

float LoadMon::_ram_used()
{
#ifdef __PX4_NUTTX
	struct mallinfo mem;

#ifdef CONFIG_CAN_PASS_STRUCTS
	mem = mallinfo();
#else
	(void)mallinfo(&mem);
#endif

	// mem.arena: total ram (bytes)
	// mem.uordblks: used (bytes)
	// mem.fordblks: free (bytes)
	// mem.mxordblk: largest remaining block (bytes)

	float load = (float)mem.uordblks / mem.arena;

	// Check for corruption of the allocation counters
	if ((mem.arena > CONFIG_DRAM_SIZE) || (mem.fordblks > CONFIG_DRAM_SIZE)) {
		load = 1.0f;
	}

	return load;
#else
	return 0.0f;
#endif
}

#ifdef __PX4_NUTTX
void LoadMon::_stack_usage()
{
	int task_index = 0;

	/* Scan maximum 3 tasks per cycle to reduce load. */
	for (int i = _stack_task_index; i < _stack_task_index + 3; i++) {
		task_index = i % CONFIG_MAX_TASKS;

		if (system_load.tasks[task_index].valid && system_load.tasks[task_index].tcb->pid > 0) {
			unsigned stack_size = (uintptr_t)system_load.tasks[task_index].tcb->adj_stack_ptr -
					      (uintptr_t)system_load.tasks[task_index].tcb->stack_alloc_ptr;
			unsigned stack_free = 0;
			uint8_t *stack_sweeper = (uint8_t *)system_load.tasks[task_index].tcb->stack_alloc_ptr;

			while (stack_free < stack_size) {
				if (*stack_sweeper++ != 0xff) {
					break;
				}

				stack_free++;
			}

			/*
			 * Found task low on stack, report and exit. Continue here in next cycle.
			 */
			if (stack_free < 300) {
				strncpy((char *)_low_stack.task_name, system_load.tasks[task_index].tcb->name, low_stack_s::MAX_REPORT_TASK_NAME_LEN);
				_low_stack.stack_free = stack_free;

				if (_low_stack_pub == nullptr) {
					_low_stack_pub = orb_advertise(ORB_ID(low_stack), &_low_stack);

				} else {
					orb_publish(ORB_ID(low_stack), _low_stack_pub, &_low_stack);
				}

				break;
			}

		} else {
			/* No task here, check one more task in same cycle. */
			_stack_task_index++;
		}
	}

	/* Continue after last checked task next cycle. */
	_stack_task_index = task_index + 1;
}
#endif

void LoadMon::printStatus()
{
	perf_print_counter(_stack_perf);
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PX4_INFO("usage: load_mon {start|stop|status}");
}


static LoadMon *load_mon = nullptr;

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int load_mon_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (load_mon != nullptr && load_mon->isRunning()) {
			PX4_WARN("already running");
			/* this is not an error */
			return 0;
		}

		load_mon = new LoadMon();

		// Check if alloc worked.
		if (load_mon == nullptr) {
			PX4_ERR("alloc failed");
			return -1;
		}

		int ret = load_mon->start();

		if (ret != 0) {
			PX4_ERR("start failed");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {

		if (load_mon == nullptr || load_mon->isRunning()) {
			PX4_WARN("not running");
			/* this is not an error */
			return 0;
		}

		load_mon->stop();

		// Wait for task to die
		int i = 0;

		do {
			/* wait up to 3s */
			usleep(100000);

		} while (load_mon->isRunning() && ++i < 30);

		delete load_mon;
		load_mon = nullptr;

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (load_mon != nullptr && load_mon->isRunning()) {
			PX4_INFO("running");
			load_mon->printStatus();

		} else {
			PX4_INFO("not running\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

} // namespace load_mon
