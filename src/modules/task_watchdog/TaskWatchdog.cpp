/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "TaskWatchdog.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <version/version.h>

#include <fcntl.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <sys/stat.h>

#ifndef CONFIG_BUILD_FLAT
# error Task Watchdog requires flat build, please enable it to use this module
#endif

using namespace time_literals;

namespace task_watchdog
{

ModuleBase::Descriptor TaskWatchdog::desc{task_spawn, custom_command, print_usage};

TaskWatchdog::TaskWatchdog() :
	ModuleParams(nullptr)
{
}

TaskWatchdog::~TaskWatchdog()
{
	hrt_cancel(&_hrt_call);
	px4_sem_destroy(&_sem);
}

int TaskWatchdog::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		TaskWatchdog *instance = new TaskWatchdog();

		if (instance == nullptr)
		{
			PX4_ERR("alloc failed");
		}

		return instance;
	}, argc, argv);
}

int TaskWatchdog::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("task_watchdog",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_LOG_WRITER,
					  PX4_STACK_ADJUSTED(3250),
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		PX4_ERR("task start failed: %d", errno);
		return -errno;
	}

	return 0;
}

void TaskWatchdog::start()
{
	px4_sem_init(&_sem, 0, 0);
	px4_sem_setprotocol(&_sem, SEM_PRIO_NONE);
	_shared.sem = &_sem;

	// Find our own task in the system_load table
	const pid_t my_pid = getpid();

	sched_lock();

	for (int i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
		if (system_load.tasks[i].valid && system_load.tasks[i].tcb->pid == my_pid) {
			_shared.monitored_task_index = i;
			break;
		}
	}

	sched_unlock();

	if (_shared.monitored_task_index < 0) {
		PX4_ERR("could not find own task in system_load");
	}

	_shared.ready_to_run_timestamp = hrt_absolute_time();

	// Start HRT ISR
	hrt_call_every(&_hrt_call, 400_ms, 400_ms, isr_callback, &_shared);
}

void TaskWatchdog::isr_callback(void *arg)
{
	watchdog_shared_s *shared = static_cast<watchdog_shared_s *>(arg);

	if (!system_load.initialized || shared->monitored_task_index < 0) {
		return;
	}

	const system_load_taskinfo_s &watchdog_task = system_load.tasks[shared->monitored_task_index];
	const hrt_abstime now = hrt_absolute_time();

	if (!watchdog_task.valid) {
		shared->monitored_task_index = -1;
		return;
	}

	// Already triggered — wait for the task side to process
	if (shared->triggered.load()) {
		return;
	}

	// Already in post-trigger priority restore phase
	if (shared->trigger_time != 0 && now > shared->trigger_time + 1500_ms) {
		// Restore our original priority
		sched_param param{};
		param.sched_priority = shared->original_priority;
		sched_setparam(watchdog_task.tcb->pid, &param);

		/* Only trigger once as the system may be in an unstable state */
		shared->monitored_task_index = -1;
		return;
	}

	// If the task ran recently, update the timestamp
	if (watchdog_task.curr_start_time > shared->ready_to_run_timestamp) {
		shared->ready_to_run_timestamp = watchdog_task.curr_start_time;
	}

	// Reset timestamp on state transitions into/out of READYTORUN
	uint8_t current_state = watchdog_task.tcb->task_state;

	if (current_state != TSTATE_TASK_READYTORUN ||
	    (shared->last_state != TSTATE_TASK_READYTORUN && current_state == TSTATE_TASK_READYTORUN)) {
		shared->ready_to_run_timestamp = now;
	}

	shared->last_state = current_state;

	// Check if we've been starved for too long
	if (!shared->manual_trigger && now <= shared->ready_to_run_timestamp + TRIGGER_THRESHOLD) {
		// Wake the task so it runs briefly, keeping curr_start_time fresh
		px4_sem_post(shared->sem);
		return;
	}

	// --- TRIGGERED ---

	shared->manual_trigger = false;

	// Boost our own priority so we actually get to run and write the dump
	sched_param param{};
	sched_getparam(watchdog_task.tcb->pid, &param);
	shared->original_priority = param.sched_priority;
	param.sched_priority = SCHED_PRIORITY_LOG_WATCHDOG;
	sched_setparam(watchdog_task.tcb->pid, &param);

	shared->trigger_time = now;
	shared->triggered.store(true);
	px4_sem_post(shared->sem);
}

void TaskWatchdog::run()
{
	start();

	while (!should_exit()) {

		if (_cpuload_pending) {
			_cpuload_pending = false;
			cpuload_monitor_stop();
			write_cpuload_file();
		}

		if (_shared.triggered.load()) {
			_shared.triggered.store(false);

			PX4_ERR("task starvation detected!");
			capture_and_write_dump();

			cpuload_monitor_start();
			init_print_load(&_load);
			_cpuload_pending = true;
		}

		if (_cpuload_pending) {
			px4_usleep(CPULOAD_ACCUMULATE_TIME);

		} else {
			// Drain any extra posts that accumulated while the task was asleep
			int val;
			px4_sem_getvalue(&_sem, &val);

			while (val > 0) {
				px4_sem_trywait(&_sem);
				px4_sem_getvalue(&_sem, &val);
			}

			while (px4_sem_wait(&_sem) != 0) {}
		}
	}

	hrt_cancel(&_hrt_call);
}

void TaskWatchdog::capture_and_write_dump()
{
	mkdir(LOG_PATH_BASE, S_IRWXU | S_IRWXG | S_IRWXO);

	char path[64];
	int ret = format_file_path(LOG_WDG, path, sizeof(path));

	if (ret != PX4_OK) {
		PX4_ERR("Could not create watchdog dump file: %d", ret);
		return;
	}

	int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);

	if (fd < 0) {
		PX4_ERR("failed to create %s: %d", path, errno);
		return;
	}

	dprintf(fd, "Task Watchdog Dump\n");
	dprintf(fd, "FW git-hash: %s\n", px4_firmware_version_string());
	dprintf(fd, "Build datetime: %s %s\n", __DATE__, __TIME__);
	dprintf(fd, "Build url: %s \n", px4_build_uri());

	const hrt_abstime now = hrt_absolute_time();

	/* Loop 1: capture under sched_lock */
	sched_lock();

	for (int i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
		const system_load_taskinfo_s &task = system_load.tasks[i];

		if (!task.valid || !task.tcb || task.tcb->pid == 0) {
			continue;
		}

		task_dump_s &d = _dumps[i];
		d.pid = task.tcb->pid;
		d.state = task.tcb->task_state;

#if CONFIG_TASK_NAME_SIZE > 0
		strncpy(d.name, task.tcb->name, CONFIG_TASK_NAME_SIZE);
		d.name[CONFIG_TASK_NAME_SIZE] = '\0';
#endif

		if (task.tcb->xcp.regs && task.tcb->task_state != TSTATE_TASK_RUNNING) {
			memcpy(d.regs, task.tcb->xcp.regs, sizeof(d.regs));
			d.has_regs = true;

			const uintptr_t sp = d.regs[REG_R13];
			const uintptr_t stack_top = (uintptr_t)task.tcb->stack_base_ptr
						    + task.tcb->adj_stack_size;

			if (sp >= (uintptr_t)task.tcb->stack_base_ptr && sp < stack_top) {
				unsigned words_avail = (stack_top - sp) / sizeof(uint32_t);
				d.stack_words = (words_avail < STACK_DUMP_WORDS) ? words_avail : STACK_DUMP_WORDS;
				memcpy(d.stack, (const void *)sp, d.stack_words * sizeof(uint32_t));
			}
		}

		d.valid = true;
	}

	sched_unlock();

	/* Loop 2: write to file — no locks held */
	for (int i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
		const task_dump_s &d = _dumps[i];

		if (!d.valid) {
			continue;
		}

		dprintf(fd, "T:%" PRIu64 " %s(%d) state:%d",
			now,
#if CONFIG_TASK_NAME_SIZE > 0
			d.name,
#else
			"?",
#endif
			(int)d.pid,
			(int)d.state);

		if (d.has_regs) {
			dprintf(fd, "\n r0:%08" PRIx32 " r1:%08" PRIx32 "  r2:%08" PRIx32 "  r3:%08" PRIx32
				"  r4:%08" PRIx32 "  r5:%08" PRIx32 " r6:%08" PRIx32 " r7:%08" PRIx32 "\n",
				d.regs[REG_R0],  d.regs[REG_R1],
				d.regs[REG_R2],  d.regs[REG_R3],
				d.regs[REG_R4],  d.regs[REG_R5],
				d.regs[REG_R6],  d.regs[REG_R7]);
			dprintf(fd, " r8:%08" PRIx32 " r9:%08" PRIx32 " r10:%08" PRIx32 " r11:%08" PRIx32
				" r12:%08" PRIx32 "  sp:%08" PRIx32 " lr:%08" PRIx32 " pc:%08" PRIx32 "\n",
				d.regs[REG_R8],  d.regs[REG_R9],
				d.regs[REG_R10], d.regs[REG_R11],
				d.regs[REG_R12], d.regs[REG_R13],
				d.regs[REG_R14], d.regs[REG_R15]);
#ifdef CONFIG_ARMV7M_USEBASEPRI
			dprintf(fd, " xpsr:%08" PRIx32 " basepri:%08" PRIx32 "\n",
				d.regs[REG_XPSR], d.regs[REG_BASEPRI]);
#else
			dprintf(fd, " xpsr:%08" PRIx32 " primask:%08" PRIx32 "\n",
				d.regs[REG_XPSR], d.regs[REG_PRIMASK]);
#endif

			if (d.stack_words > 0) {
				dprintf(fd, "STK(%u@%08" PRIx32 "):", d.stack_words, d.regs[REG_R13]);

				for (unsigned j = 0; j < d.stack_words; j++) {
					if (j > 0 && (j % 8) == 0) {
						dprintf(fd, "\n");
					}

					dprintf(fd, " %08" PRIx32, d.stack[j]);
				}

				dprintf(fd, "\n");
			}

		} else {
			dprintf(fd, " [no saved regs]\n");
		}
	}

	fsync(fd);
	close(fd);
	PX4_INFO("dump written to %s", path);
}

void TaskWatchdog::write_cpuload_file()
{
	mkdir(LOG_PATH_BASE, S_IRWXU | S_IRWXG | S_IRWXO);

	char path[64];
	int ret = format_file_path(LOG_LOAD, path, sizeof(path));

	if (ret != PX4_OK) {
		PX4_ERR("Could not create load dump file: %d", ret);
		return;
	}

	int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);

	if (fd < 0) {
		PX4_ERR("failed to create %s: %d", path, errno);
		return;
	}

	print_load(fd, &_load);

	fsync(fd);
	close(fd);
	PX4_INFO("cpuload written to %s", path);
}

int TaskWatchdog::format_file_path(log_type_t type, char *buf, size_t bufsz)
{
	if (!buf || bufsz == 0) {
		return -EINVAL;
	}

	struct timespec ts;

	struct tm tm;

	char timebuf[32];

	if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
		return -errno;
	}

	if (!localtime_r(&ts.tv_sec, &tm)) {
		return -EINVAL;
	}

	if (strftime(timebuf, sizeof(timebuf), TIME_FMT, &tm) == 0) {
		return -EINVAL;
	}

	const char *fmt = (type == LOG_WDG) ? LOG_WDG_NAME_FMT : LOG_LOAD_NAME_FMT;

	int n = snprintf(buf, bufsz, "%s/", LOG_PATH_BASE);

	if (n < 0 || (size_t)n >= bufsz) {
		return -ENAMETOOLONG;
	}

	int m = snprintf(buf + n, bufsz - n, fmt, timebuf);

	if (m < 0 || (size_t)m >= bufsz - n) {
		return -ENAMETOOLONG;
	}

	return PX4_OK;
}

int TaskWatchdog::custom_command(int argc, char *argv[])
{
	if (!is_running(desc)) {
		print_usage("task_watchdog not running");
		return 1;
	}

	if (!strcmp(argv[0], "trigger")) {
		get_instance<TaskWatchdog>(desc)->_shared.manual_trigger = true;
		PX4_INFO("manual watchdog trigger requested");
		return 0;
	}

	return print_usage("unknown command");
}

int TaskWatchdog::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Detects when a higher-priority task starves the system by running too long.
When starvation is detected, dumps the offending task's registers and stack,
and saves a cpuload snapshot.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("task_watchdog", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("trigger", "Manually trigger the watchdog");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace task_watchdog

extern "C" __EXPORT int task_watchdog_main(int argc, char *argv[])
{
	return ModuleBase::main(task_watchdog::TaskWatchdog::desc, argc, argv);
}
