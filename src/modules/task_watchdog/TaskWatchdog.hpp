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

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/printload.h>
#include <px4_platform/cpuload.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/sem.h>
#include <nuttx/sched.h>
#include <board_config.h>

#define LOG_PATH_BASE       CONFIG_BOARD_ROOT_PATH "/task_watchdog"
#define LOG_WDG_NAME_FMT    "wdg_%s.log"
#define LOG_LOAD_NAME_FMT   "load_%s.log"
#define TIME_FMT            "%Y_%m_%d_%H_%M_%S"

using namespace time_literals;

namespace task_watchdog
{

static constexpr unsigned STACK_DUMP_WORDS = 16;
#ifdef BOARD_HAS_RAM_HARDFAULT_DUMP
static constexpr int MAX_DUMP_TASKS = 24; /* RAM-constrained boards: cap to avoid heap exhaustion */
#elif defined(CONFIG_FS_PROCFS_MAX_TASKS)
static constexpr int MAX_DUMP_TASKS = CONFIG_FS_PROCFS_MAX_TASKS;
#else
static constexpr int MAX_DUMP_TASKS = 24;
#endif

struct task_dump_s {
	pid_t pid{0};
	uint8_t state{0};
#if CONFIG_TASK_NAME_SIZE > 0
	char name[CONFIG_TASK_NAME_SIZE + 1] {};
#endif
	uint32_t regs[XCPTCONTEXT_REGS] {};
	uint32_t stack[STACK_DUMP_WORDS] {};
	unsigned stack_words{0};
	bool has_regs{false};
	bool valid{false};
};

/* Shared state between ISR and task */
struct watchdog_shared_s {
	int monitored_task_index{-1};       ///< system_load index of this module's task
	hrt_abstime ready_to_run_timestamp{0};
	uint8_t last_state{TSTATE_TASK_INVALID};
	int original_priority{0};
	hrt_abstime trigger_time{0};
	px4::atomic_bool triggered{false};
	bool manual_trigger{false};    ///< set from custom_command, checked by ISR
	px4_sem_t *sem{nullptr};       ///< posted by ISR to wake the task immediately
};

typedef enum {
	LOG_WDG,
	LOG_LOAD
} log_type_t;

class TaskWatchdog : public ModuleBase
{
public:
	static Descriptor desc;

	TaskWatchdog();
	~TaskWatchdog() override;

	static int task_spawn(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	void start();

	/* Capture all tasks and output register+stack dump. Writes to file on storage boards,
	 * streams via PX4_ERR on RAM-dump boards. Called at boosted priority. */
	void capture_and_write_dump();

	/* Output cpuload snapshot. Writes to file on storage boards, streams via PX4_ERR on RAM-dump boards. */
	void write_cpuload_file();

	int format_file_path(log_type_t type, char *buf, size_t bufsz);

	/* HRT ISR callback which monitors whether this task is being starved. */
	static void isr_callback(void *arg);

	hrt_call _hrt_call{};
	watchdog_shared_s _shared{};
	px4_sem_t _sem;
	task_dump_s _dumps[MAX_DUMP_TASKS] {};
	print_load_s _load{};

	bool _cpuload_pending{false};

	static constexpr hrt_abstime TRIGGER_THRESHOLD = 2_s; ///< time in ready to run to trigger watchdog
	static constexpr hrt_abstime CPULOAD_ACCUMULATE_TIME = 300_ms;   ///< how long to accumulate cpuload data
};

} // namespace task_watchdog
