/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file top.c
 * Tool similar to UNIX top command
 * @see http://en.wikipedia.org/wiki/Top_unix
 * 
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

#include <systemlib/cpuload.h>
#include <drivers/drv_hrt.h>

#define CL "\033[K" // clear line

/**
 * Start the top application.
 */
__EXPORT int top_main(void);

extern struct system_load_s system_load;

static const char *
tstate_name(const tstate_t s)
{
	switch (s) {
	case TSTATE_TASK_INVALID:    return "init";

	case TSTATE_TASK_PENDING:    return "PEND";
	case TSTATE_TASK_READYTORUN: return "READY";
	case TSTATE_TASK_RUNNING:    return "RUN";

	case TSTATE_TASK_INACTIVE:   return "inact";
	case TSTATE_WAIT_SEM:        return "w:sem";
#ifndef CONFIG_DISABLE_SIGNALS
	case TSTATE_WAIT_SIG:        return "w:sig";
#endif
#ifndef CONFIG_DISABLE_MQUEUE
	case TSTATE_WAIT_MQNOTEMPTY: return "w:mqe";
	case TSTATE_WAIT_MQNOTFULL:  return "w:mqf";
#endif
#ifdef CONFIG_PAGING
	case TSTATE_WAIT_PAGEFILL:   return "w:pgf";
#endif

	default:
		return "ERROR";
	}
}

int
top_main(void)
{
	uint64_t total_user_time = 0;

	int running_count = 0;
	int blocked_count = 0;

	uint64_t new_time = hrt_absolute_time();
	uint64_t interval_start_time = new_time;

	uint64_t last_times[CONFIG_MAX_TASKS];
	float curr_loads[CONFIG_MAX_TASKS];

	for (int t = 0; t < CONFIG_MAX_TASKS; t++)
		last_times[t] = 0;

	float interval_time_ms_inv = 0.f;

	/* clear screen */
	printf("\033[2J");

	for (;;) {
		int   i;
		uint64_t curr_time_us;
		uint64_t idle_time_us;

		curr_time_us = hrt_absolute_time();
		idle_time_us = system_load.tasks[0].total_runtime;

		if (new_time > interval_start_time)
			interval_time_ms_inv = 1.f / ((float)((new_time - interval_start_time) / 1000));

		running_count = 0;
		blocked_count = 0;
		total_user_time = 0;

		for (i = 0; i < CONFIG_MAX_TASKS; i++) {
			uint64_t interval_runtime;

			if (system_load.tasks[i].valid) {
				switch (system_load.tasks[i].tcb->task_state) {
				case TSTATE_TASK_PENDING:
				case TSTATE_TASK_READYTORUN:
				case TSTATE_TASK_RUNNING:
					running_count++;
					break;

				case TSTATE_TASK_INVALID:
				case TSTATE_TASK_INACTIVE:
				case TSTATE_WAIT_SEM:
#ifndef CONFIG_DISABLE_SIGNALS
				case TSTATE_WAIT_SIG:
#endif
#ifndef CONFIG_DISABLE_MQUEUE
				case TSTATE_WAIT_MQNOTEMPTY:
				case TSTATE_WAIT_MQNOTFULL:
#endif
#ifdef CONFIG_PAGING
				case TSTATE_WAIT_PAGEFILL:
#endif
					blocked_count++;
					break;
				}
			}

			interval_runtime = (system_load.tasks[i].valid && last_times[i] > 0 &&
								system_load.tasks[i].total_runtime > last_times[i])
				? (system_load.tasks[i].total_runtime - last_times[i]) / 1000
				: 0;

			last_times[i] = system_load.tasks[i].total_runtime;

			if (system_load.tasks[i].valid && (new_time > interval_start_time)) {
				curr_loads[i] = interval_runtime * interval_time_ms_inv;

				if (i > 0)
					total_user_time += interval_runtime;
			} else
				curr_loads[i] = 0;
		}

		for (i = 0; i < CONFIG_MAX_TASKS; i++) {
			if (system_load.tasks[i].valid && (new_time > interval_start_time)) {
				if (system_load.tasks[i].tcb->pid == 0) {
					float idle;
					float task_load;
					float sched_load;

					idle = curr_loads[0];
					task_load = (float)(total_user_time) * interval_time_ms_inv;

					/* this can happen if one tasks total runtime was not computed
					   correctly by the scheduler instrumentation TODO */
					if (task_load > (1.f - idle))
						task_load = (1.f - idle);

					sched_load = 1.f - idle - task_load;

					/* print system information */
					printf("\033[H"); /* move cursor home and clear screen */
					printf(CL "Processes: %d total, %d running, %d sleeping\n",
						   system_load.total_count,
						   running_count,
						   blocked_count);
					printf(CL "CPU usage: %.2f%% tasks, %.2f%% sched, %.2f%% idle\n",
						   (double)(task_load * 100.f),
						   (double)(sched_load * 100.f),
						   (double)(idle * 100.f));
					printf(CL "Uptime: %.3fs total, %.3fs idle\n\n",
						   (double)curr_time_us / 1000000.d,
						   (double)idle_time_us / 1000000.d);

					/* header for task list */
					printf(CL "%4s %*-s %8s %6s %11s %10s %-6s\n",
						   "PID",
						   CONFIG_TASK_NAME_SIZE, "COMMAND",
						   "CPU(ms)",
						   "CPU(%)",
						   "USED/STACK",
						   "PRIO(BASE)",
#if CONFIG_RR_INTERVAL > 0
						   "TSLICE"
#else
						   "STATE"
#endif
						   );
				}

				unsigned stack_size = (uintptr_t)system_load.tasks[i].tcb->adj_stack_ptr -
					(uintptr_t)system_load.tasks[i].tcb->stack_alloc_ptr;
				unsigned stack_free = 0;
				uint8_t *stack_sweeper = (uint8_t *)system_load.tasks[i].tcb->stack_alloc_ptr;

				while (stack_free < stack_size) {
					if (*stack_sweeper++ != 0xff)
						break;

					stack_free++;
				}

				printf(CL "%4d %*-s %8lld %2d.%03d %5u/%5u %3u (%3u) ",
					   system_load.tasks[i].tcb->pid,
					   CONFIG_TASK_NAME_SIZE, system_load.tasks[i].tcb->name,
					   (system_load.tasks[i].total_runtime / 1000),
					   (int)(curr_loads[i] * 100),
					   (int)(curr_loads[i] * 100000.0f - (int)(curr_loads[i] * 1000.0f) * 100),
					   stack_size - stack_free,
					   stack_size,
					   system_load.tasks[i].tcb->sched_priority,
					   system_load.tasks[i].tcb->base_priority);

#if CONFIG_RR_INTERVAL > 0
				/* print scheduling info with RR time slice */
				printf(" %6d\n", system_load.tasks[i].tcb->timeslice);
#else
				// print task state instead
				printf(" %-6s\n", tstate_name(system_load.tasks[i].tcb->task_state));
#endif
			}
		}

		interval_start_time = new_time;

		/* Sleep 200 ms waiting for user input five times ~ 1s */
		for (int k = 0; k < 5; k++) {
			char c;

			struct pollfd fds;
			int ret;
			fds.fd = 0; /* stdin */
			fds.events = POLLIN;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				read(0, &c, 1);

				switch (c) {
				case 0x03: // ctrl-c
				case 0x1b: // esc
				case 'c':
				case 'q':
					return OK;
					/* not reached */
				}
			}

			usleep(200000);
		}

		new_time = hrt_absolute_time();
	}

	return OK;
}
