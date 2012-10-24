/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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

/**
 * Start the top application.
 */
__EXPORT int top_main(int argc, char *argv[]);

extern struct system_load_s system_load;

bool top_sigusr1_rcvd = false;

int top_main(int argc, char *argv[])
{
	int t;

	uint64_t total_user_time = 0;

	int running_count = 0;
	int blocked_count = 0;

	uint64_t new_time = hrt_absolute_time();
	uint64_t interval_start_time = new_time;

	uint64_t last_times[CONFIG_MAX_TASKS];
	float curr_loads[CONFIG_MAX_TASKS];

	for (t = 0; t < CONFIG_MAX_TASKS; t++)
		last_times[t] = 0;

	float interval_time_ms_inv = 0.f;

	/* Open console directly to grab CTRL-C signal */
	int console = open("/dev/console", O_NONBLOCK | O_RDONLY | O_NOCTTY);

	while (true)
//	for (t = 0; t < 10; t++)
	{
		int i;

		uint64_t curr_time_ms = (hrt_absolute_time() / 1000LLU);
		unsigned int curr_time_s = curr_time_ms / 1000LLU;

		uint64_t idle_time_total_ms = (system_load.tasks[0].total_runtime / 1000LLU);
		unsigned int idle_time_total_s = idle_time_total_ms / 1000LLU;

		if (new_time > interval_start_time)
			interval_time_ms_inv = 1.f / ((float)((new_time - interval_start_time) / 1000));

		running_count = 0;
		blocked_count = 0;
		total_user_time = 0;

		for (i = 0; i < CONFIG_MAX_TASKS; i++) {
			uint64_t interval_runtime = (system_load.tasks[i].valid && last_times[i] > 0 && system_load.tasks[i].total_runtime > last_times[i]) ? (system_load.tasks[i].total_runtime - last_times[i]) / 1000 : 0;

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
					float idle = curr_loads[0];
					float task_load = (float)(total_user_time) * interval_time_ms_inv;

					if (task_load > (1.f - idle)) task_load = (1.f - idle);	/* this can happen if one tasks total runtime was not computed correctly by the scheduler instrumentation TODO */

					float sched_load = 1.f - idle - task_load;

					/* print system information */
					printf("\033[H"); /* cursor home */
					printf("\033[KProcesses: %d total, %d running, %d sleeping\n", system_load.total_count, running_count, blocked_count);
					printf("\033[KCPU usage: %d.%02d%% tasks, %d.%02d%% sched, %d.%02d%% idle\n", (int)(task_load * 100), (int)((task_load * 10000.0f) - (int)(task_load * 100.0f) * 100), (int)(sched_load * 100), (int)((sched_load * 10000.0f) - (int)(sched_load * 100.0f) * 100), (int)(idle * 100), (int)((idle * 10000.0f) - ((int)(idle * 100)) * 100));
					printf("\033[KUptime: %u.%03u s total, %d.%03d s idle\n\033[K\n", curr_time_s, (unsigned int)(curr_time_ms - curr_time_s * 1000LLU), idle_time_total_s, (int)(idle_time_total_ms - idle_time_total_s * 1000));

					/* 34 chars command name length (32 chars plus two spaces) */
					char header_spaces[CONFIG_TASK_NAME_SIZE + 1];
					memset(header_spaces, ' ', CONFIG_TASK_NAME_SIZE);
					header_spaces[CONFIG_TASK_NAME_SIZE] = '\0';
#if CONFIG_RR_INTERVAL > 0
					printf("\033[KPID\tCOMMAND%s CPU TOTAL \t%%CPU CURR \tSTACK USE\tCURR (BASE) PRIO\tRR SLICE\n", header_spaces);
#else
					printf("\033[KPID\tCOMMAND%s CPU TOTAL \t%%CPU CURR \tSTACK USE\tCURR (BASE) PRIO\n", header_spaces);
#endif

				} else {
					enum tstate_e task_state = (enum tstate_e)system_load.tasks[i].tcb->task_state;

					if (task_state == TSTATE_TASK_PENDING ||
					    task_state == TSTATE_TASK_READYTORUN ||
					    task_state == TSTATE_TASK_RUNNING) {
						running_count++;
					}

					if (task_state == TSTATE_TASK_INACTIVE ||      /* BLOCKED      - Initialized but not yet activated */
					    task_state == TSTATE_WAIT_SEM           /* BLOCKED      - Waiting for a semaphore */
#ifndef CONFIG_DISABLE_SIGNALS
					    || task_state == TSTATE_WAIT_SIG           /* BLOCKED      - Waiting for a signal */
#endif
#ifndef CONFIG_DISABLE_MQUEUE
					    || task_state == TSTATE_WAIT_MQNOTEMPTY     /* BLOCKED      - Waiting for a MQ to become not empty. */
					    || task_state == TSTATE_WAIT_MQNOTFULL      /* BLOCKED      - Waiting for a MQ to become not full. */
#endif
#ifdef CONFIG_PAGING
					    || task_state == TSTATE_WAIT_PAGEFILL       /* BLOCKED      - Waiting for page fill */
#endif
					   ) {
						blocked_count++;
					}

					char spaces[CONFIG_TASK_NAME_SIZE + 2];

					/* count name len */
					int namelen = 0;

					while (namelen < CONFIG_TASK_NAME_SIZE) {
						if (system_load.tasks[i].tcb->name[namelen] == '\0') break;

						namelen++;
					}

					int s = 0;

					for (s = 0; s < CONFIG_TASK_NAME_SIZE + 2 - namelen; s++) {
						spaces[s] = ' ';
					}

					spaces[s] = '\0';

					char *runtime_spaces = "  ";

					if ((system_load.tasks[i].total_runtime / 1000) < 99) {
						runtime_spaces = "";
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

					printf("\033[K % 2d\t%s%s % 8lld ms%s  \t % 2d.%03d \t % 4u / % 4u",
					       (int)system_load.tasks[i].tcb->pid,
					       system_load.tasks[i].tcb->name,
					       spaces,
					       (system_load.tasks[i].total_runtime / 1000),
					       runtime_spaces,
					       (int)(curr_loads[i] * 100),
					       (int)(curr_loads[i] * 100000.0f - (int)(curr_loads[i] * 1000.0f) * 100),
					       stack_size - stack_free,
					       stack_size);
					/* Print scheduling info with RR time slice */
#if CONFIG_RR_INTERVAL > 0
					printf("\t%d\t(%d)\t\t%d\n", (int)system_load.tasks[i].tcb->sched_priority, (int)system_load.tasks[i].tcb->base_priority, (int)system_load.tasks[i].tcb->timeslice);
#else
					/* Print scheduling info without time slice*/
					printf("\t%d (%d)\n", (int)system_load.tasks[i].tcb->sched_priority, (int)system_load.tasks[i].tcb->base_priority);
#endif
				}
			}
		}

		printf("\033[K[ Hit Ctrl-C to quit. ]\n\033[J");
		fflush(stdout);

		interval_start_time = new_time;

		char c;

		/* Sleep 200 ms waiting for user input four times */
		/* XXX use poll ... */
		for (int k = 0; k < 4; k++) {
			if (read(console, &c, 1) == 1) {
				if (c == 0x03 || c == 0x63) {
					printf("Abort\n");
					close(console);
					return OK;
				}
			}

			usleep(200000);
		}

		new_time = hrt_absolute_time();
	}

	close(console);

	return OK;
}
