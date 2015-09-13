/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file printload.c
 *
 * Print the current system load.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <string.h>
#include <stdio.h>

#include <systemlib/cpuload.h>
#include <systemlib/printload.h>
#include <drivers/drv_hrt.h>

extern struct system_load_s system_load;

#define CL "\033[K" // clear line

void init_print_load_s(uint64_t t, struct print_load_s *s)
{

	s->total_user_time = 0;

	s->running_count = 0;
	s->blocked_count = 0;

	s->new_time = t;
	s->interval_start_time = t;

	for (int i = 0; i < CONFIG_MAX_TASKS; i++) {
		s->last_times[i] = 0;
	}

	s->interval_time_ms_inv = 0.f;
}

static const char *
tstate_name(const tstate_t s)
{
	switch (s) {
	case TSTATE_TASK_INVALID:
		return "init";

	case TSTATE_TASK_PENDING:
		return "PEND";

	case TSTATE_TASK_READYTORUN:
		return "READY";

	case TSTATE_TASK_RUNNING:
		return "RUN";

	case TSTATE_TASK_INACTIVE:
		return "inact";

	case TSTATE_WAIT_SEM:
		return "w:sem";
#ifndef CONFIG_DISABLE_SIGNALS

	case TSTATE_WAIT_SIG:
		return "w:sig";
#endif
#ifndef CONFIG_DISABLE_MQUEUE

	case TSTATE_WAIT_MQNOTEMPTY:
		return "w:mqe";

	case TSTATE_WAIT_MQNOTFULL:
		return "w:mqf";
#endif
#ifdef CONFIG_PAGING

	case TSTATE_WAIT_PAGEFILL:
		return "w:pgf";
#endif

	default:
		return "ERROR";
	}
}

void print_load(uint64_t t, int fd, struct print_load_s *print_state)
{
	print_state->new_time = t;

	int   i;
	uint64_t curr_time_us;
	uint64_t idle_time_us;
	char *clear_line = "";

	/* print system information */
	if (fd == 1) {
		dprintf(fd, "\033[H"); /* move cursor home and clear screen */
		clear_line = CL;
	}

	curr_time_us = t;
	idle_time_us = system_load.tasks[0].total_runtime;

	if (print_state->new_time > print_state->interval_start_time) {
		print_state->interval_time_ms_inv = 1.f / ((float)((print_state->new_time - print_state->interval_start_time) / 1000));
	}

	print_state->running_count = 0;
	print_state->blocked_count = 0;
	print_state->total_user_time = 0;

	for (i = 0; i < CONFIG_MAX_TASKS; i++) {
		uint64_t interval_runtime;

		if (system_load.tasks[i].valid) {
			switch (system_load.tasks[i].tcb->task_state) {
			case TSTATE_TASK_PENDING:
			case TSTATE_TASK_READYTORUN:
			case TSTATE_TASK_RUNNING:
				print_state->running_count++;
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
				print_state->blocked_count++;
				break;
			}
		}

		interval_runtime = (system_load.tasks[i].valid && print_state->last_times[i] > 0 &&
				    system_load.tasks[i].total_runtime > print_state->last_times[i])
				   ? (system_load.tasks[i].total_runtime - print_state->last_times[i]) / 1000
				   : 0;

		print_state->last_times[i] = system_load.tasks[i].total_runtime;

		if (system_load.tasks[i].valid && (print_state->new_time > print_state->interval_start_time)) {
			print_state->curr_loads[i] = interval_runtime * print_state->interval_time_ms_inv;

			if (i > 0) {
				print_state->total_user_time += interval_runtime;
			}

		} else {
			print_state->curr_loads[i] = 0;
		}
	}

	for (i = 0; i < CONFIG_MAX_TASKS; i++) {
		if (system_load.tasks[i].valid && (print_state->new_time > print_state->interval_start_time)) {
			if (system_load.tasks[i].tcb->pid == 0) {
				float idle;
				float task_load;
				float sched_load;

				idle = print_state->curr_loads[0];
				task_load = (float)(print_state->total_user_time) * print_state->interval_time_ms_inv;

				/* this can happen if one tasks total runtime was not computed
				   correctly by the scheduler instrumentation TODO */
				if (task_load > (1.f - idle)) {
					task_load = (1.f - idle);
				}

				sched_load = 1.f - idle - task_load;

				dprintf(fd, "%sProcesses: %d total, %d running, %d sleeping\n",
					clear_line,
					system_load.total_count,
					print_state->running_count,
					print_state->blocked_count);
				dprintf(fd, "%sCPU usage: %.2f%% tasks, %.2f%% sched, %.2f%% idle\n",
					clear_line,
					(double)(task_load * 100.f),
					(double)(sched_load * 100.f),
					(double)(idle * 100.f));
				dprintf(fd, "%sUptime: %.3fs total, %.3fs idle\n%s\n",
					clear_line,
					(double)curr_time_us / 1000000.d,
					(double)idle_time_us / 1000000.d,
					clear_line);

				/* header for task list */
				dprintf(fd, "%s%4s %*-s %8s %6s %11s %10s %-6s\n",
					clear_line,
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
				if (*stack_sweeper++ != 0xff) {
					break;
				}

				stack_free++;
			}

			dprintf(fd, "%s%4d %*-s %8lld %2d.%03d %5u/%5u %3u (%3u) ",
				clear_line,
				system_load.tasks[i].tcb->pid,
				CONFIG_TASK_NAME_SIZE, system_load.tasks[i].tcb->name,
				(system_load.tasks[i].total_runtime / 1000),
				(int)(print_state->curr_loads[i] * 100.0f),
				(int)((print_state->curr_loads[i] * 100.0f - (int)(print_state->curr_loads[i] * 100.0f)) * 1000),
				stack_size - stack_free,
				stack_size,
				system_load.tasks[i].tcb->sched_priority,
#if CONFIG_ARCH_BOARD_SIM
				0);
#else
				system_load.tasks[i].tcb->base_priority);
#endif

#if CONFIG_RR_INTERVAL > 0
			/* print scheduling info with RR time slice */
			dprintf(fd, " %6d\n", system_load.tasks[i].tcb->timeslice);
#else
			// print task state instead
			dprintf(fd, " %-6s\n", tstate_name(system_load.tasks[i].tcb->task_state));
#endif
		}
	}

	print_state->interval_start_time = print_state->new_time;
}

