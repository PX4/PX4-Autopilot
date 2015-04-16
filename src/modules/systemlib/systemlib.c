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
 * @file systemlib.c
 * Implementation of commonly used low-level system-call like functions.
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <unistd.h>
#include <float.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <stm32_pwr.h>

#include "systemlib.h"

// Didn't seem right to include up_internal.h, so direct extern instead.
extern void up_systemreset(void) noreturn_function;

void
systemreset(bool to_bootloader)
{
	if (to_bootloader) {
		stm32_pwr_enablebkp();

		/* XXX wow, this is evil - write a magic number into backup register zero */
		*(uint32_t *)0x40002850 = 0xb007b007;
	}

	up_systemreset();

	/* lock up here */
	while (true);
}

static void kill_task(FAR struct tcb_s *tcb, FAR void *arg);

void killall()
{
//	printf("Sending SIGUSR1 to all processes now\n");

	/* iterate through all tasks and send kill signal */
	sched_foreach(kill_task, NULL);
}

static void kill_task(FAR struct tcb_s *tcb, FAR void *arg)
{
	kill(tcb->pid, SIGUSR1);
}

int task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, main_t entry, char *const argv[])
{
	int pid;

	sched_lock();

	/* create the task */
	pid = task_create(name, priority, stack_size, entry, argv);

	if (pid > 0) {

		/* configure the scheduler */
		struct sched_param param;

		param.sched_priority = priority;
		sched_setscheduler(pid, scheduler, &param);

		/* XXX do any other private task accounting here before the task starts */
	}

	sched_unlock();

	return pid;
}
