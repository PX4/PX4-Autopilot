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
#include <sys/stat.h>
#include <unistd.h>
#include <arch/board/drv_eeprom.h>
#include <float.h>
#include <string.h>

#include "systemlib.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

const struct __multiport_info multiport_info = {
	.port_names = {"MULT_0_US2_RXTX", "MULT_1_US2_FLOW", "MULT_2_GPIO_12"}
};

#define EEPROM_OFFSET 64

#define EEPROM_PARAM_MAGIC_BYTE 0xAF

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void kill_task(FAR _TCB *tcb, FAR void *arg);

void killall()
{
//	printf("Sending SIGUSR1 to all processes now\n");

	/* iterate through all tasks and send kill signal */
	sched_foreach(kill_task, NULL);
}

static void kill_task(FAR _TCB *tcb, FAR void *arg)
{
	kill(tcb->pid, SIGUSR1);
}

int task_spawn(const char *name, int scheduler, int priority, int stack_size, main_t entry, const char *argv[])
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

#define PX4_BOARD_ID_FMU (5)

int fmu_get_board_info(struct fmu_board_info_s *info)
{
	/* Check which FMU version we're on */
	struct stat sb;
	int statres;

	/* Copy version-specific fields */
	statres = stat("/dev/bma180", &sb);

	if (statres == OK) {
		/* BMA180 indicates a v1.5-v1.6 board */
		strcpy(info->board_name, "FMU v1.6");
		info->board_version = 16;

	} else {
		statres = stat("/dev/accel", &sb);

		if (statres == OK) {
			/* MPU-6000 indicates a v1.7+ board */
			strcpy(info->board_name, "FMU v1.7");
			info->board_version = 17;

		} else {
			/* If no BMA and no MPU is present, it is a v1.3 board */
			strcpy(info->board_name, "FMU v1.3");
			info->board_version = 13;
		}
	}

	/* Copy general FMU fields */
	memcpy(info->header, "PX4", 3);
	info->board_id = PX4_BOARD_ID_FMU;

	return sizeof(struct fmu_board_info_s);
}

int carrier_store_board_info(const struct carrier_board_info_s *info)
{
	int ret;
	int fd = open("/dev/eeprom", O_RDWR | O_NONBLOCK);

	if (fd < 0) fprintf(stderr, "[boardinfo carrier] ERROR opening carrier eeprom\n");

	/* Enforce correct header */
	ret = write(fd, info, sizeof(struct carrier_board_info_s));
	//ret = write(fd, "PX4", 3);
	close(fd);

	return ret;
}

int carrier_get_board_info(struct carrier_board_info_s *info)
{
	int ret;
	int fd = open("/dev/eeprom", O_RDONLY | O_NONBLOCK);

	if (fd < 0)
		return -1;	/* no board */

	ret = read(fd, info, sizeof(struct carrier_board_info_s));

	/* Enforce NUL termination of human-readable string */
	if (ret == sizeof(struct carrier_board_info_s)) {
		info->board_name[sizeof(info->board_name) - 1] = '\0';
	}

	close(fd);

	return ret;
}
