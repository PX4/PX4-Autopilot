/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *
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
 * Implementation of commonly used low-level system-call like functions
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
#include <uORB/parameter_storage.h>
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
 * Public Functions
 ****************************************************************************/

void kill_task(FAR _TCB *tcb, FAR void *arg);

/****************************************************************************
 * user_start
 ****************************************************************************/

int reboot(void)
{
	sched_lock();
	// print text
	printf("\r\nRebooting system - ending tasks and performing hard reset\r\n\r\n");
	fflush(stdout);
	//usleep(5000);

	/* Sending kill signal to other tasks */
	//killall();

	/* Waiting maximum time for all to exit */
	//usleep(5000);
	//sched_lock();

	/* Resetting CPU */
	// FIXME Need check for ARM architecture here
#ifndef NVIC_AIRCR
#define NVIC_AIRCR (*((uint32_t*)0xE000ED0C))
#endif

	/* Set the SYSRESETREQ bit to force a reset */
	NVIC_AIRCR = 0x05fa0004;

	/* Spinning until the board is really reset */
	while (true);

	/* Should never reach here */
	return 0;
}

void killall()
{
//	printf("Sending SIGUSR1 to all processes now\n");

	/* iterate through all tasks and send kill signal */
	sched_foreach(kill_task, NULL);
}

void kill_task(FAR _TCB *tcb, FAR void *arg)
{
	kill(tcb->pid, SIGUSR1);
}

int store_params_in_eeprom(struct global_data_parameter_storage_t *params)
{
	int ret = ERROR;
	int fd = open("/dev/eeprom", O_RDWR | O_NONBLOCK);
	int lseek_res = lseek(fd, EEPROM_OFFSET, SEEK_SET); //don't touch first 64 bytes
	int write_res;

	if (fd < 0) {
		fprintf(stderr, "onboard eeprom: open fail\n");
		ret = ERROR;

	} else if (lseek_res < 0) {
		fprintf(stderr, "onboard eeprom: set offet fail\n");
		ret = ERROR;

	} else {
		/*Write start magic byte */
		uint8_t mb = EEPROM_PARAM_MAGIC_BYTE;
		write_res = write(fd, &mb, sizeof(mb));

		if (write_res != sizeof(mb)) {
			ret = ERROR;

		} else {
			for (int i = 0; i < params->pm.size; i++) {
				write_res = write(fd, params->pm.param_values + i, sizeof(params->pm.param_values[i]));

				if (write_res != sizeof(params->pm.param_values[i])) return ERROR;
			}

			/*Write end magic byte */
			write_res = write(fd, &mb, sizeof(mb));

			if (write_res != sizeof(mb)) {
				ret = ERROR;

			} else {
				ret = OK;
			}
		}

	}

	close(fd);

	return ret;
}

int get_params_from_eeprom(struct global_data_parameter_storage_t *params)
{
	int ret = ERROR;
	uint8_t magic_byte = 0;
	int fd = open("/dev/eeprom", O_RDWR | O_NONBLOCK);
	int lseek_res = lseek(fd, EEPROM_OFFSET, SEEK_SET);  //don't touch first 64 bytes

	if (fd < 0) {
		fprintf(stderr, "onboard eeprom: open fail\n");
		ret = ERROR;

	} else if (lseek_res < 0) {
		fprintf(stderr, "onboard eeprom: set offet fail\n");
		ret = ERROR;

	} else {
		/*Get start magic byte */
		magic_byte = 0;
		int read_res = read(fd, &magic_byte, sizeof(uint8_t));

		if (read_res != sizeof(uint8_t)) {
			ret = ERROR;

		} else {
			if (magic_byte != EEPROM_PARAM_MAGIC_BYTE) {
				ret = ERROR;
				fprintf(stderr, "onboard eeprom: parameters: start magic byte wrong\n");

			} else {
				/*get end magic byte */
				lseek_res = lseek(fd, EEPROM_OFFSET + 1 + params->pm.size * sizeof(float), SEEK_SET); // jump to 2nd magic byte

				if (lseek_res == OK) {
					/*Get end magic */
					read_res = read(fd, &magic_byte, sizeof(uint8_t));

					if (read_res != sizeof(uint8_t)) {
						ret = ERROR;

					} else {
						if (magic_byte != EEPROM_PARAM_MAGIC_BYTE) {
							ret = ERROR;
							printf("onboard eeprom: parameters: end magic byte wrong\n");

						} else {
							lseek_res = lseek(fd, EEPROM_OFFSET + 1, SEEK_SET);

							/* read data */
							if (lseek_res == OK) {

								for (int i = 0; i < params->pm.size; i++) {
									read_res = read(fd, params->pm.param_values + i,  sizeof(params->pm.param_values[i]));

									if (read_res != sizeof(params->pm.param_values[i])) return ERROR;
								}

								ret = OK;

							} else {
								/* lseek #2 failed */
								ret = ERROR;
							}
						}
					}

				} else {
					/* lseek #1 failed */
					ret = ERROR;
				}
			}
		}

	}

	close(fd);

	return ret;
}

#define PX4_BOARD_ID_FMU (5)

int fmu_get_board_info(struct fmu_board_info_s *info)
{
	/* Check which FMU version we're on */
	struct stat sb;
	int statres;

	/* Copy version-specific fields */
	statres = stat("/dev/bma280", &sb);

	if (statres == OK) {
		/* BMA280 indicates a v1.7+ board */
		strcpy(info->board_name, "FMU v1.7");
		info->board_version = 17;

	} else {
		statres = stat("/dev/bma180", &sb);

		if (statres == OK) {
			/* BMA180 indicates a v1.5-v1.6 board */
			strcpy(info->board_name, "FMU v1.6");
			info->board_version = 16;

		} else {
			/* If no BMA pressure sensor is present, it is a v1.3 board */
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

	if (fd < 0) fprintf(stderr, "[boardinfo carrier] ERROR opening carrier eeprom\n");

	ret = read(fd, info, sizeof(struct carrier_board_info_s));

	/* Enforce NUL termination of human-readable string */
	if (ret == sizeof(struct carrier_board_info_s)) {
		info->board_name[sizeof(info->board_name) - 1] = '\0';
	}

	close(fd);

	return ret;
}
