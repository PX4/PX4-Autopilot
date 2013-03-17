/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file pwm.c
 *
 * PWM servo output configuration and monitoring tool.
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/i2c.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "drivers/drv_pwm_output.h"

static void	usage(const char *reason);
__EXPORT int	pwm_main(int argc, char *argv[]);


static void
usage(const char *reason)
{
	if (reason != NULL)
		warnx("%s", reason);
	errx(1, 
		"usage:\n"
		"pwm [-v] [-d <device>] [-u <alt_rate>] [-c <alt_channel_mask>] [arm|disarm] [<channel_value> ...]\n"
		"  -v                 Print information about the PWM device\n"
		"  <device>           PWM output device (defaults to " PWM_OUTPUT_DEVICE_PATH ")\n"
		"  <alt_rate>         PWM update rate for channels in <alt_channel_mask>\n"
		"  <alt_channel_mask> Bitmask of channels to update at the alternate rate\n"
		"  arm | disarm       Arm or disarm the ouptut\n"
		"  <channel_value>... PWM output values in microseconds to assign to the PWM outputs\n");
}

int
pwm_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT_DEVICE_PATH;
	unsigned alt_rate = 0;
	uint32_t alt_channels;
	bool alt_channels_set = false;
	bool print_info = false;
	int ch;
	int ret;
	char *ep;

	while ((ch = getopt(argc, argv, "c:d:u:v")) != EOF) {
		switch (ch) {
		case 'c':
			alt_channels = strtol(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad alt_channel_mask value");
			alt_channels_set = true;
			break;

		case 'd':
			dev = optarg;
			break;

		case 'u':
			alt_rate = strtol(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad alt_rate value");

		case 'v':
			print_info = true;
			break;

		default:
			usage(NULL);
		}
	}
	argc -= optind;
	argv += optind;

	/* open for ioctl only */
	int fd = open(dev, 0);
	if (fd < 0)
		err(1, "can't open %s", dev);

	/* change alternate PWM rate */
	if (alt_rate > 0) {
		ret = ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, alt_rate);
		if (ret != OK)
			err(1, "PWM_SERVO_SET_UPDATE_RATE (check rate for sanity)");
	}

	/* assign alternate rate to channels */
	if (alt_channels_set) {
		ret = ioctl(fd, PWM_SERVO_SELECT_UPDATE_RATE, alt_channels);
		if (ret != OK)
			err(1, "PWM_SERVO_SELECT_UPDATE_RATE (check mask vs. device capabilities)");
	}

	/* iterate remaining arguments */
	unsigned channel = 0;
	while (argc--) {
		const char *arg = argv[0];
		argv++;
		if (!strcmp(arg, "arm")) {
			ret = ioctl(fd, PWM_SERVO_ARM, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_ARM");
			continue;
		}
		if (!strcmp(arg, "disarm")) {
			ret = ioctl(fd, PWM_SERVO_DISARM, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_DISARM");
			continue;
		}
		unsigned pwm_value = strtol(arg, &ep, 0);
		if (*ep == '\0') {
			ret = ioctl(fd, PWM_SERVO_SET(channel), pwm_value);
			if (ret != OK)
				err(1, "PWM_SERVO_SET(%d)", channel);
			channel++;
			continue;
		}
		usage("unrecognised option");
	}

	/* print verbose info */
	if (print_info) {
		/* get the number of servo channels */
		unsigned count;
		ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&count);
		if (ret != OK)
			err(1, "PWM_SERVO_GET_COUNT");

		/* print current servo values */
		printf("PWM output values:\n");
		for (unsigned i = 0; i < count; i++) {
			servo_position_t spos;

			ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);
			if (ret == OK) {
				printf("%u: %uus\n", i, spos);
			} else {
				printf("%u: ERROR\n", i);
			}
		}

		/* print rate groups */
		printf("Available alt_channel_mask groups:\n");
		for (unsigned i = 0; i < count; i++) {
			uint32_t group_mask;

			ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(i), (unsigned long)&group_mask);
			if (ret != OK)
				break;
			if (group_mask != 0)
				printf(" 0x%x", group_mask);
		}
		printf("\n");
		fflush(stdout);
	}
	exit(0);
}