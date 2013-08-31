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
		"pwm [-v] [-d <device>] [-u <alt_rate>] [-c <channel group>] [-m chanmask ] [arm|disarm] [<channel_value> ...]\n"
		"  -v                 Print information about the PWM device\n"
		"  <device>           PWM output device (defaults to " PWM_OUTPUT_DEVICE_PATH ")\n"
		"  <alt_rate>         PWM update rate for channels in <alt_channel_mask>\n"
		"  <channel_group>    Channel group that should update at the alternate rate (may be specified more than once)\n"
		"  arm | disarm       Arm or disarm the ouptut\n"
		"  <channel_value>... PWM output values in microseconds to assign to the PWM outputs\n"
		"  <chanmask>         Directly supply alt rate channel mask (debug use only)\n"
		"\n"
		"When -c is specified, any channel groups not listed with -c will update at the default rate.\n"
		);

}

int
pwm_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT_DEVICE_PATH;
	unsigned alt_rate = 0;
	uint32_t alt_channel_groups = 0;
	bool alt_channels_set = false;
	bool print_info = false;
	int ch;
	int ret;
	char *ep;
	unsigned group;
	int32_t set_mask = -1;

	if (argc < 2)
		usage(NULL);

	while ((ch = getopt(argc, argv, "c:d:u:vm:")) != EOF) {
		switch (ch) {
		case 'c':
			group = strtoul(optarg, &ep, 0);
			if ((*ep != '\0') || (group >= 32))
				usage("bad channel_group value");
			alt_channel_groups |= (1 << group);
			alt_channels_set = true;
			break;

		case 'd':
			dev = optarg;
			break;

		case 'u':
			alt_rate = strtol(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad alt_rate value");
			break;

		case 'm':
			set_mask = strtol(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad set_mask value");
			break;

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

	/* directly supplied channel mask */
	if (set_mask != -1) {
		ret = ioctl(fd, PWM_SERVO_SELECT_UPDATE_RATE, set_mask);
		if (ret != OK)
			err(1, "PWM_SERVO_SELECT_UPDATE_RATE");
	}

	/* assign alternate rate to channel groups */
	if (alt_channels_set) {
		uint32_t mask = 0;

		for (unsigned group = 0; group < 32; group++) {
			if ((1 << group) & alt_channel_groups) {
				uint32_t group_mask;

				ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask);
				if (ret != OK)
					err(1, "PWM_SERVO_GET_RATEGROUP(%u)", group);

				mask |= group_mask;
			}
		}

		ret = ioctl(fd, PWM_SERVO_SELECT_UPDATE_RATE, mask);
		if (ret != OK)
			err(1, "PWM_SERVO_SELECT_UPDATE_RATE");
	}

	/* iterate remaining arguments */
	unsigned nchannels = 0;
	unsigned channel[8] = {0};
	while (argc--) {
		const char *arg = argv[0];
		argv++;
		if (!strcmp(arg, "arm")) {
			/* tell IO that its ok to disable its safety with the switch */
			ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_SET_ARM_OK");
			/* tell IO that the system is armed (it will output values if safety is off) */
			ret = ioctl(fd, PWM_SERVO_ARM, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_ARM");
			continue;
		}
		if (!strcmp(arg, "disarm")) {
			/* disarm, but do not revoke the SET_ARM_OK flag */
			ret = ioctl(fd, PWM_SERVO_DISARM, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_DISARM");
			continue;
		}
		unsigned pwm_value = strtol(arg, &ep, 0);
		if (*ep == '\0') {
			if (nchannels > sizeof(channel) / sizeof(channel[0]))
				err(1, "too many pwm values (max %d)", sizeof(channel) / sizeof(channel[0]));

			channel[nchannels] = pwm_value;
			nchannels++;

			continue;
		}
		usage("unrecognized option");
	}

	/* print verbose info */
	if (print_info) {
		/* get the number of servo channels */
		unsigned count;
		ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&count);
		if (ret != OK)
			err(1, "PWM_SERVO_GET_COUNT");

		/* print current servo values */
		for (unsigned i = 0; i < count; i++) {
			servo_position_t spos;

			ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);
			if (ret == OK) {
				printf("channel %u: %uus\n", i, spos);
			} else {
				printf("%u: ERROR\n", i);
			}
		}

		/* print rate groups */
		for (unsigned i = 0; i < count; i++) {
			uint32_t group_mask;

			ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(i), (unsigned long)&group_mask);
			if (ret != OK)
				break;
			if (group_mask != 0) {
				printf("channel group %u: channels", i);
				for (unsigned j = 0; j < 32; j++)
					if (group_mask & (1 << j))
						printf(" %u", j);
				printf("\n");
			}
		}
		fflush(stdout);
	}

	/* perform PWM output */
	if (nchannels) {

		/* Open console directly to grab CTRL-C signal */
		int console = open("/dev/console", O_NONBLOCK | O_RDONLY | O_NOCTTY);
		if (!console)
			err(1, "failed opening console");

		warnx("Press CTRL-C or 'c' to abort.");

		while (1) {
			for (int i = 0; i < nchannels; i++) {
				ret = ioctl(fd, PWM_SERVO_SET(i), channel[i]);
				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d)", i);
			}

			/* abort on user request */
			char c;
			if (read(console, &c, 1) == 1) {
				if (c == 0x03 || c == 0x63 || c == 'q') {
					warnx("User abort\n");
					close(console);
					exit(0);
				}
			}

			/* rate limit to ~ 20 Hz */
			usleep(50000);
		}
	}

	exit(0);
}