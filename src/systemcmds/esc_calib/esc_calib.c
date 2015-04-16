/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Julian Oes <joes@student.ethz.ch>
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
 * @file esc_calib.c
 *
 * Tool for ESC calibration
 */

#include <nuttx/config.h>
#include <platforms/px4_defines.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
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

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>

static void	usage(const char *reason);
__EXPORT int	esc_calib_main(int argc, char *argv[]);

static void
usage(const char *reason)
{
	if (reason != NULL)
		warnx("%s", reason);

	errx(1,
		"usage:\n"
		"esc_calib\n"
		"    [-d <device>        PWM output device (defaults to " PWM_OUTPUT0_DEVICE_PATH ")\n"
		"    [-l <pwm>           Low PWM value in us (default: %dus)\n"
		"    [-h <pwm>           High PWM value in us (default: %dus)\n"
		"    [-c <channels>]     Supply channels (e.g. 1234)\n"
		"    [-m <chanmask> ]    Directly supply channel mask (e.g. 0xF)\n"
		"    [-a]                Use all outputs\n"
		, PWM_DEFAULT_MIN, PWM_DEFAULT_MAX);
}

int
esc_calib_main(int argc, char *argv[])
{
	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	char *ep;
	int ch;
	int ret;
	char c;

	unsigned max_channels = 0;

	uint32_t set_mask = 0;
	unsigned long channels;
	unsigned single_ch = 0;

	uint16_t pwm_high = PWM_DEFAULT_MAX;
	uint16_t pwm_low = PWM_DEFAULT_MIN;

	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	if (argc < 2) {
		usage("no channels provided");
	}

	int arg_consumed = 0;

	while ((ch = getopt(argc, argv, "d:c:m:al:h:")) != EOF) {
		switch (ch) {

		case 'd':
			dev = optarg;
			arg_consumed += 2;
			break;

		case 'c':
			/* Read in channels supplied as one int and convert to mask: 1234 -> 0xF */
			channels = strtoul(optarg, &ep, 0);

			while ((single_ch = channels % 10)) {

				set_mask |= 1<<(single_ch-1);
				channels /= 10;
			}
			break;

		case 'm':
			/* Read in mask directly */
			set_mask = strtoul(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad set_mask value");
			break;

		case 'a':
			/* Choose all channels */
			for (unsigned i = 0; i<PWM_OUTPUT_MAX_CHANNELS; i++) {
				set_mask |= 1<<i;
			}
			break;

		case 'l':
			/* Read in custom low value */
			pwm_low = strtoul(optarg, &ep, 0);
			if (*ep != '\0' || pwm_low < PWM_LOWEST_MIN || pwm_low > PWM_HIGHEST_MIN)
				usage("low PWM invalid");
			break;
		case 'h':
			/* Read in custom high value */
			pwm_high = strtoul(optarg, &ep, 0);
			if (*ep != '\0' || pwm_high > PWM_HIGHEST_MAX || pwm_high < PWM_LOWEST_MAX)
				usage("high PWM invalid");
			break;
		default:
			usage(NULL);
		}
	}

	if (set_mask == 0) {
		usage("no channels chosen");
	}

	if (pwm_low > pwm_high) {
		usage("low pwm is higher than high pwm");
	}

	/* make sure no other source is publishing control values now */
	struct actuator_controls_s actuators;
	int act_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);

	/* clear changed flag */
	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_sub, &actuators);

	/* wait 50 ms */
	usleep(50000);

	/* now expect nothing changed on that topic */
	bool orb_updated;
	orb_check(act_sub, &orb_updated);

	if (orb_updated) {
		errx(1, "ABORTING! Attitude control still active. Please ensure to shut down all controllers:\n"
			"\tmc_att_control stop\n"
			"\tfw_att_control stop\n");
	}

	printf("\nATTENTION, please remove or fix propellers before starting calibration!\n"
	       "\n"
	       "Make sure\n"
	       "\t - that the ESCs are not powered\n"
	       "\t - that safety is off (two short blinks)\n"
	       "\t - that the controllers are stopped\n"
	       "\n"
	       "Do you want to start calibration now: y or n?\n");

	/* wait for user input */
	while (1) {

		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 'y' || c == 'Y') {

				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				printf("ESC calibration exited\n");
				exit(0);

			} else if (c == 'n' || c == 'N') {
				printf("ESC calibration aborted\n");
				exit(0);

			} else {
				printf("Unknown input, ESC calibration aborted\n");
				exit(0);
			}
		}

		/* rate limit to ~ 20 Hz */
		usleep(50000);
	}

	/* open for ioctl only */
	int fd = open(dev, 0);

	if (fd < 0)
		err(1, "can't open %s", dev);

	/* get number of channels available on the device */
	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&max_channels);
	if (ret != OK)
		err(1, "PWM_SERVO_GET_COUNT");

	/* tell IO/FMU that its ok to disable its safety with the switch */
	ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_SET_ARM_OK");
	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	ret = ioctl(fd, PWM_SERVO_ARM, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_ARM");

	warnx("Outputs armed");


	/* wait for user confirmation */
	printf("\nHigh PWM set: %d\n"
	       "\n"
	       "Connect battery now and hit ENTER after the ESCs confirm the first calibration step\n"
	       "\n", pwm_high);
	fflush(stdout);

	while (1) {
		/* set max PWM */
		for (unsigned i = 0; i < max_channels; i++) {

			if (set_mask & 1<<i) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_high);

				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d), value: %d", i, pwm_high);
			}
		}

		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 13) {

				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("ESC calibration exited");
				exit(0);
			}
		}

		/* rate limit to ~ 20 Hz */
		usleep(50000);
	}

	printf("Low PWM set: %d\n"
	       "\n"
	       "Hit ENTER when finished\n"
	       "\n", pwm_low);

	while (1) {

		/* set disarmed PWM */
		for (unsigned i = 0; i < max_channels; i++) {
			if (set_mask & 1<<i) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_low);

				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d), value: %d", i, pwm_low);
			}
		}

		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 13) {

				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				printf("ESC calibration exited\n");
				exit(0);
			}
		}

		/* rate limit to ~ 20 Hz */
		usleep(50000);
	}

	/* disarm */
	ret = ioctl(fd, PWM_SERVO_DISARM, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_DISARM");

	warnx("Outputs disarmed");

	printf("ESC calibration finished\n");

	exit(0);
}
