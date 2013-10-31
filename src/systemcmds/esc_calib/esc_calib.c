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

static void	usage(const char *reason);
__EXPORT int	esc_calib_main(int argc, char *argv[]);

#define MAX_CHANNELS 14

static void
usage(const char *reason)
{
	if (reason != NULL)
		warnx("%s", reason);

	errx(1,
	     "usage:\n"
	     "esc_calib [-l <low pwm>] [-h <high pwm>] [-d <device>] <channels>\n"
	     "  <device>           PWM output device (defaults to " PWM_OUTPUT_DEVICE_PATH ")\n"
	     "  <channels>         Provide channels (e.g.: 1 2 3 4)\n"
	    );

}

int
esc_calib_main(int argc, char *argv[])
{
	char *dev = PWM_OUTPUT_DEVICE_PATH;
	char *ep;
	bool channels_selected[MAX_CHANNELS] = {false};
	int ch;
	int ret;
	char c;

	int low = -1;
	int high = -1;

	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	if (argc < 2) {
		usage("no channels provided");
	}

	int arg_consumed = 0;

	while ((ch = getopt(argc, &argv[0], "l:h:d:")) != -1) {
		switch (ch) {

		case 'd':
			dev = optarg;
			arg_consumed += 2;
			break;

		case 'l':
			low = strtoul(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad low pwm value");
			arg_consumed += 2;
			break;

		case 'h':
			high = strtoul(optarg, &ep, 0);
			if (*ep != '\0')
				usage("bad high pwm value");
			arg_consumed += 2;
			break;

		default:
			usage(NULL);
		}
	}

	while ((--argc - arg_consumed) > 0) {
		const char *arg = argv[argc];
		unsigned channel_number = strtol(arg, &ep, 0);

		warnx("adding channel #%d", channel_number);

		if (*ep == '\0') {
			if (channel_number > MAX_CHANNELS || channel_number <= 0) {
				err(1, "invalid channel number: %d", channel_number);

			} else {
				channels_selected[channel_number - 1] = true;

			}
		}
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
			"\tmultirotor_att_control stop\n"
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

	/* get max PWM value setting */
	uint16_t pwm_max = 0;

	if (high > 0 && high > low && high < 2200) {
		pwm_max = high;
	} else {
		ret = ioctl(fd, PWM_SERVO_GET_MAX_PWM, &pwm_max);

		if (ret != OK)
			err(1, "PWM_SERVO_GET_MAX_PWM");
	}

	/* bound to sane values */
	if (pwm_max > 2200)
		pwm_max = 2200;

	if (pwm_max < 1700)
		pwm_max = 1700;

	/* get disarmed PWM value setting */
	uint16_t pwm_disarmed = 0;

	if (low > 0 && low < high && low > 800) {
		pwm_disarmed = low;
	} else {
		ret = ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, &pwm_disarmed);

		if (ret != OK)
			err(1, "PWM_SERVO_GET_DISARMED_PWM");

		if (pwm_disarmed == 0) {
			ret = ioctl(fd, PWM_SERVO_GET_MIN_PWM, &pwm_disarmed);

			if (ret != OK)
				err(1, "PWM_SERVO_GET_MIN_PWM");
		}
	}

	/* bound to sane values */
	if (pwm_disarmed > 1300)
		pwm_disarmed = 1300;

	if (pwm_disarmed < 800)
		pwm_disarmed = 800;

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
	       "\n", pwm_max);
	fflush(stdout);

	while (1) {
		/* set max PWM */
		for (unsigned i = 0; i < MAX_CHANNELS; i++) {
			if (channels_selected[i]) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_max);

				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d), value: %d", i, pwm_max);
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
	       "\n", pwm_disarmed);

	while (1) {

		/* set disarmed PWM */
		for (unsigned i = 0; i < MAX_CHANNELS; i++) {
			if (channels_selected[i]) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_disarmed);

				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d), value: %d", i, pwm_disarmed);
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
