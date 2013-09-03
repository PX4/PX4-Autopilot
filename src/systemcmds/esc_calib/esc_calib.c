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
__EXPORT int	esc_calib_main(int argc, char *argv[]);

#define MAX_CHANNELS 8

static void
usage(const char *reason)
{
	if (reason != NULL)
		warnx("%s", reason);
	errx(1, 
		"usage:\n"
		"esc_calib [-d <device>] <channels>\n"
		"  <device>           PWM output device (defaults to " PWM_OUTPUT_DEVICE_PATH ")\n"
		"  <channels>         Provide channels (e.g.: 1 2 3 4)\n"
		);

}

int
esc_calib_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT_DEVICE_PATH;
	char *ep;
	bool channels_selected[MAX_CHANNELS] = {false};
	int ch;
	int ret;
	char c;

	if (argc < 2)
		usage(NULL);

	while ((ch = getopt(argc, argv, "d:")) != EOF) {
		switch (ch) {
		
		case 'd':
			dev = optarg;
			argc--;
			break;

		default:
			usage(NULL);
		}
	}

	if(argc < 1) {
		usage("no channels provided");
	}

	while (--argc) {
		const char *arg = argv[argc];
		unsigned channel_number = strtol(arg, &ep, 0);

		if (*ep == '\0') {
			if (channel_number > MAX_CHANNELS || channel_number <= 0) {
				err(1, "invalid channel number: %d", channel_number);
			} else {
				channels_selected[channel_number-1] = true;

			}
		}
	}

	/* Wait for confirmation */
	int console = open("/dev/console", O_NONBLOCK | O_RDONLY | O_NOCTTY);
	if (!console)
		err(1, "failed opening console");

	warnx("ATTENTION, please remove or fix props before starting calibration!\n"
		"\n"
		"Also press the arming switch first for safety off\n"
		"\n"
		"Do you really want to start calibration: y or n?\n");

	/* wait for user input */
	while (1) {
		
		if (read(console, &c, 1) == 1) {
			if (c == 'y' || c == 'Y') {
				
				break;
			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("ESC calibration exited");
				close(console);
				exit(0);
			} else if (c == 'n' || c == 'N') {
				warnx("ESC calibration aborted");
				close(console);
				exit(0);
			} else {
				warnx("Unknown input, ESC calibration aborted");
				close(console);
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

	// XXX arming not necessaire at the moment
	// /* Then arm */
	// ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
	// 	if (ret != OK)
	// 		err(1, "PWM_SERVO_SET_ARM_OK");

	// ret = ioctl(fd, PWM_SERVO_ARM, 0);
	// 	if (ret != OK)
	// 		err(1, "PWM_SERVO_ARM");


	

	/* Wait for user confirmation */
	warnx("Set high PWM\n"
	      "Connect battery now and hit ENTER after the ESCs confirm the first calibration step");

	while (1) {

		/* First set high PWM */
		for (unsigned i = 0; i<MAX_CHANNELS; i++) {
			if(channels_selected[i]) {
				ret = ioctl(fd, PWM_SERVO_SET(i), 2100);
				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d)", i);
			}
		}

		if (read(console, &c, 1) == 1) {
			if (c == 13) {
				
				break;
			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("ESC calibration exited");
				close(console);
				exit(0);
			}
		}
		/* rate limit to ~ 20 Hz */
		usleep(50000);
	}

	/* we don't need any more user input */
	

	warnx("Set low PWM, hit ENTER when finished");

	while (1) {

		/* Then set low PWM */
		for (unsigned i = 0; i<MAX_CHANNELS; i++) {
			if(channels_selected[i]) {
				ret = ioctl(fd, PWM_SERVO_SET(i), 900);
				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d)", i);
			}
		}

		if (read(console, &c, 1) == 1) {
			if (c == 13) {
				
				break;
			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("ESC calibration exited");
				close(console);
				exit(0);
			}
		}
		/* rate limit to ~ 20 Hz */
		usleep(50000);
	}

	
	warnx("ESC calibration finished");
	close(console);

	// XXX disarming not necessaire at the moment
	/* Now disarm again */
	// ret = ioctl(fd, PWM_SERVO_DISARM, 0);

	

	exit(0);
}