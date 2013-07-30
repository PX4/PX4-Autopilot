/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file config.c
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * config tool.
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <arch/board/board.h>

#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"

__EXPORT int config_main(int argc, char *argv[]);

static void	do_gyro(int argc, char *argv[]);
static void	do_accel(int argc, char *argv[]);
static void	do_mag(int argc, char *argv[]);

int
config_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "gyro")) {
			if (argc >= 3) {
				do_gyro(argc - 2, argv + 2);
			} else {
				errx(1, "not enough parameters.");
			}
		}

		if (!strcmp(argv[1], "accel")) {
			if (argc >= 3) {
				do_accel(argc - 2, argv + 2);
			} else {
				errx(1, "not enough parameters.");
			}
		}

		if (!strcmp(argv[1], "mag")) {
			if (argc >= 3) {
				do_mag(argc - 2, argv + 2);
			} else {
				errx(1, "not enough parameters.");
			}
		}
	}
	
	errx(1, "expected a command, try 'gyro', 'accel', 'mag'");
}

static void
do_gyro(int argc, char *argv[])
{
	int	fd;

	fd = open(GYRO_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", GYRO_DEVICE_PATH);
		errx(1, "FATAL: no gyro found");

	} else {

		if (argc >= 2) {

			char* end;
			int i = strtol(argv[1],&end,10);

			if (!strcmp(argv[0], "sampling")) {

				/* set the accel internal sampling rate up to at leat i Hz */
				ioctl(fd, GYROIOCSSAMPLERATE, i);

			} else if (!strcmp(argv[0], "rate")) {

				/* set the driver to poll at i Hz */
				ioctl(fd, SENSORIOCSPOLLRATE, i);
			} else if (!strcmp(argv[0], "range")) {

				/* set the range to i dps */
				ioctl(fd, GYROIOCSRANGE, i);
			}

		} else if (!(argc > 0 && !strcmp(argv[0], "info"))) {
			warnx("no arguments given. Try: \n\n\t'sampling 500' to set sampling to 500 Hz\n\t'rate 500' to set publication rate to 500 Hz\n\t'range 2000' to set measurement range to 2000 dps\n\t");
		}

		int srate = ioctl(fd, GYROIOCGSAMPLERATE, 0);
		int prate = ioctl(fd, SENSORIOCGPOLLRATE, 0);
		int range = ioctl(fd, GYROIOCGRANGE, 0);

		warnx("gyro: \n\tsample rate:\t%d Hz\n\tread rate:\t%d Hz\n\trange:\t%d dps", srate, prate, range);

		close(fd);
	}

	exit(0);
}

static void
do_mag(int argc, char *argv[])
{
	exit(0);
}

static void
do_accel(int argc, char *argv[])
{
	int	fd;

	fd = open(ACCEL_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", ACCEL_DEVICE_PATH);
		errx(1, "FATAL: no accelerometer found");

	} else {

		if (argc >= 2) {

			char* end;
			int i = strtol(argv[1],&end,10);

			if (!strcmp(argv[0], "sampling")) {

				/* set the accel internal sampling rate up to at leat i Hz */
				ioctl(fd, ACCELIOCSSAMPLERATE, i);

			} else if (!strcmp(argv[0], "rate")) {

				/* set the driver to poll at i Hz */
				ioctl(fd, SENSORIOCSPOLLRATE, i);
			} else if (!strcmp(argv[0], "range")) {

				/* set the range to i dps */
				ioctl(fd, ACCELIOCSRANGE, i);
			}
		} else if (!(argc > 0 && !strcmp(argv[0], "info"))) {
			warnx("no arguments given. Try: \n\n\t'sampling 500' to set sampling to 500 Hz\n\t'rate 500' to set publication rate to 500 Hz\n\t'range 2' to set measurement range to 2 G\n\t");
		}

		int srate = ioctl(fd, ACCELIOCGSAMPLERATE, 0);
		int prate = ioctl(fd, SENSORIOCGPOLLRATE, 0);
		int range = ioctl(fd, ACCELIOCGRANGE, 0);

		warnx("accel: \n\tsample rate:\t%d Hz\n\tread rate:\t%d Hz\n\trange:\t%d m/s", srate, prate, range);

		close(fd);
	}

	exit(0);
}
