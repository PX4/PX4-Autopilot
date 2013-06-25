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
 * @file mag_calibration.c
 * Magnetometer calibration routine
 */

#include "mag_calibration.h"
#include "commander_helper.h"
#include "calibration_routines.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <math.h>
#include <fcntl.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_mag.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>


void do_mag_calibration(int mavlink_fd)
{
	mavlink_log_info(mavlink_fd, "mag calibration starting, hold still");

	int sub_mag = orb_subscribe(ORB_ID(sensor_mag));
	struct mag_report mag;

	/* 45 seconds */
	uint64_t calibration_interval = 45 * 1000 * 1000;

	/* maximum 2000 values */
	const unsigned int calibration_maxcount = 500;
	unsigned int calibration_counter = 0;

	/* limit update rate to get equally spaced measurements over time (in ms) */
	orb_set_interval(sub_mag, (calibration_interval / 1000) / calibration_maxcount);

	int fd = open(MAG_DEVICE_PATH, O_RDONLY);

	/* erase old calibration */
	struct mag_scale mscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	if (OK != ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
		warn("WARNING: failed to set scale / offsets for mag");
		mavlink_log_info(mavlink_fd, "failed to set scale / offsets for mag");
	}

	/* calibrate range */
	if (OK != ioctl(fd, MAGIOCCALIBRATE, fd)) {
		warnx("failed to calibrate scale");
	}

	close(fd);

	/* calibrate offsets */

	// uint64_t calibration_start = hrt_absolute_time();

	uint64_t axis_deadline = hrt_absolute_time();
	uint64_t calibration_deadline = hrt_absolute_time() + calibration_interval;

	const char axislabels[3] = { 'X', 'Y', 'Z'};
	int axis_index = -1;

	float *x = (float *)malloc(sizeof(float) * calibration_maxcount);
	float *y = (float *)malloc(sizeof(float) * calibration_maxcount);
	float *z = (float *)malloc(sizeof(float) * calibration_maxcount);

	if (x == NULL || y == NULL || z == NULL) {
		warnx("mag cal failed: out of memory");
		mavlink_log_info(mavlink_fd, "mag cal failed: out of memory");
		warnx("x:%p y:%p z:%p\n", x, y, z);
		return;
	}

	while (hrt_absolute_time() < calibration_deadline &&
	       calibration_counter < calibration_maxcount) {

		/* wait blocking for new data */
		struct pollfd fds[1] = { { .fd = sub_mag, .events = POLLIN } };

		/* user guidance */
		if (hrt_absolute_time() >= axis_deadline &&
		    axis_index < 3) {

			axis_index++;

			char buf[50];
			sprintf(buf, "please rotate around %c", axislabels[axis_index]);
			mavlink_log_info(mavlink_fd, buf);
			tune_neutral();

			axis_deadline += calibration_interval / 3;
		}

		if (!(axis_index < 3)) {
			break;
		}

		// int axis_left = (int64_t)axis_deadline - (int64_t)hrt_absolute_time();

		// if ((axis_left / 1000) == 0 && axis_left > 0) {
		// 	char buf[50];
		// 	sprintf(buf, "[cmd] %d seconds left for axis %c", axis_left, axislabels[axis_index]);
		// 	mavlink_log_info(mavlink_fd, buf);
		// }

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(sensor_mag), sub_mag, &mag);

			x[calibration_counter] = mag.x;
			y[calibration_counter] = mag.y;
			z[calibration_counter] = mag.z;

			/* get min/max values */

			// if (mag.x < mag_min[0]) {
			// 	mag_min[0] = mag.x;
			// }
			// else if (mag.x > mag_max[0]) {
			// 	mag_max[0] = mag.x;
			// }

			// if (raw.magnetometer_ga[1] < mag_min[1]) {
			// 	mag_min[1] = raw.magnetometer_ga[1];
			// }
			// else if (raw.magnetometer_ga[1] > mag_max[1]) {
			// 	mag_max[1] = raw.magnetometer_ga[1];
			// }

			// if (raw.magnetometer_ga[2] < mag_min[2]) {
			// 	mag_min[2] = raw.magnetometer_ga[2];
			// }
			// else if (raw.magnetometer_ga[2] > mag_max[2]) {
			// 	mag_max[2] = raw.magnetometer_ga[2];
			// }

			calibration_counter++;

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			mavlink_log_info(mavlink_fd, "mag cal canceled (timed out)");
			break;
		}
	}

	float sphere_x;
	float sphere_y;
	float sphere_z;
	float sphere_radius;

	sphere_fit_least_squares(x, y, z, calibration_counter, 100, 0.0f, &sphere_x, &sphere_y, &sphere_z, &sphere_radius);

	free(x);
	free(y);
	free(z);

	if (isfinite(sphere_x) && isfinite(sphere_y) && isfinite(sphere_z)) {

		fd = open(MAG_DEVICE_PATH, 0);

		struct mag_scale mscale;

		if (OK != ioctl(fd, MAGIOCGSCALE, (long unsigned int)&mscale))
			warn("WARNING: failed to get scale / offsets for mag");

		mscale.x_offset = sphere_x;
		mscale.y_offset = sphere_y;
		mscale.z_offset = sphere_z;

		if (OK != ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale))
			warn("WARNING: failed to set scale / offsets for mag");

		close(fd);

		/* announce and set new offset */

		if (param_set(param_find("SENS_MAG_XOFF"), &(mscale.x_offset))) {
			warnx("Setting X mag offset failed!\n");
		}

		if (param_set(param_find("SENS_MAG_YOFF"), &(mscale.y_offset))) {
			warnx("Setting Y mag offset failed!\n");
		}

		if (param_set(param_find("SENS_MAG_ZOFF"), &(mscale.z_offset))) {
			warnx("Setting Z mag offset failed!\n");
		}

		if (param_set(param_find("SENS_MAG_XSCALE"), &(mscale.x_scale))) {
			warnx("Setting X mag scale failed!\n");
		}

		if (param_set(param_find("SENS_MAG_YSCALE"), &(mscale.y_scale))) {
			warnx("Setting Y mag scale failed!\n");
		}

		if (param_set(param_find("SENS_MAG_ZSCALE"), &(mscale.z_scale))) {
			warnx("Setting Z mag scale failed!\n");
		}

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
			mavlink_log_info(mavlink_fd, "FAILED storing calibration");
		}

		warnx("\tscale: %.6f %.6f %.6f\n         \toffset: %.6f %.6f %.6f\nradius: %.6f GA\n",
		       (double)mscale.x_scale, (double)mscale.y_scale, (double)mscale.z_scale,
		       (double)mscale.x_offset, (double)mscale.y_offset, (double)mscale.z_offset, (double)sphere_radius);

		char buf[52];
		sprintf(buf, "mag off: x:%.2f y:%.2f z:%.2f Ga", (double)mscale.x_offset,
			(double)mscale.y_offset, (double)mscale.z_offset);
		mavlink_log_info(mavlink_fd, buf);

		sprintf(buf, "mag scale: x:%.2f y:%.2f z:%.2f", (double)mscale.x_scale,
			(double)mscale.y_scale, (double)mscale.z_scale);
		mavlink_log_info(mavlink_fd, buf);

		mavlink_log_info(mavlink_fd, "mag calibration done");

		tune_positive();
		/* third beep by cal end routine */

	} else {
		mavlink_log_info(mavlink_fd, "mag calibration FAILED (NaN in sphere fit)");
	}

	close(sub_mag);
}