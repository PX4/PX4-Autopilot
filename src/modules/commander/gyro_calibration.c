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
 * @file gyro_calibration.c
 * Gyroscope calibration routine
 */

#include "gyro_calibration.h"
#include "commander_helper.h"

#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_gyro.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>


void do_gyro_calibration(int mavlink_fd)
{
	mavlink_log_info(mavlink_fd, "gyro calibration starting, hold still");

	const int calibration_count = 5000;

	int sub_sensor_combined = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s raw;

	int calibration_counter = 0;
	float gyro_offset[3] = {0.0f, 0.0f, 0.0f};

	/* set offsets to zero */
	int fd = open(GYRO_DEVICE_PATH, 0);
	struct gyro_scale gscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale_null))
		warn("WARNING: failed to set scale / offsets for gyro");

	close(fd);

	while (calibration_counter < calibration_count) {

		/* wait blocking for new data */
		struct pollfd fds[1] = { { .fd = sub_sensor_combined, .events = POLLIN } };

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(sensor_combined), sub_sensor_combined, &raw);
			gyro_offset[0] += raw.gyro_rad_s[0];
			gyro_offset[1] += raw.gyro_rad_s[1];
			gyro_offset[2] += raw.gyro_rad_s[2];
			calibration_counter++;

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			mavlink_log_info(mavlink_fd, "gyro calibration aborted, retry");
			return;
		}
	}

	gyro_offset[0] = gyro_offset[0] / calibration_count;
	gyro_offset[1] = gyro_offset[1] / calibration_count;
	gyro_offset[2] = gyro_offset[2] / calibration_count;

	if (isfinite(gyro_offset[0]) && isfinite(gyro_offset[1]) && isfinite(gyro_offset[2])) {

		if (param_set(param_find("SENS_GYRO_XOFF"), &(gyro_offset[0]))
			|| param_set(param_find("SENS_GYRO_YOFF"), &(gyro_offset[1]))
			|| param_set(param_find("SENS_GYRO_ZOFF"), &(gyro_offset[2]))) {
			mavlink_log_critical(mavlink_fd, "Setting gyro offsets failed!");
		}

		/* set offsets to actual value */
		fd = open(GYRO_DEVICE_PATH, 0);
		struct gyro_scale gscale = {
			gyro_offset[0],
			1.0f,
			gyro_offset[1],
			1.0f,
			gyro_offset[2],
			1.0f,
		};

		if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale))
			warn("WARNING: failed to set scale / offsets for gyro");

		close(fd);

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
		}

		// char buf[50];
		// sprintf(buf, "cal: x:%8.4f y:%8.4f z:%8.4f", (double)gyro_offset[0], (double)gyro_offset[1], (double)gyro_offset[2]);
		// mavlink_log_info(mavlink_fd, buf);
		mavlink_log_info(mavlink_fd, "gyro calibration done");

		tune_positive();
		/* third beep by cal end routine */

	} else {
		mavlink_log_info(mavlink_fd, "gyro calibration FAILED (NaN)");
	}

	close(sub_sensor_combined);
}
