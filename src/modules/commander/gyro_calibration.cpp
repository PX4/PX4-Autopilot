/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file gyro_calibration.cpp
 *
 * Gyroscope calibration routine
 */

#include "gyro_calibration.h"
#include "calibration_messages.h"
#include "commander_helper.h"

#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_gyro.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mcu_version.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static const char *sensor_name = "gyro";

int do_gyro_calibration(int mavlink_fd)
{
	const unsigned max_gyros = 3;

	int32_t device_id[3];
	mavlink_log_info(mavlink_fd, CAL_STARTED_MSG, sensor_name);
	mavlink_log_info(mavlink_fd, "HOLD STILL");

	/* wait for the user to respond */
	sleep(2);

	struct gyro_scale gyro_scale_zero = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	struct gyro_scale gyro_scale[max_gyros] = {};

	int res = OK;

	/* store board ID */
	uint32_t mcu_id[3];
	mcu_unique_id(&mcu_id[0]);

	/* store last 32bit number - not unique, but unique in a given set */
	(void)param_set(param_find("CAL_BOARD_ID"), &mcu_id[2]);

	char str[30];

	for (unsigned s = 0; s < max_gyros; s++) {

		/* ensure all scale fields are initialized tha same as the first struct */
		(void)memcpy(&gyro_scale[s], &gyro_scale_zero, sizeof(gyro_scale[0]));

		sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);
		/* reset all offsets to zero and all scales to one */
		int fd = open(str, 0);

		if (fd < 0) {
			continue;
		}

		device_id[s] = ioctl(fd, DEVIOCGDEVICEID, 0);

		res = ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale_zero);
		close(fd);

		if (res != OK) {
			mavlink_log_critical(mavlink_fd, CAL_FAILED_RESET_CAL_MSG, s);
		}
	}

	unsigned calibration_counter[max_gyros] = { 0 };
	const unsigned calibration_count = 5000;

	struct gyro_report gyro_report_0 = {};

	if (res == OK) {
		/* determine gyro mean values */
		unsigned poll_errcount = 0;

		/* subscribe to gyro sensor topic */
		int sub_sensor_gyro[max_gyros];
		struct pollfd fds[max_gyros];

		for (unsigned s = 0; s < max_gyros; s++) {
			sub_sensor_gyro[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
			fds[s].fd = sub_sensor_gyro[s];
			fds[s].events = POLLIN;
		}

		struct gyro_report gyro_report;

		/* use first gyro to pace, but count correctly per-gyro for statistics */
		while (calibration_counter[0] < calibration_count) {
			/* wait blocking for new data */

			int poll_ret = poll(&fds[0], max_gyros, 1000);

			if (poll_ret > 0) {

				for (unsigned s = 0; s < max_gyros; s++) {
					bool changed;
					orb_check(sub_sensor_gyro[s], &changed);

					if (changed) {
						orb_copy(ORB_ID(sensor_gyro), sub_sensor_gyro[s], &gyro_report);

						if (s == 0) {
							orb_copy(ORB_ID(sensor_gyro), sub_sensor_gyro[s], &gyro_report_0);
						}

						gyro_scale[s].x_offset += gyro_report.x;
						gyro_scale[s].y_offset += gyro_report.y;
						gyro_scale[s].z_offset += gyro_report.z;
						calibration_counter[s]++;
					}

					if (s == 0 && calibration_counter[0] % (calibration_count / 20) == 0) {
						mavlink_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, (calibration_counter[0] * 100) / calibration_count);
					}
				}

			} else {
				poll_errcount++;
			}

			if (poll_errcount > 1000) {
				mavlink_log_critical(mavlink_fd, CAL_FAILED_SENSOR_MSG);
				res = ERROR;
				break;
			}
		}

		for (unsigned s = 0; s < max_gyros; s++) {
			close(sub_sensor_gyro[s]);

			gyro_scale[s].x_offset /= calibration_counter[s];
			gyro_scale[s].y_offset /= calibration_counter[s];
			gyro_scale[s].z_offset /= calibration_counter[s];
		}
	}

	if (res == OK) {
		/* check offsets */
		float xdiff = gyro_report_0.x - gyro_scale[0].x_offset;
		float ydiff = gyro_report_0.y - gyro_scale[0].y_offset;
		float zdiff = gyro_report_0.z - gyro_scale[0].z_offset;

		/* maximum allowable calibration error in radians */
		const float maxoff = 0.002f;

		if (!isfinite(gyro_scale[0].x_offset) ||
		    !isfinite(gyro_scale[0].y_offset) ||
		    !isfinite(gyro_scale[0].z_offset) ||
		    fabsf(xdiff) > maxoff ||
		    fabsf(ydiff) > maxoff ||
		    fabsf(zdiff) > maxoff) {
			mavlink_log_critical(mavlink_fd, "ERROR: Motion during calibration");
			res = ERROR;
		}
	}

	if (res == OK) {
		/* set offset parameters to new values */
		bool failed = false;

		for (unsigned s = 0; s < max_gyros; s++) {

			/* if any reasonable amount of data is missing, skip */
			if (calibration_counter[s] < calibration_count / 2) {
				continue;
			}

			(void)sprintf(str, "CAL_GYRO%u_XOFF", s);
			failed |= (OK != param_set(param_find(str), &(gyro_scale[s].x_offset)));
			(void)sprintf(str, "CAL_GYRO%u_YOFF", s);
			failed |= (OK != param_set(param_find(str), &(gyro_scale[s].y_offset)));
			(void)sprintf(str, "CAL_GYRO%u_ZOFF", s);
			failed |= (OK != param_set(param_find(str), &(gyro_scale[s].z_offset)));
			(void)sprintf(str, "CAL_GYRO%u_ID", s);
			failed |= (OK != param_set(param_find(str), &(device_id[s])));

			/* apply new scaling and offsets */
			(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);
			int fd = open(str, 0);

			if (fd < 0) {
				failed = true;
				continue;
			}

			res = ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale[s]);
			close(fd);

			if (res != OK) {
				mavlink_log_critical(mavlink_fd, CAL_FAILED_APPLY_CAL_MSG);
			}
		}

		if (failed) {
			mavlink_and_console_log_critical(mavlink_fd, "ERROR: failed to set offset params");
			res = ERROR;
		}
	}

	if (res == OK) {
		/* auto-save to EEPROM */
		res = param_save_default();

		if (res != OK) {
			mavlink_log_critical(mavlink_fd, CAL_FAILED_SAVE_PARAMS_MSG);
		}
	}

	if (res == OK) {
		mavlink_log_info(mavlink_fd, CAL_DONE_MSG, sensor_name);

	} else {
		mavlink_log_info(mavlink_fd, CAL_FAILED_MSG, sensor_name);
	}

	return res;
}
