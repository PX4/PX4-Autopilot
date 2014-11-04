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
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_gyro.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static const char *sensor_name = "gyro";

int do_gyro_calibration(int mavlink_fd)
{
	mavlink_log_info(mavlink_fd, CAL_STARTED_MSG, sensor_name);
	mavlink_log_info(mavlink_fd, "HOLD STILL");

	/* wait for the user to respond */
	sleep(2);

	struct gyro_scale gyro_scale = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	int res = OK;

	/* reset all offsets to zero and all scales to one */
	int fd = open(GYRO_DEVICE_PATH, 0);
	res = ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale);
	close(fd);

	if (res != OK) {
		mavlink_log_critical(mavlink_fd, CAL_FAILED_RESET_CAL_MSG);
	}

	if (res == OK) {
		/* determine gyro mean values */
		const unsigned calibration_count = 5000;
		unsigned calibration_counter = 0;
		unsigned poll_errcount = 0;

		/* subscribe to gyro sensor topic */
		int sub_sensor_gyro = orb_subscribe(ORB_ID(sensor_gyro0));
		struct gyro_report gyro_report;

		while (calibration_counter < calibration_count) {
			/* wait blocking for new data */
			struct pollfd fds[1];
			fds[0].fd = sub_sensor_gyro;
			fds[0].events = POLLIN;

			int poll_ret = poll(fds, 1, 1000);

			if (poll_ret > 0) {
				orb_copy(ORB_ID(sensor_gyro0), sub_sensor_gyro, &gyro_report);
				gyro_scale.x_offset += gyro_report.x;
				gyro_scale.y_offset += gyro_report.y;
				gyro_scale.z_offset += gyro_report.z;
				calibration_counter++;

				if (calibration_counter % (calibration_count / 20) == 0) {
					mavlink_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, (calibration_counter * 100) / calibration_count);
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

		close(sub_sensor_gyro);

		gyro_scale.x_offset /= calibration_count;
		gyro_scale.y_offset /= calibration_count;
		gyro_scale.z_offset /= calibration_count;
	}

	if (res == OK) {
		/* check offsets */
		if (!isfinite(gyro_scale.x_offset) || !isfinite(gyro_scale.y_offset) || !isfinite(gyro_scale.z_offset)) {
			mavlink_log_critical(mavlink_fd, "ERROR: offset is NaN");
			res = ERROR;
		}
	}

	if (res == OK) {
		/* set offset parameters to new values */
		if (param_set(param_find("SENS_GYRO_XOFF"), &(gyro_scale.x_offset))
		    || param_set(param_find("SENS_GYRO_YOFF"), &(gyro_scale.y_offset))
		    || param_set(param_find("SENS_GYRO_ZOFF"), &(gyro_scale.z_offset))) {
			mavlink_log_critical(mavlink_fd, "ERROR: failed to set offset params");
			res = ERROR;
		}
	}

#if 0
	/* beep on offset calibration end */
	mavlink_log_info(mavlink_fd, "gyro offset calibration done");
	tune_neutral();

	/* scale calibration */
	/* this was only a proof of concept and is currently not working. scaling will be set to 1.0 for now. */

	mavlink_log_info(mavlink_fd, "offset done. Rotate for scale 30x or wait 5s to skip.");
	warnx("offset calibration finished. Rotate for scale 30x, or do not rotate and wait for 5 seconds to skip.");

	/* apply new offsets */
	fd = open(GYRO_DEVICE_PATH, 0);

	if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale)) {
		warn("WARNING: failed to apply new offsets for gyro");
	}

	close(fd);


	unsigned rotations_count = 30;
	float gyro_integral = 0.0f;
	float baseline_integral = 0.0f;

	// XXX change to mag topic
	orb_copy(ORB_ID(sensor_combined), sub_sensor_combined, &raw);

	float mag_last = -atan2f(raw.magnetometer_ga[1], raw.magnetometer_ga[0]);

	if (mag_last > M_PI_F) { mag_last -= 2 * M_PI_F; }

	if (mag_last < -M_PI_F) { mag_last += 2 * M_PI_F; }


	uint64_t last_time = hrt_absolute_time();
	uint64_t start_time = hrt_absolute_time();

	while ((int)fabsf(baseline_integral / (2.0f * M_PI_F)) < rotations_count) {

		/* abort this loop if not rotated more than 180 degrees within 5 seconds */
		if ((fabsf(baseline_integral / (2.0f * M_PI_F)) < 0.6f)
		    && (hrt_absolute_time() - start_time > 5 * 1e6)) {
			mavlink_log_info(mavlink_fd, "scale skipped, gyro calibration done");
			close(sub_sensor_combined);
			return OK;
		}

		/* wait blocking for new data */
		struct pollfd fds[1];
		fds[0].fd = sub_sensor_combined;
		fds[0].events = POLLIN;

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {

			float dt_ms = (hrt_absolute_time() - last_time) / 1e3f;
			last_time = hrt_absolute_time();

			orb_copy(ORB_ID(sensor_combined), sub_sensor_combined, &raw);

			// XXX this is just a proof of concept and needs world / body
			// transformation and more

			//math::Vector2f magNav(raw.magnetometer_ga);

			// calculate error between estimate and measurement
			// apply declination correction for true heading as well.
			//float mag = -atan2f(magNav(1),magNav(0));
			float mag = -atan2f(raw.magnetometer_ga[1], raw.magnetometer_ga[0]);

			if (mag > M_PI_F) { mag -= 2 * M_PI_F; }

			if (mag < -M_PI_F) { mag += 2 * M_PI_F; }

			float diff = mag - mag_last;

			if (diff > M_PI_F) { diff -= 2 * M_PI_F; }

			if (diff < -M_PI_F) { diff += 2 * M_PI_F; }

			baseline_integral += diff;
			mag_last = mag;
			// Jump through some timing scale hoops to avoid
			// operating near the 1e6/1e8 max sane resolution of float.
			gyro_integral += (raw.gyro_rad_s[2] * dt_ms) / 1e3f;

//			warnx("dbg: b: %6.4f, g: %6.4f", (double)baseline_integral, (double)gyro_integral);

			// } else if (poll_ret == 0) {
			// 	/* any poll failure for 1s is a reason to abort */
			// 	mavlink_log_info(mavlink_fd, "gyro calibration aborted, retry");
			// 	return;
		}
	}

	float gyro_scale = baseline_integral / gyro_integral;

	warnx("gyro scale: yaw (z): %6.4f", (double)gyro_scale);
	mavlink_log_info(mavlink_fd, "gyro scale: yaw (z): %6.4f", (double)gyro_scale);


	if (!isfinite(gyro_scale.x_scale) || !isfinite(gyro_scale.y_scale) || !isfinite(gyro_scale.z_scale)) {
		mavlink_log_info(mavlink_fd, "gyro scale calibration FAILED (NaN)");
		close(sub_sensor_gyro);
		mavlink_log_critical(mavlink_fd, "gyro calibration failed");
		return ERROR;
	}

	/* beep on calibration end */
	mavlink_log_info(mavlink_fd, "gyro scale calibration done");
	tune_neutral();

#endif

	if (res == OK) {
		/* set scale parameters to new values */
		if (param_set(param_find("SENS_GYRO_XSCALE"), &(gyro_scale.x_scale))
		    || param_set(param_find("SENS_GYRO_YSCALE"), &(gyro_scale.y_scale))
		    || param_set(param_find("SENS_GYRO_ZSCALE"), &(gyro_scale.z_scale))) {
			mavlink_log_critical(mavlink_fd, "ERROR: failed to set scale params");
			res = ERROR;
		}
	}

	if (res == OK) {
		/* apply new scaling and offsets */
		fd = open(GYRO_DEVICE_PATH, 0);
		res = ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale);
		close(fd);

		if (res != OK) {
			mavlink_log_critical(mavlink_fd, CAL_FAILED_APPLY_CAL_MSG);
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
