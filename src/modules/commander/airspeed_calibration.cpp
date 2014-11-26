/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file airspeed_calibration.cpp
 * Airspeed sensor calibration routine
 */

#include "airspeed_calibration.h"
#include "calibration_messages.h"
#include "commander_helper.h"

#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_airspeed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/differential_pressure.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static const char *sensor_name = "dpress";

#define HUMAN_ASPD_CAL_FAILED_MSG "Calibration failed, see http://px4.io/help/aspd"

static void feedback_calibration_failed(int mavlink_fd)
{
	sleep(5);
	mavlink_log_critical(mavlink_fd, CAL_FAILED_MSG, sensor_name);
	mavlink_log_critical(mavlink_fd, HUMAN_ASPD_CAL_FAILED_MSG);
}

int do_airspeed_calibration(int mavlink_fd)
{
	/* give directions */
	mavlink_log_info(mavlink_fd, CAL_STARTED_MSG, sensor_name);

	const unsigned calibration_count = 2000;

	int diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	struct differential_pressure_s diff_pres;

	float diff_pres_offset = 0.0f;

	/* Reset sensor parameters */
	struct airspeed_scale airscale = {
		diff_pres_offset,
		1.0f,
	};

	bool paramreset_successful = false;
	int  fd = open(AIRSPEED_DEVICE_PATH, 0);

	if (fd > 0) {
		if (OK == ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
			paramreset_successful = true;

		} else {
			mavlink_log_critical(mavlink_fd, "airspeed offset zero failed");
		}

		close(fd);
	}

	if (!paramreset_successful) {

		/* only warn if analog scaling is zero */
		float analog_scaling = 0.0f;
		param_get(param_find("SENS_DPRES_ANSC"), &(analog_scaling));
		if (fabsf(analog_scaling) < 0.1f) {
			mavlink_log_critical(mavlink_fd, "No airspeed sensor, see http://px4.io/help/aspd");
			close(diff_pres_sub);
			return ERROR;
		}

		/* set scaling offset parameter */
		if (param_set(param_find("SENS_DPRES_OFF"), &(diff_pres_offset))) {
			mavlink_log_critical(mavlink_fd, CAL_FAILED_SET_PARAMS_MSG);
			close(diff_pres_sub);
			return ERROR;
		}
	}

	unsigned calibration_counter = 0;

	mavlink_log_critical(mavlink_fd, "Ensure sensor is not measuring wind");
	usleep(500 * 1000);

	while (calibration_counter < calibration_count) {

		/* wait blocking for new data */
		struct pollfd fds[1];
		fds[0].fd = diff_pres_sub;
		fds[0].events = POLLIN;

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);

			diff_pres_offset += diff_pres.differential_pressure_raw_pa;
			calibration_counter++;

			if (calibration_counter % (calibration_count / 20) == 0) {
				mavlink_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, (calibration_counter * 80) / calibration_count);
			}

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			feedback_calibration_failed(mavlink_fd);
			close(diff_pres_sub);
			return ERROR;
		}
	}

	diff_pres_offset = diff_pres_offset / calibration_count;

	if (isfinite(diff_pres_offset)) {

		int  fd_scale = open(AIRSPEED_DEVICE_PATH, 0);
		airscale.offset_pa = diff_pres_offset;
		if (fd_scale > 0) {
			if (OK != ioctl(fd_scale, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				mavlink_log_critical(mavlink_fd, "airspeed offset update failed");
			}

			close(fd_scale);
		}

		if (param_set(param_find("SENS_DPRES_OFF"), &(diff_pres_offset))) {
			mavlink_log_critical(mavlink_fd, CAL_FAILED_SET_PARAMS_MSG);
			close(diff_pres_sub);
			return ERROR;
		}

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
			mavlink_log_critical(mavlink_fd, CAL_FAILED_SAVE_PARAMS_MSG);
			close(diff_pres_sub);
			return ERROR;
		}

	} else {
		feedback_calibration_failed(mavlink_fd);
		close(diff_pres_sub);
		return ERROR;
	}

	mavlink_log_critical(mavlink_fd, "Offset of %d Pascal", (int)diff_pres_offset);

	/* wait 500 ms to ensure parameter propagated through the system */
	usleep(500 * 1000);

	mavlink_log_critical(mavlink_fd, "Create airflow now");

	calibration_counter = 0;
	const unsigned maxcount = 3000;

	/* just take a few samples and make sure pitot tubes are not reversed, timeout after ~30 seconds */
	while (calibration_counter < maxcount) {

		/* wait blocking for new data */
		struct pollfd fds[1];
		fds[0].fd = diff_pres_sub;
		fds[0].events = POLLIN;

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);

			calibration_counter++;

			if (fabsf(diff_pres.differential_pressure_raw_pa) < 50.0f) {
				if (calibration_counter % 500 == 0) {
					mavlink_log_info(mavlink_fd, "Create air pressure! (got %d, wanted: 50 Pa)",
						(int)diff_pres.differential_pressure_raw_pa);
				}
				continue;
			}

			/* do not allow negative values */
			if (diff_pres.differential_pressure_raw_pa < 0.0f) {
				mavlink_log_info(mavlink_fd, "ERROR: Negative pressure difference detected! (%d Pa)",
						(int)diff_pres.differential_pressure_raw_pa);
				mavlink_log_critical(mavlink_fd, "Swap static and dynamic ports!");
				close(diff_pres_sub);

				/* the user setup is wrong, wipe the calibration to force a proper re-calibration */

				diff_pres_offset = 0.0f;
				if (param_set(param_find("SENS_DPRES_OFF"), &(diff_pres_offset))) {
					mavlink_log_critical(mavlink_fd, CAL_FAILED_SET_PARAMS_MSG);
					close(diff_pres_sub);
					return ERROR;
				}

				/* save */
				mavlink_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 0);
				(void)param_save_default();

				close(diff_pres_sub);

				feedback_calibration_failed(mavlink_fd);
				return ERROR;
			} else {
				mavlink_log_info(mavlink_fd, "Positive pressure: OK (%d Pa)",
					(int)diff_pres.differential_pressure_raw_pa);
				break;
			}

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			feedback_calibration_failed(mavlink_fd);
			close(diff_pres_sub);
			return ERROR;
		}
	}

	if (calibration_counter == maxcount) {
		feedback_calibration_failed(mavlink_fd);
		close(diff_pres_sub);
		return ERROR;
	}

	mavlink_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 100);

	mavlink_log_info(mavlink_fd, CAL_DONE_MSG, sensor_name);
	tune_neutral(true);
	close(diff_pres_sub);
	return OK;
}
