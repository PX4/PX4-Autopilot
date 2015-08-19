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
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cmath>
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

static void feedback_calibration_failed(int mavlink_fd)
{
	sleep(5);
	mavlink_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, sensor_name);
}

int do_airspeed_calibration(int mavlink_fd)
{
	int result = OK;
	unsigned calibration_counter = 0;
	const unsigned maxcount = 2400;

	/* give directions */
	mavlink_log_info(mavlink_fd, CAL_QGC_STARTED_MSG, sensor_name);

	const unsigned calibration_count = (maxcount * 2) / 3;

	int diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	struct differential_pressure_s diff_pres;

	float diff_pres_offset = 0.0f;

	/* Reset sensor parameters */
	struct airspeed_scale airscale = {
		diff_pres_offset,
		1.0f,
	};

	bool paramreset_successful = false;
	int  fd = px4_open(AIRSPEED0_DEVICE_PATH, 0);

	if (fd > 0) {
		if (OK == px4_ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
			paramreset_successful = true;

		} else {
			mavlink_log_critical(mavlink_fd, "[cal] airspeed offset zero failed");
		}

		px4_close(fd);
	}
    
	int cancel_sub = calibrate_cancel_subscribe();

	if (!paramreset_successful) {

		/* only warn if analog scaling is zero */
		float analog_scaling = 0.0f;
		param_get(param_find("SENS_DPRES_ANSC"), &(analog_scaling));
		if (fabsf(analog_scaling) < 0.1f) {
			mavlink_log_critical(mavlink_fd, "[cal] No airspeed sensor, see http://px4.io/help/aspd");
			goto error_return;
		}

		/* set scaling offset parameter */
		if (param_set(param_find("SENS_DPRES_OFF"), &(diff_pres_offset))) {
			mavlink_log_critical(mavlink_fd, CAL_ERROR_SET_PARAMS_MSG);
			goto error_return;
		}
	}

	mavlink_log_critical(mavlink_fd, "[cal] Ensure sensor is not measuring wind");
	usleep(500 * 1000);

	while (calibration_counter < calibration_count) {

		if (calibrate_cancel_check(mavlink_fd, cancel_sub)) {
			goto error_return;
		}
        
		/* wait blocking for new data */
		px4_pollfd_struct_t fds[1];
		fds[0].fd = diff_pres_sub;
		fds[0].events = POLLIN;

		int poll_ret = px4_poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);

			diff_pres_offset += diff_pres.differential_pressure_raw_pa;
			calibration_counter++;

			if (calibration_counter % (calibration_count / 20) == 0) {
				mavlink_log_info(mavlink_fd, CAL_QGC_PROGRESS_MSG, (calibration_counter * 80) / calibration_count);
			}

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			feedback_calibration_failed(mavlink_fd);
			goto error_return;
		}
	}

	diff_pres_offset = diff_pres_offset / calibration_count;

	if (PX4_ISFINITE(diff_pres_offset)) {

		int  fd_scale = px4_open(AIRSPEED0_DEVICE_PATH, 0);
		airscale.offset_pa = diff_pres_offset;
		if (fd_scale > 0) {
			if (OK != px4_ioctl(fd_scale, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				mavlink_log_critical(mavlink_fd, "[cal] airspeed offset update failed");
			}

			px4_close(fd_scale);
		}

		if (param_set(param_find("SENS_DPRES_OFF"), &(diff_pres_offset))) {
			mavlink_log_critical(mavlink_fd, CAL_ERROR_SET_PARAMS_MSG);
			goto error_return;
		}

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
			mavlink_log_critical(mavlink_fd, CAL_ERROR_SAVE_PARAMS_MSG);
			goto error_return;
		}

	} else {
		feedback_calibration_failed(mavlink_fd);
		goto error_return;
	}

	mavlink_log_critical(mavlink_fd, "[cal] Offset of %d Pascal", (int)diff_pres_offset);

	/* wait 500 ms to ensure parameter propagated through the system */
	usleep(500 * 1000);

	mavlink_log_critical(mavlink_fd, "[cal] Create airflow now");

	calibration_counter = 0;

	/* just take a few samples and make sure pitot tubes are not reversed, timeout after ~30 seconds */
	while (calibration_counter < maxcount) {

		if (calibrate_cancel_check(mavlink_fd, cancel_sub)) {
			goto error_return;
		}

		/* wait blocking for new data */
		px4_pollfd_struct_t fds[1];
		fds[0].fd = diff_pres_sub;
		fds[0].events = POLLIN;

		int poll_ret = px4_poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);

			calibration_counter++;

			if (fabsf(diff_pres.differential_pressure_raw_pa) < 50.0f) {
				if (calibration_counter % 500 == 0) {
					mavlink_log_info(mavlink_fd, "[cal] Create air pressure! (got %d, wanted: 50 Pa)",
						(int)diff_pres.differential_pressure_raw_pa);
				}
				continue;
			}

			/* do not allow negative values */
			if (diff_pres.differential_pressure_raw_pa < 0.0f) {
				mavlink_log_info(mavlink_fd, "[cal] Negative pressure difference detected (%d Pa)",
						(int)diff_pres.differential_pressure_raw_pa);
				mavlink_log_info(mavlink_fd, "[cal] Swap static and dynamic ports!");

				/* the user setup is wrong, wipe the calibration to force a proper re-calibration */

				diff_pres_offset = 0.0f;
				if (param_set(param_find("SENS_DPRES_OFF"), &(diff_pres_offset))) {
					mavlink_log_critical(mavlink_fd, CAL_ERROR_SET_PARAMS_MSG);
					goto error_return;
				}

				/* save */
				mavlink_log_info(mavlink_fd, CAL_QGC_PROGRESS_MSG, 0);
				(void)param_save_default();

				feedback_calibration_failed(mavlink_fd);
				goto error_return;
			} else {
				mavlink_log_info(mavlink_fd, "[cal] Positive pressure: OK (%d Pa)",
					(int)diff_pres.differential_pressure_raw_pa);
				break;
			}

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			feedback_calibration_failed(mavlink_fd);
			goto error_return;
		}
	}

	if (calibration_counter == maxcount) {
		feedback_calibration_failed(mavlink_fd);
		goto error_return;
	}

	mavlink_log_info(mavlink_fd, CAL_QGC_PROGRESS_MSG, 100);

	mavlink_log_info(mavlink_fd, CAL_QGC_DONE_MSG, sensor_name);
	tune_neutral(true);

normal_return:
	calibrate_cancel_unsubscribe(cancel_sub);
	px4_close(diff_pres_sub);
	
	// This give a chance for the log messages to go out of the queue before someone else stomps on then
	sleep(1);
	
	return result;
    
error_return:
	result = ERROR;
	goto normal_return;
}
