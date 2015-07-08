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
#include "calibration_routines.h"
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

static const unsigned max_gyros = 3;

/// Data passed to calibration worker routine
typedef struct  {
	int			mavlink_fd;
	int32_t			device_id[max_gyros];
	int			gyro_sensor_sub[max_gyros];
	struct gyro_scale	gyro_scale[max_gyros];
	struct gyro_report	gyro_report_0;
} gyro_worker_data_t;

static calibrate_return gyro_calibration_worker(int cancel_sub, void* data)
{
	gyro_worker_data_t*	worker_data = (gyro_worker_data_t*)(data);
	unsigned		calibration_counter[max_gyros] = { 0 };
	const unsigned		calibration_count = 5000;
	struct gyro_report	gyro_report;
	unsigned		poll_errcount = 0;
	
	struct pollfd fds[max_gyros];
	for (unsigned s = 0; s < max_gyros; s++) {
		fds[s].fd = worker_data->gyro_sensor_sub[s];
		fds[s].events = POLLIN;
	}
	
	memset(&worker_data->gyro_report_0, 0, sizeof(worker_data->gyro_report_0));
	memset(&worker_data->gyro_scale, 0, sizeof(worker_data->gyro_scale));
	
	/* use first gyro to pace, but count correctly per-gyro for statistics */
	while (calibration_counter[0] < calibration_count) {
		if (calibrate_cancel_check(worker_data->mavlink_fd, cancel_sub)) {
			return calibrate_return_cancelled;
		}
		
		int poll_ret = poll(&fds[0], max_gyros, 1000);
		
		if (poll_ret > 0) {
			
			for (unsigned s = 0; s < max_gyros; s++) {
				bool changed;
				orb_check(worker_data->gyro_sensor_sub[s], &changed);
				
				if (changed) {
					orb_copy(ORB_ID(sensor_gyro), worker_data->gyro_sensor_sub[s], &gyro_report);
					
					if (s == 0) {
						orb_copy(ORB_ID(sensor_gyro), worker_data->gyro_sensor_sub[s], &worker_data->gyro_report_0);
					}
					
					worker_data->gyro_scale[s].x_offset += gyro_report.x;
					worker_data->gyro_scale[s].y_offset += gyro_report.y;
					worker_data->gyro_scale[s].z_offset += gyro_report.z;
					calibration_counter[s]++;
				}
				
				if (s == 0 && calibration_counter[0] % (calibration_count / 20) == 0) {
					mavlink_log_info(worker_data->mavlink_fd, CAL_QGC_PROGRESS_MSG, (calibration_counter[0] * 100) / calibration_count);
				}
			}
			
		} else {
			poll_errcount++;
		}
		
		if (poll_errcount > 1000) {
			mavlink_log_critical(worker_data->mavlink_fd, CAL_ERROR_SENSOR_MSG);
			return calibrate_return_error;
		}
	}
	
	for (unsigned s = 0; s < max_gyros; s++) {
		if (worker_data->device_id[s] != 0 && calibration_counter[s] < calibration_count / 2) {
			mavlink_log_critical(worker_data->mavlink_fd, "[cal] ERROR: missing data, sensor %d", s)
			return calibrate_return_error;
		}

		worker_data->gyro_scale[s].x_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].y_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].z_offset /= calibration_counter[s];
	}

	return calibrate_return_ok;
}

int do_gyro_calibration(int mavlink_fd)
{
	int			res = OK;
	gyro_worker_data_t	worker_data = {};

	mavlink_log_info(mavlink_fd, CAL_QGC_STARTED_MSG, sensor_name);

	worker_data.mavlink_fd = mavlink_fd;
	
	struct gyro_scale gyro_scale_zero = {
		0.0f,	// x offset
		1.0f,	// x scale
		0.0f,	// y offset
		1.0f,	// y scale
		0.0f,	// z offset
		1.0f,	// z scale
	};
	
	for (unsigned s = 0; s < max_gyros; s++) {
		char str[30];
		
		// Reset gyro ids to unavailable
		worker_data.device_id[s] = 0;
		(void)sprintf(str, "CAL_GYRO%u_ID", s);
		res = param_set_no_notification(param_find(str), &(worker_data.device_id[s]));
		if (res != OK) {
			mavlink_log_critical(mavlink_fd, "[cal] Unable to reset CAL_GYRO%u_ID", s);
			return ERROR;
		}
		
		// Reset all offsets to 0 and scales to 1
		(void)memcpy(&worker_data.gyro_scale[s], &gyro_scale_zero, sizeof(gyro_scale));
		sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);
		int fd = open(str, 0);
		if (fd >= 0) {
			worker_data.device_id[s] = ioctl(fd, DEVIOCGDEVICEID, 0);
			res = ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale_zero);
			close(fd);

			if (res != OK) {
				mavlink_log_critical(mavlink_fd, CAL_ERROR_RESET_CAL_MSG, s);
				return ERROR;
			}
		}
		
	}
	
	for (unsigned s = 0; s < max_gyros; s++) {
		worker_data.gyro_sensor_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	int cancel_sub  = calibrate_cancel_subscribe();

	unsigned try_count = 0;
	unsigned max_tries = 20;
	res = ERROR;
	
	do {
		// Calibrate gyro and ensure user didn't move
		calibrate_return cal_return = gyro_calibration_worker(cancel_sub, &worker_data);

		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already sent, we are done here
			res = ERROR;
			break;

		} else if (cal_return == calibrate_return_error) {
			res = ERROR;

		} else {
			/* check offsets */
			float xdiff = worker_data.gyro_report_0.x - worker_data.gyro_scale[0].x_offset;
			float ydiff = worker_data.gyro_report_0.y - worker_data.gyro_scale[0].y_offset;
			float zdiff = worker_data.gyro_report_0.z - worker_data.gyro_scale[0].z_offset;

			/* maximum allowable calibration error in radians */
			const float maxoff = 0.0055f;

			if (!isfinite(worker_data.gyro_scale[0].x_offset) ||
			    !isfinite(worker_data.gyro_scale[0].y_offset) ||
			    !isfinite(worker_data.gyro_scale[0].z_offset) ||
			    fabsf(xdiff) > maxoff ||
			    fabsf(ydiff) > maxoff ||
			    fabsf(zdiff) > maxoff) {

				mavlink_and_console_log_critical(mavlink_fd, "[cal] motion, retrying..");
				res = ERROR;

			} else {
				res = OK;
			}
		}
		try_count++;

	} while (res == ERROR && try_count <= max_tries);

	if (try_count >= max_tries) {
		mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: Motion during calibration");
		res = ERROR;
	}

	calibrate_cancel_unsubscribe(cancel_sub);
	
	for (unsigned s = 0; s < max_gyros; s++) {
		close(worker_data.gyro_sensor_sub[s]);
	}

	if (res == OK) {
		/* set offset parameters to new values */
		bool failed = false;

		for (unsigned s = 0; s < max_gyros; s++) {
			if (worker_data.device_id[s] != 0) {
				char str[30];
				
				(void)sprintf(str, "CAL_GYRO%u_XOFF", s);
				failed |= (OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[s].x_offset)));
				(void)sprintf(str, "CAL_GYRO%u_YOFF", s);
				failed |= (OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[s].y_offset)));
				(void)sprintf(str, "CAL_GYRO%u_ZOFF", s);
				failed |= (OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[s].z_offset)));
				(void)sprintf(str, "CAL_GYRO%u_ID", s);
				failed |= (OK != param_set_no_notification(param_find(str), &(worker_data.device_id[s])));

				/* apply new scaling and offsets */
				(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);
				int fd = open(str, 0);

				if (fd < 0) {
					failed = true;
					continue;
				}

				res = ioctl(fd, GYROIOCSSCALE, (long unsigned int)&worker_data.gyro_scale[s]);
				close(fd);

				if (res != OK) {
					mavlink_log_critical(mavlink_fd, CAL_ERROR_APPLY_CAL_MSG);
				}
			}
		}

		if (failed) {
			mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: failed to set offset params");
			res = ERROR;
		}
	}

	/* store board ID */
	uint32_t mcu_id[3];
	mcu_unique_id(&mcu_id[0]);

	/* store last 32bit number - not unique, but unique in a given set */
	(void)param_set(param_find("CAL_BOARD_ID"), &mcu_id[2]);

	if (res == OK) {
		/* auto-save to EEPROM */
		res = param_save_default();

		if (res != OK) {
			mavlink_log_critical(mavlink_fd, CAL_ERROR_SAVE_PARAMS_MSG);
		}
	}

	/* if there is a any preflight-check system response, let the barrage of messages through */
	usleep(200000);

	if (res == OK) {
		mavlink_log_info(mavlink_fd, CAL_QGC_DONE_MSG, sensor_name);
	} else {
		mavlink_log_info(mavlink_fd, CAL_QGC_FAILED_MSG, sensor_name);
	}

	/* give this message enough time to propagate */
	usleep(600000);
	
	return res;
}
