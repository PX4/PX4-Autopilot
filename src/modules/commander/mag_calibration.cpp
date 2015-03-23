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
 * @file mag_calibration.cpp
 *
 * Magnetometer calibration routine
 */

#include "mag_calibration.h"
#include "commander_helper.h"
#include "calibration_routines.h"
#include "calibration_messages.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <math.h>
#include <fcntl.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_mag.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static const char *sensor_name = "mag";

int calibrate_instance(int mavlink_fd, unsigned cur_mag, unsigned device_id);
int mag_calibration_worker(detect_orientation_return orientation, void* worker_data);

/// Data passed to calibration worker routine
typedef struct  {
	int		mavlink_fd;
	unsigned	done_count;
	int		sub_mag;
	unsigned int	calibration_points_perside;
	unsigned int	calibration_interval_perside_seconds;
	uint64_t	calibration_interval_perside_useconds;
	unsigned int	calibration_counter_total;
	bool		side_data_collected[detect_orientation_side_count];
	float*		x;
	float*		y;
	float*		z;
} mag_worker_data_t;


int do_mag_calibration(int mavlink_fd)
{
	const unsigned max_mags = 3;

	int32_t device_id[max_mags];
	mavlink_and_console_log_info(mavlink_fd, CAL_STARTED_MSG, sensor_name);

	struct mag_scale mscale_null[max_mags] = {
	{
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	}
	} ;

	int res = ERROR;

	char str[30];

	unsigned calibrated_ok = 0;

	for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {

		/* erase old calibration */
		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
		int fd = open(str, O_RDONLY);

		if (fd < 0) {
			continue;
		}

		mavlink_and_console_log_info(mavlink_fd, "Calibrating magnetometer #%u %s", cur_mag, cur_mag == 0  ? "(onboard)" : "(external)" );

		device_id[cur_mag] = ioctl(fd, DEVIOCGDEVICEID, 0);

		/* ensure all scale fields are initialized tha same as the first struct */
		(void)memcpy(&mscale_null[cur_mag], &mscale_null[0], sizeof(mscale_null[0]));

		res = ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale_null[cur_mag]);

		if (res != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_FAILED_RESET_CAL_MSG);
		}

		if (res == OK) {
			/* calibrate range */
			res = ioctl(fd, MAGIOCCALIBRATE, fd);

			if (res != OK) {
				mavlink_and_console_log_info(mavlink_fd, "Skipped scale calibration");
				/* this is non-fatal - mark it accordingly */
				res = OK;
			}
		}

		close(fd);

		if (res == OK) {
			res = calibrate_instance(mavlink_fd, cur_mag, device_id[cur_mag]);

			if (res == OK) {
				calibrated_ok++;
			}
		}
	}

	if (calibrated_ok) {

		mavlink_and_console_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 100);
		usleep(100000);
		mavlink_and_console_log_info(mavlink_fd, CAL_DONE_MSG, sensor_name);

		/* auto-save to EEPROM */
		res = param_save_default();

		if (res != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_FAILED_SAVE_PARAMS_MSG);
		}
	} else {
		mavlink_and_console_log_critical(mavlink_fd, CAL_FAILED_MSG, sensor_name);
	}

	return res;
}

int mag_calibration_worker(detect_orientation_return orientation, void* data)
{
	int result = OK;
	
	unsigned int calibration_counter_side;

	mag_worker_data_t* worker_data = (mag_worker_data_t*)(data);
	
	mavlink_and_console_log_info(worker_data->mavlink_fd, "Rotate vehicle around the detected orientation");
	mavlink_and_console_log_info(worker_data->mavlink_fd, "Continue rotation for %u seconds", worker_data->calibration_interval_perside_seconds);
	sleep(2);
	
	// Rotation for mag calibration goes here
	struct mag_report mag;
	
	
	/* calibrate offsets */
	uint64_t calibration_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_useconds;
	unsigned poll_errcount = 0;
	
	calibration_counter_side = 0;
	
	while (hrt_absolute_time() < calibration_deadline &&
	       calibration_counter_side < worker_data->calibration_points_perside) {
		
		/* wait blocking for new data */
		struct pollfd fds[1];
		fds[0].fd = worker_data->sub_mag;
		fds[0].events = POLLIN;
		
		int poll_ret = poll(fds, 1, 1000);
		
		if (poll_ret > 0) {
			orb_copy(ORB_ID(sensor_mag), worker_data->sub_mag, &mag);
			
			worker_data->x[worker_data->calibration_counter_total] = mag.x;
			worker_data->y[worker_data->calibration_counter_total] = mag.y;
			worker_data->z[worker_data->calibration_counter_total] = mag.z;
			
			worker_data->calibration_counter_total++;
			calibration_counter_side++;
			
			// Progress indicator for side
			mavlink_and_console_log_info(worker_data->mavlink_fd,
						     "%s %s calibration: progress <%u>",
						     sensor_name,
						     detect_orientation_str(orientation),
						     (unsigned)(100 * ((float)calibration_counter_side / (float)worker_data->calibration_points_perside)))
		} else {
			poll_errcount++;
		}
		
		if (poll_errcount > worker_data->calibration_points_perside * 3) {
			result = ERROR;
			mavlink_and_console_log_info(worker_data->mavlink_fd, CAL_FAILED_SENSOR_MSG);
			break;
		}
	}
	
	// Mark the opposite side as collected as well. No need to collect opposite side since it
	// would generate similar points.
	switch (orientation) {
		case DETECT_ORIENTATION_TAIL_DOWN:
			worker_data->side_data_collected[DETECT_ORIENTATION_NOSE_DOWN] = true;
			break;
		case DETECT_ORIENTATION_NOSE_DOWN:
			worker_data->side_data_collected[DETECT_ORIENTATION_TAIL_DOWN] = true;
			break;
		case DETECT_ORIENTATION_LEFT:
			worker_data->side_data_collected[DETECT_ORIENTATION_RIGHT] = true;
			break;
		case DETECT_ORIENTATION_RIGHT:
			worker_data->side_data_collected[DETECT_ORIENTATION_LEFT] = true;
			break;
		case DETECT_ORIENTATION_UPSIDE_DOWN:
			worker_data->side_data_collected[DETECT_ORIENTATION_RIGHTSIDE_UP] = true;
			break;
		case DETECT_ORIENTATION_RIGHTSIDE_UP:
			worker_data->side_data_collected[DETECT_ORIENTATION_UPSIDE_DOWN] = true;
			break;
		case DETECT_ORIENTATION_ERROR:
			warnx("Invalid orientation in mag_calibration_worker");
			break;
	}
	
	worker_data->done_count++;
	mavlink_and_console_log_info(worker_data->mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 34 * worker_data->done_count);
	
	return result;
}

int calibrate_instance(int mavlink_fd, unsigned cur_mag, unsigned device_id)
{
	int result = OK;

	mag_worker_data_t worker_data;
	
	worker_data.mavlink_fd = mavlink_fd;
	worker_data.done_count = 0;
	worker_data.calibration_counter_total = 0;
	worker_data.calibration_points_perside = 80;
	worker_data.calibration_interval_perside_seconds = 20;
	worker_data.calibration_interval_perside_useconds = worker_data.calibration_interval_perside_seconds * 1000 * 1000;
	for (size_t i=0; i<6; i++) {
		worker_data.side_data_collected[i] = false;
	}

	const unsigned int calibration_sides = 3;
	const unsigned int calibration_points_maxcount = calibration_sides * worker_data.calibration_points_perside;
	
	char str[30];
	
	worker_data.x = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
	worker_data.y = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
	worker_data.z = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));

	if (worker_data.x == NULL || worker_data.y == NULL || worker_data.z == NULL) {
		mavlink_and_console_log_critical(mavlink_fd, "ERROR: out of memory");
		result = ERROR;
	}
	
	// Setup subscriptions to mag sensor
	if (result == OK) {
		worker_data.sub_mag = orb_subscribe_multi(ORB_ID(sensor_mag), cur_mag);
		if (worker_data.sub_mag < 0) {
			mavlink_and_console_log_critical(mavlink_fd, "No mag found, abort");
			result = ERROR;
		}
	}
	
	if (result == OK) {
		/* limit update rate to get equally spaced measurements over time (in ms) */
		unsigned int orb_interval_msecs = (worker_data.calibration_interval_perside_useconds / 1000) / worker_data.calibration_points_perside;
		
		//mavlink_and_console_log_info(mavlink_fd, "Orb interval %u msecs", orb_interval_msecs);
		orb_set_interval(worker_data.sub_mag, orb_interval_msecs);
		
		result = calibrate_from_orientation(mavlink_fd, worker_data.side_data_collected, mag_calibration_worker, &worker_data);
	}

	// Sensor subcriptions are no longer needed
	if (worker_data.sub_mag >= 0) {
		close(worker_data.sub_mag);
	}

	// FIXME: Check as to how this happens?
	if (result == OK && worker_data.calibration_counter_total < (calibration_points_maxcount / 2)) {
		mavlink_and_console_log_info(mavlink_fd, "ERROR: Not enough points collected");
		result = ERROR;
	}
	
	float sphere_x;
	float sphere_y;
	float sphere_z;
	float sphere_radius;
	
	if (result == OK) {
		/* sphere fit */
		mavlink_and_console_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 70);
		sphere_fit_least_squares(worker_data.x, worker_data.y, worker_data.z, worker_data.calibration_counter_total, 100, 0.0f, &sphere_x, &sphere_y, &sphere_z, &sphere_radius);
		mavlink_and_console_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 80);

		if (!isfinite(sphere_x) || !isfinite(sphere_y) || !isfinite(sphere_z)) {
			mavlink_and_console_log_info(mavlink_fd, "ERROR: NaN in sphere fit");
			result = ERROR;
		}
	}
	
	// Data points are no longer needed
	free(worker_data.x);
	free(worker_data.y);
	free(worker_data.z);
	
	int fd_mag = -1;
	struct mag_scale mscale;
	
	if (result == OK) {
		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
		
		fd_mag = open(str, 0);
		if (fd_mag < 0) {
			mavlink_and_console_log_info(mavlink_fd, "ERROR: unable to open mag device");
			result = ERROR;
		}
	}
	
	if (result == OK) {
		result = ioctl(fd_mag, MAGIOCGSCALE, (long unsigned int)&mscale);
		if (result != OK) {
			mavlink_and_console_log_info(mavlink_fd, "ERROR: failed to get current calibration");
			result = ERROR;
		}
	}

	if (result == OK) {
		mscale.x_offset = sphere_x;
		mscale.y_offset = sphere_y;
		mscale.z_offset = sphere_z;

		result = ioctl(fd_mag, MAGIOCSSCALE, (long unsigned int)&mscale);
		if (result != OK) {
			mavlink_and_console_log_info(mavlink_fd, CAL_FAILED_APPLY_CAL_MSG, cur_mag);
			result = ERROR;
		}
	}
	
	// Mag device no longer needed
	if (fd_mag >= 0) {
		close(fd_mag);
	}

	if (result == OK) {
		bool failed = false;
		
		/* set parameters */
		(void)sprintf(str, "CAL_MAG%u_ID", cur_mag);
		failed |= (OK != param_set_no_notification(param_find(str), &(device_id)));
		(void)sprintf(str, "CAL_MAG%u_XOFF", cur_mag);
		failed |= (OK != param_set_no_notification(param_find(str), &(mscale.x_offset)));
		(void)sprintf(str, "CAL_MAG%u_YOFF", cur_mag);
		failed |= (OK != param_set_no_notification(param_find(str), &(mscale.y_offset)));
		(void)sprintf(str, "CAL_MAG%u_ZOFF", cur_mag);
		failed |= (OK != param_set_no_notification(param_find(str), &(mscale.z_offset)));
		(void)sprintf(str, "CAL_MAG%u_XSCALE", cur_mag);
		failed |= (OK != param_set_no_notification(param_find(str), &(mscale.x_scale)));
		(void)sprintf(str, "CAL_MAG%u_YSCALE", cur_mag);
		failed |= (OK != param_set_no_notification(param_find(str), &(mscale.y_scale)));
		(void)sprintf(str, "CAL_MAG%u_ZSCALE", cur_mag);
		failed |= (OK != param_set_no_notification(param_find(str), &(mscale.z_scale)));

		if (failed) {
			mavlink_and_console_log_info(mavlink_fd, CAL_FAILED_SET_PARAMS_MSG);
			result = ERROR;
		} else {
			mavlink_and_console_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 90);

			mavlink_and_console_log_info(mavlink_fd, "mag off: x:%.2f y:%.2f z:%.2f Ga", (double)mscale.x_offset,
					 (double)mscale.y_offset, (double)mscale.z_offset);
			mavlink_and_console_log_info(mavlink_fd, "mag scale: x:%.2f y:%.2f z:%.2f", (double)mscale.x_scale,
					 (double)mscale.y_scale, (double)mscale.z_scale);
		}
	}

	return result;
}
