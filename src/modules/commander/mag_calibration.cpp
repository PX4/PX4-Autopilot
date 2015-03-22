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

int calibrate_instance(int mavlink_fd, unsigned cur_mag, unsigned device_id)
{
	int result = OK;
	
	const unsigned int calibration_interval_perside_seconds = 20;
	const uint64_t calibration_interval_perside_useconds = calibration_interval_perside_seconds * 1000 * 1000;

	const unsigned int calibration_sides = 3;
	const unsigned int calibration_points_perside = 80;
	const unsigned int calibration_points_maxcount = calibration_sides * calibration_points_perside;
	unsigned int calibration_counter_total = 0;
	unsigned int calibration_counter_side;

	char str[30];
	
	float* x = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
	float* y = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
	float* z = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));

	if (x == NULL || y == NULL || z == NULL) {
		mavlink_and_console_log_critical(mavlink_fd, "ERROR: out of memory");
		return ERROR;
	}
	
	// Setup subscriptions to mag and onboard accel sensors
	// FIXME: Is it ok to assume first accel is onboard accel
	
	int sub_accel = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	if (sub_accel < 0) {
		mavlink_and_console_log_critical(mavlink_fd, "No onboard accel found, abort");
		result = ERROR;
	}
	
	int sub_mag = orb_subscribe_multi(ORB_ID(sensor_mag), cur_mag);
	if (sub_mag < 0) {
		mavlink_and_console_log_critical(mavlink_fd, "No mag found, abort");
		result = ERROR;
	}
	
	// FIXME: Worker routine for accel orientation detection
	
	if (result == OK) {
		unsigned orientation_failures = 0;
		
		// We only need to collect information from the three main sides. The reverse orientation of
		// those sides is not needed since it would create the same points around the sphere.
		bool side_data_collected[detect_orientation_side_count] = { false, true, false, true, false, true };
		
		// Rotate through all three main positions
		while (true) {
			if (orientation_failures > 10) {
				result = ERROR;
				mavlink_and_console_log_info(mavlink_fd, CAL_FAILED_ORIENTATION_TIMEOUT);
				break;
			}
			
			unsigned int side_complete_count = 0;
			
			// Update the number of completed sides
			for (unsigned i = 0; i < detect_orientation_side_count; i++) {
				if (side_data_collected[i]) {
					side_complete_count++;
				}
			}
			
			if (side_complete_count == detect_orientation_side_count) {
				// We have completed all sides, move on
				break;
			}
			
			// FIXME: Worker routine for pending string creation
			
			/* inform user which axes are still needed */
			char pendingStr[256];
			pendingStr[0] = 0;
			
			for (unsigned int cur_orientation=0; cur_orientation<detect_orientation_side_count; cur_orientation++) {
				if (!side_data_collected[cur_orientation]) {
					strcat(pendingStr, " ");
					strcat(pendingStr, detect_orientation_str((enum detect_orientation_return)cur_orientation));
				}
			}
			mavlink_and_console_log_info(mavlink_fd, "pending:%s", pendingStr);
			
			mavlink_and_console_log_info(mavlink_fd, "hold the vehicle still in one of the pending orientations");
			enum detect_orientation_return orient = detect_orientation(mavlink_fd, sub_accel);
			
			if (orient == DETECT_ORIENTATION_ERROR) {
				orientation_failures++;
				mavlink_and_console_log_info(mavlink_fd, "invalid motion, hold still...");
				continue;
			}
			
			/* inform user about already handled side */
			if (side_data_collected[orient]) {
				orientation_failures++;
				mavlink_and_console_log_info(mavlink_fd, "%s side already done, rotate to a pending orientation", detect_orientation_str(orient));
				continue;
			}
			
			mavlink_and_console_log_info(mavlink_fd, "%s orientation detected", detect_orientation_str(orient));
			mavlink_and_console_log_info(mavlink_fd, "Rotate vehicle around the detected orientation");
			mavlink_and_console_log_info(mavlink_fd, "Continue rotation for %u seconds", calibration_interval_perside_seconds);
			orientation_failures = 0;
			sleep(2);
			
			// Rotation for mag calibration goes here
			struct mag_report mag;
			
			/* limit update rate to get equally spaced measurements over time (in ms) */
			unsigned int orb_interval_msecs = (calibration_interval_perside_useconds / 1000) / calibration_points_perside;
			//mavlink_and_console_log_info(mavlink_fd, "Orb interval %u msecs", orb_interval_msecs);
			orb_set_interval(sub_mag, orb_interval_msecs);
			
			/* calibrate offsets */
			uint64_t calibration_deadline = hrt_absolute_time() + calibration_interval_perside_useconds;
			unsigned poll_errcount = 0;
			
			calibration_counter_side = 0;
			
			while (hrt_absolute_time() < calibration_deadline &&
			       calibration_counter_side < calibration_points_perside) {
				
				/* wait blocking for new data */
				struct pollfd fds[1];
				fds[0].fd = sub_mag;
				fds[0].events = POLLIN;
				
				int poll_ret = poll(fds, 1, 1000);
				
				if (poll_ret > 0) {
					orb_copy(ORB_ID(sensor_mag), sub_mag, &mag);
					
					x[calibration_counter_total] = mag.x;
					y[calibration_counter_total] = mag.y;
					z[calibration_counter_total] = mag.z;
					
					calibration_counter_total++;
					calibration_counter_side++;
					
#if 0
					// FIXME: Check total progress percentage
					if (calibration_counter % (calibration_points_perside / 20) == 0) {
						mavlink_and_console_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 20 + (calibration_counter * 50) / calibration_maxcount);
					}
#endif
					// Progress indicator for side
					mavlink_and_console_log_info(mavlink_fd, "%s side calibration: progress <%u>", sensor_name, (unsigned)(100 * ((float)calibration_counter_side / (float)calibration_points_perside)))
				} else {
					poll_errcount++;
				}
				
				// FIXME: How does this error count relate to poll interval? Seems to high.
				// Seems like it should be some percentage of total points captured.
				if (poll_errcount > 1000) {
					result = ERROR;
					mavlink_and_console_log_info(mavlink_fd, CAL_FAILED_SENSOR_MSG);
					break;
				}
			}

			// Note that this side is complete
			side_data_collected[orient] = true;
			tune_neutral(true);
		}
	}
	
	// Sensor subcriptions are no longer needed
	if (sub_mag >= 0) {
		close(sub_mag);
	}
	if (sub_accel >= 0) {
		close(sub_accel);
	}

	// FIXME: Check as to how this happens?
	if (result == OK && calibration_counter_total < (calibration_points_maxcount / 2)) {
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
		sphere_fit_least_squares(x, y, z, calibration_counter_total, 100, 0.0f, &sphere_x, &sphere_y, &sphere_z, &sphere_radius);
		mavlink_and_console_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 80);

		if (!isfinite(sphere_x) || !isfinite(sphere_y) || !isfinite(sphere_z)) {
			mavlink_and_console_log_info(mavlink_fd, "ERROR: NaN in sphere fit");
			result = ERROR;
		}
	}
	
	// Data points are no longer needed
	free(x);
	free(y);
	free(z);
	
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
		failed |= (OK != param_set(param_find(str), &(device_id)));
		(void)sprintf(str, "CAL_MAG%u_XOFF", cur_mag);
		failed |= (OK != param_set(param_find(str), &(mscale.x_offset)));
		(void)sprintf(str, "CAL_MAG%u_YOFF", cur_mag);
		failed |= (OK != param_set(param_find(str), &(mscale.y_offset)));
		(void)sprintf(str, "CAL_MAG%u_ZOFF", cur_mag);
		failed |= (OK != param_set(param_find(str), &(mscale.z_offset)));
		(void)sprintf(str, "CAL_MAG%u_XSCALE", cur_mag);
		failed |= (OK != param_set(param_find(str), &(mscale.x_scale)));
		(void)sprintf(str, "CAL_MAG%u_YSCALE", cur_mag);
		failed |= (OK != param_set(param_find(str), &(mscale.y_scale)));
		(void)sprintf(str, "CAL_MAG%u_ZSCALE", cur_mag);
		failed |= (OK != param_set(param_find(str), &(mscale.z_scale)));

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
