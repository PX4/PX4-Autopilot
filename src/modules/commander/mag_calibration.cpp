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
static const unsigned max_mags = 3;

int mag_calibrate_all(int mavlink_fd, int32_t (&device_ids)[max_mags]);
int mag_calibration_worker(detect_orientation_return orientation, void* worker_data);

/// Data passed to calibration worker routine
typedef struct  {
	int		mavlink_fd;
	unsigned	done_count;
	int		sub_mag[max_mags];
	unsigned int	calibration_points_perside;
	unsigned int	calibration_interval_perside_seconds;
	uint64_t	calibration_interval_perside_useconds;
	unsigned int	calibration_counter_total;
	bool		side_data_collected[detect_orientation_side_count];
	float*		x[max_mags];
	float*		y[max_mags];
	float*		z[max_mags];
} mag_worker_data_t;


int do_mag_calibration(int mavlink_fd)
{
	mavlink_and_console_log_info(mavlink_fd, CAL_STARTED_MSG, sensor_name);

	struct mag_scale mscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	int result = OK;
	
	// Determine which mags are available and reset each

	int32_t	device_ids[max_mags];
	char str[30];

	for (size_t i=0; i<max_mags; i++) {
		device_ids[i] = 0; // signals no mag
	}
	
	for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {
		// Reset mag id to mag not available
		(void)sprintf(str, "CAL_MAG%u_ID", cur_mag);
		result = param_set_no_notification(param_find(str), &(device_ids[cur_mag]));
		if (result != OK) {
			mavlink_and_console_log_info(mavlink_fd, "Unabled to reset CAL_MAG%u_ID", cur_mag);
			break;
		}

		// Attempt to open mag
		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
		int fd = open(str, O_RDONLY);
		if (fd < 0) {
			continue;
		}

		// Get device id for this mag
		device_ids[cur_mag] = ioctl(fd, DEVIOCGDEVICEID, 0);

		// Reset mag scale
		result = ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale_null);

		if (result != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_FAILED_RESET_CAL_MSG, cur_mag);
		}

		if (result == OK) {
			/* calibrate range */
			result = ioctl(fd, MAGIOCCALIBRATE, fd);

			if (result != OK) {
				mavlink_and_console_log_info(mavlink_fd, "Skipped scale calibration, sensor %u", cur_mag);
				/* this is non-fatal - mark it accordingly */
				result = OK;
			}
		}

		close(fd);
	}

	if (result == OK) {
		// Calibrate all mags at the same time
		result = mag_calibrate_all(mavlink_fd, device_ids);
	}
	
	if (result == OK) {
		/* auto-save to EEPROM */
		result = param_save_default();
		if (result != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_FAILED_SAVE_PARAMS_MSG);
		}
	}
	
	if (result == OK) {
		mavlink_and_console_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 100);
		mavlink_and_console_log_info(mavlink_fd, CAL_DONE_MSG, sensor_name);
	} else {
		mavlink_and_console_log_critical(mavlink_fd, CAL_FAILED_MSG, sensor_name);
	}

	return result;
}

int mag_calibration_worker(detect_orientation_return orientation, void* data)
{
	int result = OK;
	
	unsigned int calibration_counter_side;

	mag_worker_data_t* worker_data = (mag_worker_data_t*)(data);
	
	mavlink_and_console_log_info(worker_data->mavlink_fd, "Rotate vehicle around the detected orientation");
	mavlink_and_console_log_info(worker_data->mavlink_fd, "Continue rotation for %u seconds", worker_data->calibration_interval_perside_seconds);
	sleep(2);
	
	uint64_t calibration_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_useconds;
	unsigned poll_errcount = 0;
	
	calibration_counter_side = 0;
	
	while (hrt_absolute_time() < calibration_deadline &&
	       calibration_counter_side < worker_data->calibration_points_perside) {
		
		// Wait clocking for new data on all mags
		struct pollfd fds[max_mags];
		size_t fd_count = 0;
		for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (worker_data->sub_mag[cur_mag] >= 0) {
				fds[fd_count].fd = worker_data->sub_mag[cur_mag];
				fds[fd_count].events = POLLIN;
				fd_count++;
			}
		}
		int poll_ret = poll(fds, fd_count, 1000);
		
		if (poll_ret > 0) {
			for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
				if (worker_data->sub_mag[cur_mag] >= 0) {
					struct mag_report mag;

					orb_copy(ORB_ID(sensor_mag), worker_data->sub_mag[cur_mag], &mag);
					
					worker_data->x[cur_mag][worker_data->calibration_counter_total] = mag.x;
					worker_data->y[cur_mag][worker_data->calibration_counter_total] = mag.y;
					worker_data->z[cur_mag][worker_data->calibration_counter_total] = mag.z;
					
				}
			}
			
			worker_data->calibration_counter_total++;
			calibration_counter_side++;
			
			// Progress indicator for side
			mavlink_and_console_log_info(worker_data->mavlink_fd,
						     "%s %s side calibration: progress <%u>",
						     sensor_name,
						     detect_orientation_str(orientation),
						     (unsigned)(100 * ((float)calibration_counter_side / (float)worker_data->calibration_points_perside)));
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
	detect_orientation_return alternateOrientation = orientation;
	switch (orientation) {
		case DETECT_ORIENTATION_TAIL_DOWN:
			alternateOrientation = DETECT_ORIENTATION_NOSE_DOWN;
			break;
		case DETECT_ORIENTATION_NOSE_DOWN:
			alternateOrientation = DETECT_ORIENTATION_TAIL_DOWN;
			break;
		case DETECT_ORIENTATION_LEFT:
			alternateOrientation = DETECT_ORIENTATION_RIGHT;
			break;
		case DETECT_ORIENTATION_RIGHT:
			alternateOrientation = DETECT_ORIENTATION_LEFT;
			break;
		case DETECT_ORIENTATION_UPSIDE_DOWN:
			alternateOrientation = DETECT_ORIENTATION_RIGHTSIDE_UP;
			break;
		case DETECT_ORIENTATION_RIGHTSIDE_UP:
			alternateOrientation = DETECT_ORIENTATION_UPSIDE_DOWN;
			break;
		case DETECT_ORIENTATION_ERROR:
			warnx("Invalid orientation in mag_calibration_worker");
			break;
	}
	worker_data->side_data_collected[alternateOrientation] = true;
	mavlink_and_console_log_info(worker_data->mavlink_fd, "%s side done, rotate to a different side", detect_orientation_str(alternateOrientation));
	
	worker_data->done_count++;
	mavlink_and_console_log_info(worker_data->mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 34 * worker_data->done_count);
	
	return result;
}

int mag_calibrate_all(int mavlink_fd, int32_t (&device_ids)[max_mags])
{
	int result = OK;

	mag_worker_data_t worker_data;
	
	worker_data.mavlink_fd = mavlink_fd;
	worker_data.done_count = 0;
	worker_data.calibration_counter_total = 0;
	worker_data.calibration_points_perside = 80;
	worker_data.calibration_interval_perside_seconds = 20;
	worker_data.calibration_interval_perside_useconds = worker_data.calibration_interval_perside_seconds * 1000 * 1000;

	// Initialize to collect all sides
	for (size_t cur_side=0; cur_side<6; cur_side++) {
		worker_data.side_data_collected[cur_side] = false;
	}
	
	for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
		// Initialize to no subscription
		worker_data.sub_mag[cur_mag] = -1;
		
		// Initialize to no memory allocated
		worker_data.x[cur_mag] = NULL;
		worker_data.y[cur_mag] = NULL;
		worker_data.z[cur_mag] = NULL;
	}

	const unsigned int calibration_sides = 3;
	const unsigned int calibration_points_maxcount = calibration_sides * worker_data.calibration_points_perside;
	
	char str[30];
	
	for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
		worker_data.x[cur_mag] = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
		worker_data.y[cur_mag] = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
		worker_data.z[cur_mag] = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
		if (worker_data.x[cur_mag] == NULL || worker_data.y[cur_mag] == NULL || worker_data.z[cur_mag] == NULL) {
			mavlink_and_console_log_critical(mavlink_fd, "ERROR: out of memory");
			result = ERROR;
		}
	}

	
	// Setup subscriptions to mag sensors
	if (result == OK) {
		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available
				worker_data.sub_mag[cur_mag] = orb_subscribe_multi(ORB_ID(sensor_mag), cur_mag);
				if (worker_data.sub_mag[cur_mag] < 0) {
					mavlink_and_console_log_critical(mavlink_fd, "Mag #%u not found, abort", cur_mag);
					result = ERROR;
					break;
				}
			}
		}
	}
	
	// Limit update rate to get equally spaced measurements over time (in ms)
	if (result == OK) {
		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available
				unsigned int orb_interval_msecs = (worker_data.calibration_interval_perside_useconds / 1000) / worker_data.calibration_points_perside;
				
				//mavlink_and_console_log_info(mavlink_fd, "Orb interval %u msecs", orb_interval_msecs);
				orb_set_interval(worker_data.sub_mag[cur_mag], orb_interval_msecs);
			}
		}
		
	}

	result = calibrate_from_orientation(mavlink_fd, worker_data.side_data_collected, mag_calibration_worker, &worker_data);
	
	// Close subscriptions
	for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
		if (worker_data.sub_mag[cur_mag] >= 0) {
			close(worker_data.sub_mag[cur_mag]);
		}
	}
	
	// Calculate calibration values for each mag
	
	
	float sphere_x[max_mags];
	float sphere_y[max_mags];
	float sphere_z[max_mags];
	float sphere_radius[max_mags];
	
	// Sphere fit the data to get calibration values
	if (result == OK) {
		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available and we should have values for it to calibrate
				
				sphere_fit_least_squares(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
							 worker_data.calibration_counter_total,
							 100, 0.0f,
							 &sphere_x[cur_mag], &sphere_y[cur_mag], &sphere_z[cur_mag],
							 &sphere_radius[cur_mag]);
				
				if (!isfinite(sphere_x[cur_mag]) || !isfinite(sphere_y[cur_mag]) || !isfinite(sphere_z[cur_mag])) {
					mavlink_and_console_log_info(mavlink_fd, "ERROR: NaN in sphere fit for mag #%u", cur_mag);
					result = ERROR;
				}
			}
		}
	}
	
	// Data points are no longer needed
	for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
		free(worker_data.x[cur_mag]);
		free(worker_data.y[cur_mag]);
		free(worker_data.z[cur_mag]);
	}
	
	if (result == OK) {
		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				int fd_mag = -1;
				struct mag_scale mscale;
				
				// Set new scale
				
				(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
				fd_mag = open(str, 0);
				if (fd_mag < 0) {
					mavlink_and_console_log_info(mavlink_fd, "ERROR: unable to open mag device #%u", cur_mag);
					result = ERROR;
				}
				
				if (result == OK) {
					result = ioctl(fd_mag, MAGIOCGSCALE, (long unsigned int)&mscale);
					if (result != OK) {
						mavlink_and_console_log_info(mavlink_fd, "ERROR: failed to get current calibration #%u", cur_mag);
						result = ERROR;
					}
				}

				if (result == OK) {
					mscale.x_offset = sphere_x[cur_mag];
					mscale.y_offset = sphere_y[cur_mag];
					mscale.z_offset = sphere_z[cur_mag];

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
					failed |= (OK != param_set_no_notification(param_find(str), &(device_ids[cur_mag])));
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
						mavlink_and_console_log_info(mavlink_fd, CAL_FAILED_SET_PARAMS_MSG, cur_mag);
						result = ERROR;
					} else {
						mavlink_and_console_log_info(mavlink_fd, "mag #%u off: x:%.2f y:%.2f z:%.2f Ga",
									     cur_mag,
									     (double)mscale.x_offset, (double)mscale.y_offset, (double)mscale.z_offset);
						mavlink_and_console_log_info(mavlink_fd, "mag #%u scale: x:%.2f y:%.2f z:%.2f",
									     cur_mag,
									     (double)mscale.x_scale, (double)mscale.y_scale, (double)mscale.z_scale);
					}
				}
			}
		}
	}

	return result;
}
