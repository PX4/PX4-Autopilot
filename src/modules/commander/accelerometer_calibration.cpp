/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file accelerometer_calibration.cpp
 *
 * Implementation of accelerometer calibration.
 *
 * Transform acceleration vector to true orientation, scale and offset
 *
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling transform
 * accel_raw[3]  - raw acceleration vector
 * accel_offs[3] - acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up
 *                        | -g  0  0 |     // nose down
 *                        |  0  g  0 |     // left side down
 *                        |  0 -g  0 |     // right side down
 *                        |  0  0  g |     // on back
 *                        [  0  0 -g ]     // level
 * accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j = 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
 * g = 9.80665
 *
 * ===== Rotation =====
 *
 * Calibrating using model:
 * accel_corr = accel_T_r * (rot * accel_raw - accel_offs_r)
 *
 * Actual correction:
 * accel_corr = rot * accel_T * (accel_raw - accel_offs)
 *
 * Known: accel_T_r, accel_offs_r, rot
 * Unknown: accel_T, accel_offs
 *
 * Solution:
 * accel_T_r * (rot * accel_raw - accel_offs_r) = rot * accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * (rot * accel_raw - accel_offs_r) = accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * rot * accel_raw - rot^-1 * accel_T_r * accel_offs_r = accel_T * accel_raw - accel_T * accel_offs)
 * => accel_T = rot^-1 * accel_T_r * rot
 * => accel_offs = rot^-1 * accel_offs_r
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "accelerometer_calibration.h"
#include "calibration_messages.h"
#include "commander_helper.h"

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <math.h>
#include <float.h>
#include <mathlib/mathlib.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_accel.h>
#include <geo/geo.h>
#include <conversion/rotation.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <mavlink/mavlink_log.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static const char *sensor_name = "accel";

int do_accel_calibration_measurements(int mavlink_fd, float accel_offs[3], float accel_T[3][3]);
int detect_orientation(int mavlink_fd, int sub_sensor_combined);
int read_accelerometer_avg(int sensor_combined_sub, float accel_avg[3], int samples_num);
int mat_invert3(float src[3][3], float dst[3][3]);
int calculate_calibration_values(float accel_ref[6][3], float accel_T[3][3], float accel_offs[3], float g);

int do_accel_calibration(int mavlink_fd)
{
	int fd;

	mavlink_log_info(mavlink_fd, CAL_STARTED_MSG, sensor_name);

	mavlink_log_info(mavlink_fd, "You need to put the system on all six sides");
	sleep(3);
	mavlink_log_info(mavlink_fd, "Follow the instructions on the screen");
	sleep(5);

	struct accel_scale accel_scale = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	int res = OK;

	/* reset all offsets to zero and all scales to one */
	fd = open(ACCEL_DEVICE_PATH, 0);
	res = ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&accel_scale);
	close(fd);

	if (res != OK) {
		mavlink_log_critical(mavlink_fd, CAL_FAILED_RESET_CAL_MSG);
	}

	float accel_offs[3];
	float accel_T[3][3];

	if (res == OK) {
		/* measure and calculate offsets & scales */
		res = do_accel_calibration_measurements(mavlink_fd, accel_offs, accel_T);
	}

	if (res == OK) {
		/* measurements completed successfully, rotate calibration values */
		param_t board_rotation_h = param_find("SENS_BOARD_ROT");
		int32_t board_rotation_int;
		param_get(board_rotation_h, &(board_rotation_int));
		enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;
		math::Matrix<3, 3> board_rotation;
		get_rot_matrix(board_rotation_id, &board_rotation);
		math::Matrix<3, 3> board_rotation_t = board_rotation.transposed();
		math::Vector<3> accel_offs_vec(&accel_offs[0]);
		math::Vector<3> accel_offs_rotated = board_rotation_t *accel_offs_vec;
		math::Matrix<3, 3> accel_T_mat(&accel_T[0][0]);
		math::Matrix<3, 3> accel_T_rotated = board_rotation_t *accel_T_mat * board_rotation;

		accel_scale.x_offset = accel_offs_rotated(0);
		accel_scale.x_scale = accel_T_rotated(0, 0);
		accel_scale.y_offset = accel_offs_rotated(1);
		accel_scale.y_scale = accel_T_rotated(1, 1);
		accel_scale.z_offset = accel_offs_rotated(2);
		accel_scale.z_scale = accel_T_rotated(2, 2);

		/* set parameters */
		if (param_set(param_find("SENS_ACC_XOFF"), &(accel_scale.x_offset))
		    || param_set(param_find("SENS_ACC_YOFF"), &(accel_scale.y_offset))
		    || param_set(param_find("SENS_ACC_ZOFF"), &(accel_scale.z_offset))
		    || param_set(param_find("SENS_ACC_XSCALE"), &(accel_scale.x_scale))
		    || param_set(param_find("SENS_ACC_YSCALE"), &(accel_scale.y_scale))
		    || param_set(param_find("SENS_ACC_ZSCALE"), &(accel_scale.z_scale))) {
			mavlink_log_critical(mavlink_fd, CAL_FAILED_SET_PARAMS_MSG);
			res = ERROR;
		}
	}

	if (res == OK) {
		/* apply new scaling and offsets */
		fd = open(ACCEL_DEVICE_PATH, 0);
		res = ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&accel_scale);
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

int do_accel_calibration_measurements(int mavlink_fd, float accel_offs[3], float accel_T[3][3])
{
	const int samples_num = 2500;
	float accel_ref[6][3];
	bool data_collected[6] = { false, false, false, false, false, false };
	const char *orientation_strs[6] = { "front", "back", "left", "right", "top", "bottom" };

	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	unsigned done_count = 0;
	int res = OK;

	while (true) {
		bool done = true;
		unsigned old_done_count = done_count;
		done_count = 0;

		for (int i = 0; i < 6; i++) {
			if (data_collected[i]) {
				done_count++;

			} else {
				done = false;
			}
		}

		if (old_done_count != done_count) {
			mavlink_log_info(mavlink_fd, CAL_PROGRESS_MSG, sensor_name, 17 * done_count);
		}

		if (done) {
			break;
		}

		/* inform user which axes are still needed */
		mavlink_log_info(mavlink_fd, "pending: %s%s%s%s%s%s",
				 (!data_collected[0]) ? "front " : "",
				 (!data_collected[1]) ? "back " : "",
				 (!data_collected[2]) ? "left " : "",
				 (!data_collected[3]) ? "right " : "",
				 (!data_collected[4]) ? "up " : "",
				 (!data_collected[5]) ? "down " : "");

		/* allow user enough time to read the message */
		sleep(3);

		int orient = detect_orientation(mavlink_fd, sensor_combined_sub);

		if (orient < 0) {
			mavlink_log_info(mavlink_fd, "invalid motion, hold still...");
			sleep(3);
			continue;
		}

		/* inform user about already handled side */
		if (data_collected[orient]) {
			mavlink_log_info(mavlink_fd, "%s side done, rotate to a different side", orientation_strs[orient]);
			sleep(4);
			continue;
		}

		mavlink_log_info(mavlink_fd, "Hold still, starting to measure %s side", orientation_strs[orient]);
		sleep(1);
		read_accelerometer_avg(sensor_combined_sub, &(accel_ref[orient][0]), samples_num);
		mavlink_log_info(mavlink_fd, "result for %s side: [ %.2f %.2f %.2f ]", orientation_strs[orient],
				 (double)accel_ref[orient][0],
				 (double)accel_ref[orient][1],
				 (double)accel_ref[orient][2]);

		data_collected[orient] = true;
		tune_neutral(true);
	}

	close(sensor_combined_sub);

	if (res == OK) {
		/* calculate offsets and transform matrix */
		res = calculate_calibration_values(accel_ref, accel_T, accel_offs, CONSTANTS_ONE_G);

		if (res != OK) {
			mavlink_log_info(mavlink_fd, "ERROR: calibration values calculation error");
		}
	}

	return res;
}

/*
 * Wait for vehicle become still and detect it's orientation.
 *
 * @return 0..5 according to orientation when vehicle is still and ready for measurements,
 * ERROR if vehicle is not still after 30s or orientation error is more than 5m/s^2
 */
int detect_orientation(int mavlink_fd, int sub_sensor_combined)
{
	struct sensor_combined_s sensor;
	/* exponential moving average of accel */
	float accel_ema[3] = { 0.0f, 0.0f, 0.0f };
	/* max-hold dispersion of accel */
	float accel_disp[3] = { 0.0f, 0.0f, 0.0f };
	/* EMA time constant in seconds*/
	float ema_len = 0.5f;
	/* set "still" threshold to 0.25 m/s^2 */
	float still_thr2 = pow(0.25f, 2);
	/* set accel error threshold to 5m/s^2 */
	float accel_err_thr = 5.0f;
	/* still time required in us */
	hrt_abstime still_time = 2000000;
	struct pollfd fds[1];
	fds[0].fd = sub_sensor_combined;
	fds[0].events = POLLIN;

	hrt_abstime t_start = hrt_absolute_time();
	/* set timeout to 30s */
	hrt_abstime timeout = 30000000;
	hrt_abstime t_timeout = t_start + timeout;
	hrt_abstime t = t_start;
	hrt_abstime t_prev = t_start;
	hrt_abstime t_still = 0;

	unsigned poll_errcount = 0;

	while (true) {
		/* wait blocking for new data */
		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(sensor_combined), sub_sensor_combined, &sensor);
			t = hrt_absolute_time();
			float dt = (t - t_prev) / 1000000.0f;
			t_prev = t;
			float w = dt / ema_len;

			for (int i = 0; i < 3; i++) {
				float d = sensor.accelerometer_m_s2[i] - accel_ema[i];
				accel_ema[i] += d * w;
				d = d * d;
				accel_disp[i] = accel_disp[i] * (1.0f - w);

				if (d > still_thr2 * 8.0f) {
					d = still_thr2 * 8.0f;
				}

				if (d > accel_disp[i]) {
					accel_disp[i] = d;
				}
			}

			/* still detector with hysteresis */
			if (accel_disp[0] < still_thr2 &&
			    accel_disp[1] < still_thr2 &&
			    accel_disp[2] < still_thr2) {
				/* is still now */
				if (t_still == 0) {
					/* first time */
					mavlink_log_info(mavlink_fd, "detected rest position, hold still...");
					t_still = t;
					t_timeout = t + timeout;

				} else {
					/* still since t_still */
					if (t > t_still + still_time) {
						/* vehicle is still, exit from the loop to detection of its orientation */
						break;
					}
				}

			} else if (accel_disp[0] > still_thr2 * 4.0f ||
				   accel_disp[1] > still_thr2 * 4.0f ||
				   accel_disp[2] > still_thr2 * 4.0f) {
				/* not still, reset still start time */
				if (t_still != 0) {
					mavlink_log_info(mavlink_fd, "detected motion, hold still...");
					sleep(3);
					t_still = 0;
				}
			}

		} else if (poll_ret == 0) {
			poll_errcount++;
		}

		if (t > t_timeout) {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {
			mavlink_log_critical(mavlink_fd, CAL_FAILED_SENSOR_MSG);
			return ERROR;
		}
	}

	if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return 0;        // [ g, 0, 0 ]
	}

	if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return 1;        // [ -g, 0, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return 2;        // [ 0, g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return 3;        // [ 0, -g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) {
		return 4;        // [ 0, 0, g ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) {
		return 5;        // [ 0, 0, -g ]
	}

	mavlink_log_critical(mavlink_fd, "ERROR: invalid orientation");

	return ERROR;	// Can't detect orientation
}

/*
 * Read specified number of accelerometer samples, calculate average and dispersion.
 */
int read_accelerometer_avg(int sensor_combined_sub, float accel_avg[3], int samples_num)
{
	struct pollfd fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;
	int count = 0;
	float accel_sum[3] = { 0.0f, 0.0f, 0.0f };

	int errcount = 0;

	while (count < samples_num) {
		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret == 1) {
			struct sensor_combined_s sensor;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

			for (int i = 0; i < 3; i++) {
				accel_sum[i] += sensor.accelerometer_m_s2[i];
			}

			count++;

		} else {
			errcount++;
			continue;
		}

		if (errcount > samples_num / 10) {
			return ERROR;
		}
	}

	for (int i = 0; i < 3; i++) {
		accel_avg[i] = accel_sum[i] / count;
	}

	return OK;
}

int mat_invert3(float src[3][3], float dst[3][3])
{
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
		    src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
		    src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	if (fabsf(det) < FLT_EPSILON) {
		return ERROR;        // Singular matrix
	}

	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

	return OK;
}

int calculate_calibration_values(float accel_ref[6][3], float accel_T[3][3], float accel_offs[3], float g)
{
	/* calculate offsets */
	for (int i = 0; i < 3; i++) {
		accel_offs[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;
	}

	/* fill matrix A for linear equations system*/
	float mat_A[3][3];
	memset(mat_A, 0, sizeof(mat_A));

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float a = accel_ref[i * 2][j] - accel_offs[j];
			mat_A[i][j] = a;
		}
	}

	/* calculate inverse matrix for A */
	float mat_A_inv[3][3];

	if (mat_invert3(mat_A, mat_A_inv) != OK) {
		return ERROR;
	}

	/* copy results to accel_T */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			accel_T[j][i] = mat_A_inv[j][i] * g;
		}
	}

	return OK;
}
