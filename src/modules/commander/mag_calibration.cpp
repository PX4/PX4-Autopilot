/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <matrix/math.hpp>
#include <lib/conversion/rotation.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/err.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>

using namespace matrix;
using namespace time_literals;

static constexpr char sensor_name[] {"mag"};
static constexpr unsigned MAX_MAGS = 4;
static constexpr float MAG_SPHERE_RADIUS = 0.2f;
static constexpr unsigned int calibration_total_points = 240;	///< The total points per magnetometer
static constexpr unsigned int calibraton_duration_s = 42; 	///< The total duration the routine is allowed to take

static constexpr float MAG_MAX_OFFSET_LEN =
	1.3f;	///< The maximum measurement range is ~1.9 Ga, the earth field is ~0.6 Ga, so an offset larger than ~1.3 Ga means the mag will saturate in some directions.

static constexpr uint8_t MAG_DEFAULT_PRIORITY = 50;
static constexpr uint8_t MAG_DEFAULT_EXTERNAL_PRIORITY = 75;

static unsigned calibration_sides = 6;			///< The total number of sides
static unsigned last_mag_progress = 0;

calibrate_return mag_calibrate_all(orb_advert_t *mavlink_log_pub, int32_t cal_mask);

/// Data passed to calibration worker routine
typedef struct  {
	orb_advert_t	*mavlink_log_pub;
	bool		append_to_existing_calibration;
	unsigned	done_count;
	bool		side_data_collected[detect_orientation_side_count];
	unsigned int	calibration_points_perside;
	uint64_t	calibration_interval_perside_us;
	bool		existing_calibration_available[MAX_MAGS];
	unsigned int	calibration_counter_total[MAX_MAGS];
	float		*x[MAX_MAGS];
	float		*y[MAX_MAGS];
	float		*z[MAX_MAGS];
	int32_t		device_ids[MAX_MAGS];
	bool		internal[MAX_MAGS];
	int32_t		priority[MAX_MAGS];
	int32_t		rotation[MAX_MAGS];
	Vector3f	scale_existing[MAX_MAGS];
	Vector3f	offset_existing[MAX_MAGS];
	Vector3f	power_compensation[MAX_MAGS];
} mag_worker_data_t;


int do_mag_calibration(orb_advert_t *mavlink_log_pub)
{
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	int result = PX4_OK;

	char str[30];

	// reset the learned EKF mag in-flight bias offsets which have been learned for the previous
	//  sensor calibration and will be invalidated by a new sensor calibration
	(void)sprintf(str, "EKF2_MAGBIAS_X");
	float x_offset = 0.f;
	result = param_set_no_notification(param_find(str), &x_offset);

	if (result != PX4_OK) {
		PX4_ERR("unable to reset %s", str);
	}

	(void)sprintf(str, "EKF2_MAGBIAS_Y");
	float y_offset = 0.f;
	result = param_set_no_notification(param_find(str), &y_offset);

	if (result != PX4_OK) {
		PX4_ERR("unable to reset %s", str);
	}

	(void)sprintf(str, "EKF2_MAGBIAS_Z");
	float z_offset = 0.f;
	result = param_set_no_notification(param_find(str), &z_offset);

	if (result != PX4_OK) {
		PX4_ERR("unable to reset %s", str);
	}

	last_mag_progress = 0;

	// Collect: As defined by configuration
	// start with a full mask, all six bits set
	int32_t cal_mask = (1 << 6) - 1;
	param_get(param_find("CAL_MAG_SIDES"), &cal_mask);

	// Calibrate all mags at the same time
	if (result == PX4_OK) {
		switch (mag_calibrate_all(mavlink_log_pub, cal_mask)) {
		case calibrate_return_cancelled:
			// Cancel message already displayed, we're done here
			result = PX4_ERROR;
			break;

		case calibrate_return_ok:
			/* if there is a any preflight-check system response, let the barrage of messages through */
			px4_usleep(200000);

			calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 100);
			px4_usleep(20000);
			calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
			px4_usleep(20000);
			break;

		default:
			calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
			px4_usleep(20000);
			break;
		}
	}

	/* give this message enough time to propagate */
	px4_usleep(600000);

	return result;
}

static bool reject_sample(float sx, float sy, float sz, float x[], float y[], float z[], unsigned count,
			  unsigned max_count)
{
	float min_sample_dist = fabsf(5.4f * MAG_SPHERE_RADIUS / sqrtf(max_count)) / 3.0f;

	for (size_t i = 0; i < count; i++) {
		float dx = sx - x[i];
		float dy = sy - y[i];
		float dz = sz - z[i];
		float dist = sqrtf(dx * dx + dy * dy + dz * dz);

		if (dist < min_sample_dist) {
			return true;
		}
	}

	return false;
}

static unsigned progress_percentage(mag_worker_data_t *worker_data)
{
	return 100 * ((float)worker_data->done_count) / calibration_sides;
}

// Returns calibrate_return_error if any parameter is not finite
// Logs if parameters are out of range
static calibrate_return check_calibration_result(float offset_x, float offset_y, float offset_z,
		float sphere_radius,
		float diag_x, float diag_y, float diag_z,
		float offdiag_x, float offdiag_y, float offdiag_z,
		orb_advert_t *mavlink_log_pub, uint8_t cur_mag, bool internal[MAX_MAGS])
{
	float must_be_finite[] = {offset_x, offset_y, offset_z,
				  sphere_radius,
				  diag_x, diag_y, diag_z,
				  offdiag_x, offdiag_y, offdiag_z
				 };

	float should_be_not_huge[] = {offset_x, offset_y, offset_z};
	float should_be_positive[] = {sphere_radius, diag_x, diag_y, diag_z};

	// Make sure every parameter is finite
	const int num_finite = sizeof(must_be_finite) / sizeof(*must_be_finite);

	for (unsigned i = 0; i < num_finite; ++i) {
		if (!PX4_ISFINITE(must_be_finite[i])) {
			calibration_log_emergency(mavlink_log_pub,
						  "ERROR: Retry calibration (sphere NaN, #%u)", cur_mag);
			return calibrate_return_error;
		}
	}

	// Notify if offsets are too large
	const int num_not_huge = sizeof(should_be_not_huge) / sizeof(*should_be_not_huge);

	for (unsigned i = 0; i < num_not_huge; ++i) {
		if (fabsf(should_be_not_huge[i]) > MAG_MAX_OFFSET_LEN) {
			calibration_log_critical(mavlink_log_pub, "Warning: %s mag (#%u) with large offsets",
						 (internal[cur_mag]) ? "internal" : "external", cur_mag);
			break;
		}
	}

	// Notify if a parameter which should be positive is non-positive
	const int num_positive = sizeof(should_be_positive) / sizeof(*should_be_positive);

	for (unsigned i = 0; i < num_positive; ++i) {
		if (should_be_positive[i] <= 0.0f) {
			calibration_log_critical(mavlink_log_pub, "Warning: %s mag (#%u) with non-positive scale",
						 (internal[cur_mag]) ? "internal" : "external", cur_mag);
			break;
		}
	}

	return calibrate_return_ok;
}

static calibrate_return mag_calibration_worker(detect_orientation_return orientation, void *data)
{
	const hrt_abstime calibration_started = hrt_absolute_time();
	calibrate_return result = calibrate_return_ok;

	mag_worker_data_t *worker_data = (mag_worker_data_t *)(data);

	// notify user to start rotating
	set_tune(TONE_SINGLE_BEEP_TUNE);

	calibration_log_info(worker_data->mavlink_log_pub, "[cal] Rotate vehicle around the detected orientation");
	calibration_log_info(worker_data->mavlink_log_pub, "[cal] Continue rotation for %s %.1f s",
			     detect_orientation_str(orientation), worker_data->calibration_interval_perside_us / 1e6);

	/*
	 * Detect if the system is rotating.
	 *
	 * We're detecting this as a general rotation on any axis, not necessary on the one we
	 * asked the user for. This is because we really just need two roughly orthogonal axes
	 * for a good result, so we're not constraining the user more than we have to.
	 */

	const hrt_abstime detection_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_us * 5;
	hrt_abstime last_gyro = 0;
	float gyro_x_integral = 0.0f;
	float gyro_y_integral = 0.0f;
	float gyro_z_integral = 0.0f;

	static constexpr float gyro_int_thresh_rad = 0.5f;

	uORB::SubscriptionBlocking<sensor_gyro_s> gyro_sub{ORB_ID(sensor_gyro)};

	while (fabsf(gyro_x_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_y_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_z_integral) < gyro_int_thresh_rad) {

		/* abort on request */
		if (calibrate_cancel_check(worker_data->mavlink_log_pub, calibration_started)) {
			result = calibrate_return_cancelled;
			return result;
		}

		/* abort with timeout */
		if (hrt_absolute_time() > detection_deadline) {
			result = calibrate_return_error;
			PX4_ERR("gyro int: %8.4f, %8.4f, %8.4f", (double)gyro_x_integral, (double)gyro_y_integral, (double)gyro_z_integral);
			calibration_log_critical(worker_data->mavlink_log_pub, "Failed: This calibration requires rotation.");
			break;
		}

		/* Wait clocking for new data on all gyro */
		sensor_gyro_s gyro;

		if (gyro_sub.updateBlocking(gyro, 1000_ms)) {

			/* ensure we have a valid first timestamp */
			if (last_gyro > 0) {

				/* integrate */
				float delta_t = (gyro.timestamp - last_gyro) / 1e6f;
				gyro_x_integral += gyro.x * delta_t;
				gyro_y_integral += gyro.y * delta_t;
				gyro_z_integral += gyro.z * delta_t;
			}

			last_gyro = gyro.timestamp;
		}
	}

	uint32_t interval_us = worker_data->calibration_interval_perside_us / worker_data->calibration_points_perside;

	uORB::SubscriptionBlocking<sensor_mag_s> mag_sub[MAX_MAGS] {
		{ORB_ID(sensor_mag), interval_us, 0},
		{ORB_ID(sensor_mag), 0, 1},
		{ORB_ID(sensor_mag), 0, 2},
		{ORB_ID(sensor_mag), 0, 3},
	};

	uint64_t calibration_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_us;
	unsigned poll_errcount = 0;
	unsigned calibration_counter_side = 0;

	while (hrt_absolute_time() < calibration_deadline &&
	       calibration_counter_side < worker_data->calibration_points_perside) {

		if (calibrate_cancel_check(worker_data->mavlink_log_pub, calibration_started)) {
			result = calibrate_return_cancelled;
			break;
		}

		if (mag_sub[0].updatedBlocking(1000_ms)) {
			bool rejected = false;
			Vector3f new_samples[MAX_MAGS] {};

			for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
				sensor_mag_s mag;

				if (worker_data->device_ids[cur_mag] != 0) {
					if (mag_sub[cur_mag].update(&mag)) {

						if (worker_data->append_to_existing_calibration && worker_data->existing_calibration_available[cur_mag]) {
							// keep and update the existing calibration when we are not doing a full 6-axis calibration
							const Vector3f offset{worker_data->offset_existing[cur_mag]};
							const Vector3f scale{worker_data->scale_existing[cur_mag]};

							const Vector3f m{(Vector3f{mag.x, mag.y, mag.z} - offset).emult(scale)};

							mag.x = m(0);
							mag.y = m(1);
							mag.z = m(2);
						}

						// Check if this measurement is good to go in
						bool reject = reject_sample(mag.x, mag.y, mag.z,
									    worker_data->x[cur_mag], worker_data->y[cur_mag], worker_data->z[cur_mag],
									    worker_data->calibration_counter_total[cur_mag],
									    calibration_sides * worker_data->calibration_points_perside);

						if (reject) {
							rejected = true;
							PX4_DEBUG("Mag: %d rejected X: %.3f Y: %.3f Z: %.3f", cur_mag, (double)mag.x, (double)mag.y, (double)mag.z);

						} else {
							new_samples[cur_mag](0) = mag.x;
							new_samples[cur_mag](1) = mag.y;
							new_samples[cur_mag](2) = mag.z;
						}

					} else {
						rejected = true;
					}
				}
			}

			// Keep calibration of all mags in lockstep
			if (!rejected) {
				for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (worker_data->device_ids[cur_mag] != 0) {
						worker_data->x[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](0);
						worker_data->y[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](1);
						worker_data->z[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](2);
						worker_data->calibration_counter_total[cur_mag]++;
					}
				}

				calibration_counter_side++;

				unsigned new_progress = progress_percentage(worker_data) +
							(unsigned)((100 / calibration_sides) * ((float)calibration_counter_side / (float)
									worker_data->calibration_points_perside));

				if (new_progress - last_mag_progress > 3) {
					// Progress indicator for side
					calibration_log_info(worker_data->mavlink_log_pub,
							     "[cal] %s side calibration: progress <%u>",
							     detect_orientation_str(orientation), new_progress);
					px4_usleep(10000);

					last_mag_progress = new_progress;
				}
			}

		} else {
			poll_errcount++;
		}

		if (poll_errcount > worker_data->calibration_points_perside * 3) {
			result = calibrate_return_error;
			calibration_log_info(worker_data->mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			break;
		}
	}

	if (result == calibrate_return_ok) {
		calibration_log_info(worker_data->mavlink_log_pub, "[cal] %s side done, rotate to a different side",
				     detect_orientation_str(orientation));

		worker_data->done_count++;
		px4_usleep(20000);
		calibration_log_info(worker_data->mavlink_log_pub, CAL_QGC_PROGRESS_MSG, progress_percentage(worker_data));
	}

	return result;
}

calibrate_return mag_calibrate_all(orb_advert_t *mavlink_log_pub, int32_t cal_mask)
{
	calibrate_return result = calibrate_return_ok;

	mag_worker_data_t worker_data;

	// keep and update the existing calibration when we are not doing a full 6-axis calibration
	worker_data.append_to_existing_calibration = cal_mask < ((1 << 6) - 1);
	worker_data.mavlink_log_pub = mavlink_log_pub;
	worker_data.done_count = 0;
	worker_data.calibration_points_perside = calibration_total_points / detect_orientation_side_count;
	worker_data.calibration_interval_perside_us = (calibraton_duration_s / detect_orientation_side_count) * 1000 * 1000;

	calibration_sides = 0;

	for (unsigned i = 0; i < (sizeof(worker_data.side_data_collected) / sizeof(worker_data.side_data_collected[0])); i++) {

		if ((cal_mask & (1 << i)) > 0) {
			// mark as missing
			worker_data.side_data_collected[i] = false;
			calibration_sides++;

		} else {
			// mark as completed from the beginning
			worker_data.side_data_collected[i] = true;

			calibration_log_info(mavlink_log_pub,
					     "[cal] %s side done, rotate to a different side",
					     detect_orientation_str(static_cast<enum detect_orientation_return>(i)));
			px4_usleep(100000);
		}
	}

	// calibration values for each mag
	Vector3f sphere[MAX_MAGS];
	Vector3f diag[MAX_MAGS];
	Vector3f offdiag[MAX_MAGS];
	float sphere_radius[MAX_MAGS];

	for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		worker_data.existing_calibration_available[cur_mag] = false;
		worker_data.device_ids[cur_mag] = 0;
		worker_data.internal[cur_mag] = false;
		worker_data.priority[cur_mag] = MAG_DEFAULT_PRIORITY;
		worker_data.rotation[cur_mag] = -1;
		worker_data.scale_existing[cur_mag] = Vector3f{1.f, 1.f, 1.f};
		worker_data.offset_existing[cur_mag].zero();
		worker_data.power_compensation[cur_mag].zero();

		// Initialize to no memory allocated
		worker_data.x[cur_mag] = nullptr;
		worker_data.y[cur_mag] = nullptr;
		worker_data.z[cur_mag] = nullptr;
		worker_data.calibration_counter_total[cur_mag] = 0;

		sphere[cur_mag].zero();
		diag[cur_mag] = Vector3f{1.f, 1.f, 1.f};
		offdiag[cur_mag].zero();
		sphere_radius[cur_mag] = MAG_SPHERE_RADIUS;
	}

	const unsigned int calibration_points_maxcount = calibration_sides * worker_data.calibration_points_perside;

	for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

		uORB::SubscriptionData<sensor_mag_s> mag_sub{ORB_ID(sensor_mag), cur_mag};
		mag_sub.update();

		if (mag_sub.advertised() && (mag_sub.get().device_id != 0) && (mag_sub.get().timestamp > 0)) {

			worker_data.device_ids[cur_mag] = mag_sub.get().device_id;
			worker_data.internal[cur_mag] = !mag_sub.get().is_external;

			if (worker_data.internal[cur_mag]) {
				worker_data.rotation[cur_mag] = -1; // internal mags match have no configurable rotation
				worker_data.priority[cur_mag] = MAG_DEFAULT_PRIORITY;

			} else {
				worker_data.rotation[cur_mag] = ROTATION_NONE; // external default rotation none
				worker_data.priority[cur_mag] = MAG_DEFAULT_EXTERNAL_PRIORITY;
			}

			// preserve any existing power compensation or configured rotation (external only)
			for (uint8_t cal_index = 0; cal_index < MAX_MAGS; cal_index++) {
				char str[20] {};
				sprintf(str, "CAL_%s%u_ID", "MAG", cal_index);
				int32_t cal_device_id = 0;

				if (param_get(param_find(str), &cal_device_id) == PX4_OK) {
					if ((cal_device_id != 0) && (cal_device_id == worker_data.device_ids[cur_mag])) {
						// if external preserve configured rotation
						if (!worker_data.internal[cur_mag]) {
							sprintf(str, "CAL_%s%u_ROT", "MAG", cal_index);
							param_get(param_find(str), &worker_data.rotation[cur_mag]);

							// check configured rotation and reset if necessary
							if (worker_data.rotation[cur_mag] < 0 || worker_data.rotation[cur_mag] > ROTATION_MAX) {
								worker_data.rotation[cur_mag] = ROTATION_NONE;
							}
						}

						// CAL_MAGx_PRIO
						sprintf(str, "CAL_%s%u_PRIO", "MAG", cal_index);
						param_get(param_find(str), &worker_data.priority[cur_mag]);

						// check configured priority and reset if necessary
						if (worker_data.priority[cur_mag] < 0 || worker_data.priority[cur_mag] > 100) {
							worker_data.priority[cur_mag] = worker_data.internal[cur_mag] ? MAG_DEFAULT_PRIORITY : MAG_DEFAULT_EXTERNAL_PRIORITY;
						}

						for (int axis = 0; axis < 3; axis++) {
							char axis_char = 'X' + axis;

							// offsets
							sprintf(str, "CAL_%s%u_%cOFF", "MAG", cal_index, axis_char);
							param_get(param_find(str), &worker_data.offset_existing[cur_mag](axis));

							// scale
							sprintf(str, "CAL_%s%u_%cSCALE", "MAG", cal_index, axis_char);
							param_get(param_find(str), &worker_data.scale_existing[cur_mag](axis));

							// power compensation
							sprintf(str, "CAL_%s%u_%cCOMP", "MAG", cal_index, axis_char);
							param_get(param_find(str), &worker_data.power_compensation[cur_mag](axis));
						}

						worker_data.existing_calibration_available[cur_mag] = true;
					}
				}
			}

			worker_data.x[cur_mag] = static_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
			worker_data.y[cur_mag] = static_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
			worker_data.z[cur_mag] = static_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));

			if (worker_data.x[cur_mag] == nullptr || worker_data.y[cur_mag] == nullptr || worker_data.z[cur_mag] == nullptr) {
				calibration_log_critical(mavlink_log_pub, "ERROR: out of memory");
				result = calibrate_return_error;
				break;
			}

		} else {
			break;
		}
	}

	if (result == calibrate_return_ok) {
		result = calibrate_from_orientation(mavlink_log_pub,                    // uORB handle to write output
						    worker_data.side_data_collected,    // Sides to calibrate
						    mag_calibration_worker,             // Calibration worker
						    &worker_data,			// Opaque data for calibration worked
						    true);				// true: lenient still detection
	}

	// Sphere fit the data to get calibration values
	if (result == calibrate_return_ok) {
		for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (worker_data.device_ids[cur_mag] != 0) {
				// Mag in this slot is available and we should have values for it to calibrate

				// Estimate only the offsets if two-sided calibration is selected, as the problem is not constrained
				// enough to reliably estimate both scales and offsets with 2 sides only (even if the existing calibration
				// is already close)
				bool sphere_fit_only = calibration_sides <= 2;
				ellipsoid_fit_least_squares(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
							    worker_data.calibration_counter_total[cur_mag], 100,
							    &sphere[cur_mag](0), &sphere[cur_mag](1), &sphere[cur_mag](2),
							    &sphere_radius[cur_mag],
							    &diag[cur_mag](0), &diag[cur_mag](1), &diag[cur_mag](2),
							    &offdiag[cur_mag](0), &offdiag[cur_mag](1), &offdiag[cur_mag](2),
							    sphere_fit_only);

				result = check_calibration_result(sphere[cur_mag](0), sphere[cur_mag](1), sphere[cur_mag](2),
								  sphere_radius[cur_mag],
								  diag[cur_mag](0), diag[cur_mag](1), diag[cur_mag](2),
								  offdiag[cur_mag](0), offdiag[cur_mag](1), offdiag[cur_mag](2),
								  mavlink_log_pub, cur_mag, worker_data.internal);

				if (result == calibrate_return_error) {
					break;
				}
			}
		}
	}

	// Print uncalibrated data points
	if (result == calibrate_return_ok) {

		// DO NOT REMOVE! Critical validation data!
#if 0
		printf("RAW DATA:\n--------------------\n");

		for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

			if (worker_data.calibration_counter_total[cur_mag] == 0) {
				continue;
			}

			printf("RAW: MAG %u with %u samples:\n", cur_mag, worker_data.calibration_counter_total[cur_mag]);

			for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
				float x = worker_data.x[cur_mag][i];
				float y = worker_data.y[cur_mag][i];
				float z = worker_data.z[cur_mag][i];
				printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
			}

			printf(">>>>>>>\n");
		}

		printf("CALIBRATED DATA:\n--------------------\n");

		for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

			if (worker_data.calibration_counter_total[cur_mag] == 0) {
				continue;
			}

			printf("Calibrated: MAG %u with %u samples:\n", cur_mag, worker_data.calibration_counter_total[cur_mag]);

			for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
				float x = worker_data.x[cur_mag][i] - sphere[cur_mag](0);
				float y = worker_data.y[cur_mag][i] - sphere[cur_mag](1);
				float z = worker_data.z[cur_mag][i] - sphere[cur_mag](2);
				printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
			}

			printf("SPHERE RADIUS: %8.4f\n", (double)sphere_radius[cur_mag]);
			printf(">>>>>>>\n");
		}

#endif // DO NOT REMOVE! Critical validation data!
	}


	// Attempt to automatically determine external mag rotations
	if (result == calibrate_return_ok) {
		int32_t param_cal_mag_rot_auto = 0;
		param_get(param_find("CAL_MAG_ROT_AUTO"), &param_cal_mag_rot_auto);

		if ((calibration_sides >= 3) && (param_cal_mag_rot_auto == 1)) {

			// find first internal mag to use as reference
			int internal_index = -1;

			for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
				if (worker_data.internal[cur_mag] && (worker_data.device_ids[cur_mag] != 0)) {
					internal_index = cur_mag;
					break;
				}
			}

			// only proceed if there's a valid internal
			if (internal_index >= 0) {

				// apply new calibrations to all raw sensor data before comparison
				for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (worker_data.device_ids[cur_mag] != 0) {
						for (unsigned i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {

							const Vector3f sample{worker_data.x[cur_mag][i], worker_data.y[cur_mag][i], worker_data.z[cur_mag][i]};

							// apply calibration
							const Vector3f m{(sample - sphere[cur_mag]).emult(diag[cur_mag])};

							// store back in worker_data
							worker_data.x[cur_mag][i] = m(0);
							worker_data.y[cur_mag][i] = m(1);
							worker_data.z[cur_mag][i] = m(2);
						}
					}
				}

				// rotate internal mag data to board
				param_t board_rotation_h = param_find("SENS_BOARD_ROT");
				int32_t board_rotation_int = 0;
				param_get(board_rotation_h, &(board_rotation_int));
				const enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;

				if (board_rotation_int != ROTATION_NONE) {
					for (unsigned i = 0; i < worker_data.calibration_counter_total[internal_index]; i++) {
						rotate_3f(board_rotation_id,
							  worker_data.x[internal_index][i],
							  worker_data.y[internal_index][i],
							  worker_data.z[internal_index][i]
							 );
					}
				}

				for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (!worker_data.internal[cur_mag] && (worker_data.device_ids[cur_mag] != 0)) {

						const int last_sample_index = math::min(worker_data.calibration_counter_total[internal_index],
											worker_data.calibration_counter_total[cur_mag]);

						float diff_sum[ROTATION_MAX] {};

						float min_diff = FLT_MAX;
						int32_t best_rotation = ROTATION_NONE;

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
							for (int i = 0; i < last_sample_index; i++) {

								float x = worker_data.x[cur_mag][i];
								float y = worker_data.y[cur_mag][i];
								float z = worker_data.z[cur_mag][i];
								rotate_3f((enum Rotation)r, x, y, z);

								Vector3f diff = Vector3f{x, y, z} - Vector3f{worker_data.x[internal_index][i], worker_data.y[internal_index][i], worker_data.z[internal_index][i]};

								diff_sum[r] += diff.norm();
							}

							if (diff_sum[r] < min_diff) {
								min_diff = diff_sum[r];
								best_rotation = r;
							}
						}


						// Check that the best rotation is at least twice as good as the next best
						bool smallest_check_passed = true;

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
							if (r != best_rotation) {
								if (diff_sum[r] < (min_diff * 2.f)) {
									smallest_check_passed = false;
								}
							}
						}


						// Check that the average error across all samples (relative to internal mag) is less than the minimum earth field (~ 0.25 Gauss)
						const float mag_error_ga = (min_diff / last_sample_index);
						bool total_error_check_passed = (mag_error_ga < 0.25f);

						if (smallest_check_passed && total_error_check_passed) {
							if (best_rotation != worker_data.rotation[cur_mag]) {
								calibration_log_info(mavlink_log_pub, "[cal] External Mag: %d (%d), determined rotation: %d", cur_mag,
										     worker_data.device_ids[cur_mag], best_rotation);

								worker_data.rotation[cur_mag] = best_rotation;
							}
						}

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
							PX4_DEBUG("Mag: %d, rotation: %d error: %.6f", cur_mag, r, (double)diff_sum[r]);
						}
					}
				}
			}
		}
	}

	// Data points are no longer needed
	for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		free(worker_data.x[cur_mag]);
		free(worker_data.y[cur_mag]);
		free(worker_data.z[cur_mag]);
	}

	if (result == calibrate_return_ok) {
		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			Vector3f offset;
			Vector3f scale;

			if (worker_data.device_ids[cur_mag] != 0) {
				if (worker_data.append_to_existing_calibration && worker_data.existing_calibration_available[cur_mag]) {
					// Update calibration
					// The formula for applying the calibration is:
					//   mag_value = (mag_readout - (offset_existing + offset_new/scale_existing)) * scale_existing * scale_new
					offset = worker_data.offset_existing[cur_mag] + sphere[cur_mag].edivide(worker_data.scale_existing[cur_mag]);
					scale = worker_data.scale_existing[cur_mag].emult(diag[cur_mag]);

					PX4_DEBUG("[cal] %s %u updating offset: [%.4f %.4f %.4f] -> [%.4f %.4f %.4f]", "MAG", worker_data.device_ids[cur_mag],
						  (double)worker_data.offset_existing[cur_mag](0),
						  (double)worker_data.offset_existing[cur_mag](1),
						  (double)worker_data.offset_existing[cur_mag](2),
						  (double)offset(0), (double)offset(1), (double)offset(2));

					PX4_DEBUG("[cal] %s %u updating scale: [%.4f %.4f %.4f] -> [%.4f %.4f %.4f]", "MAG", worker_data.device_ids[cur_mag],
						  (double)worker_data.scale_existing[cur_mag](0),
						  (double)worker_data.scale_existing[cur_mag](1),
						  (double)worker_data.scale_existing[cur_mag](2),
						  (double)scale(0), (double)scale(1), (double)scale(2));

				} else {
					offset = sphere[cur_mag];
					scale = diag[cur_mag];

					PX4_DEBUG("[cal] %s %u offset: [%.4f %.4f %.4f] scale: [%.4f %.4f %.4f]", "MAG", worker_data.device_ids[cur_mag],
						  (double)offset(0), (double)offset(1), (double)offset(2),
						  (double)scale(0), (double)scale(1), (double)scale(2));
				}

			} else {
				// all unused parameters set to default values
				worker_data.device_ids[cur_mag] = 0;
				worker_data.rotation[cur_mag] = -1;
				worker_data.priority[cur_mag] = MAG_DEFAULT_PRIORITY;

				offset.zero();
				scale = Vector3f{1.f, 1.f, 1.f};
				worker_data.power_compensation[cur_mag].zero();
			}

			// save calibration
			char str[20] {};

			sprintf(str, "CAL_%s%u_ID", "MAG", cur_mag);
			param_set_no_notification(param_find(str), &worker_data.device_ids[cur_mag]);
			sprintf(str, "CAL_%s%u_ROT", "MAG", cur_mag);
			param_set_no_notification(param_find(str), &worker_data.rotation[cur_mag]);
			sprintf(str, "CAL_%s%u_PRIO", "MAG", cur_mag);
			param_set_no_notification(param_find(str), &worker_data.priority[cur_mag]);

			for (int axis = 0; axis < 3; axis++) {
				char axis_char = 'X' + axis;

				// offsets
				sprintf(str, "CAL_%s%u_%cOFF", "MAG", cur_mag, axis_char);
				param_set_no_notification(param_find(str), &offset(axis));

				// scale
				sprintf(str, "CAL_%s%u_%cSCALE", "MAG", cur_mag, axis_char);
				param_set_no_notification(param_find(str), &scale(axis));

				// power compensation (preserved)
				sprintf(str, "CAL_%s%u_%cCOMP", "MAG", cur_mag, axis_char);
				param_set_no_notification(param_find(str), &worker_data.power_compensation[cur_mag](axis));
			}
		}

		param_notify_changes();
	}

	return result;
}
