/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
#include "lm_fit.hpp"
#include "calibration_messages.h"
#include "factory_calibration_storage.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <matrix/math.hpp>
#include <lib/sensor_calibration/Magnetometer.hpp>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/conversion/rotation.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/err.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/mag_worker_data.h>

using namespace matrix;
using namespace time_literals;

static constexpr char sensor_name[] {"mag"};
static constexpr int MAX_MAGS = 4;
static constexpr float MAG_SPHERE_RADIUS_DEFAULT = 0.2f;
static constexpr unsigned int calibration_total_points = 240;	///< The total points per magnetometer
static constexpr unsigned int calibraton_duration_s = 42; 	///< The total duration the routine is allowed to take

calibrate_return mag_calibrate_all(orb_advert_t *mavlink_log_pub, int32_t cal_mask);

/// Data passed to calibration worker routine
struct mag_worker_data_t {
	orb_advert_t	*mavlink_log_pub;
	orb_advert_t	mag_worker_data_pub;
	bool		append_to_existing_calibration;
	unsigned	last_mag_progress;
	unsigned	done_count;
	unsigned	calibration_sides;					///< The total number of sides
	bool		side_data_collected[detect_orientation_side_count];
	unsigned int	calibration_points_perside;
	uint64_t	calibration_interval_perside_us;
	unsigned int	calibration_counter_total[MAX_MAGS];

	float		*x[MAX_MAGS];
	float		*y[MAX_MAGS];
	float		*z[MAX_MAGS];

	calibration::Magnetometer calibration[MAX_MAGS] {};
};

int do_mag_calibration(orb_advert_t *mavlink_log_pub)
{
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	int result = PX4_OK;

	// Collect: As defined by configuration
	// start with a full mask, all six bits set
	int32_t cal_mask = (1 << 6) - 1;
	param_get(param_find("SENS_MAG_SIDES"), &cal_mask);

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
			  unsigned max_count, float mag_sphere_radius)
{
	float min_sample_dist = fabsf(5.4f * mag_sphere_radius / sqrtf(max_count)) / 3.0f;

	for (size_t i = 0; i < count; i++) {
		float dx = sx - x[i];
		float dy = sy - y[i];
		float dz = sz - z[i];
		float dist = sqrtf(dx * dx + dy * dy + dz * dz);

		if (dist < min_sample_dist) {
			PX4_DEBUG("rejected X: %.3f Y: %.3f Z: %.3f (%.3f < %.3f) (%u/%u) ", (double)sx, (double)sy, (double)sz, (double)dist,
				  (double)min_sample_dist, count, max_count);

			return true;
		}
	}

	return false;
}

static unsigned progress_percentage(mag_worker_data_t *worker_data)
{
	return 100 * ((float)worker_data->done_count) / worker_data->calibration_sides;
}

// Returns calibrate_return_error if any parameter is not finite
// Logs if parameters are out of range
static calibrate_return check_calibration_result(float offset_x, float offset_y, float offset_z,
		float sphere_radius,
		float diag_x, float diag_y, float diag_z,
		float offdiag_x, float offdiag_y, float offdiag_z,
		orb_advert_t *mavlink_log_pub, uint8_t cur_mag)
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
			calibration_log_emergency(mavlink_log_pub, "Retry calibration (sphere NaN, %" PRIu8 ")", cur_mag);
			return calibrate_return_error;
		}
	}

	// earth field between 0.25 and 0.65 Gauss
	if (sphere_radius < 0.2f || sphere_radius >= 0.7f) {
		calibration_log_emergency(mavlink_log_pub, "Retry calibration (mag %" PRIu8 " sphere radius invalid %.3f)", cur_mag,
					  (double)sphere_radius);
		return calibrate_return_error;
	}

	// Notify if a parameter which should be positive is non-positive
	const int num_positive = sizeof(should_be_positive) / sizeof(*should_be_positive);

	for (unsigned i = 0; i < num_positive; ++i) {
		if (should_be_positive[i] <= 0.0f) {
			calibration_log_emergency(mavlink_log_pub, "Retry calibration (mag %" PRIu8 " with non-positive scale)", cur_mag);
			return calibrate_return_error;
		}
	}

	// Notify if offsets are too large
	const int num_not_huge = sizeof(should_be_not_huge) / sizeof(*should_be_not_huge);

	for (unsigned i = 0; i < num_not_huge; ++i) {
		// maximum measurement range is ~1.9 Ga, the earth field is ~0.6 Ga,
		//  so an offset larger than ~1.3 Ga means the mag will saturate in some directions.
		static constexpr float MAG_MAX_OFFSET_LEN = 1.3f;

		if (fabsf(should_be_not_huge[i]) > MAG_MAX_OFFSET_LEN) {
			calibration_log_critical(mavlink_log_pub, "Warning: mag %" PRIu8 " with large offsets", cur_mag);
			break;
		}
	}

	return calibrate_return_ok;
}

static float get_sphere_radius()
{
	// if GPS is available use real field intensity from world magnetic model
	uORB::SubscriptionMultiArray<sensor_gps_s, 3> gps_subs{ORB_ID::vehicle_gps_position};

	for (auto &gps_sub : gps_subs) {
		sensor_gps_s gps;

		if (gps_sub.copy(&gps)) {
			if (hrt_elapsed_time(&gps.timestamp) < 100_s && (gps.fix_type >= 2) && (gps.eph < 1000)) {
				// magnetic field data returned by the geo library using the current GPS position
				return get_mag_strength_gauss(gps.latitude_deg, gps.longitude_deg);
			}
		}
	}

	return MAG_SPHERE_RADIUS_DEFAULT;
}

static calibrate_return mag_calibration_worker(detect_orientation_return orientation, void *data)
{
	const hrt_abstime calibration_started = hrt_absolute_time();
	calibrate_return result = calibrate_return_ok;

	mag_worker_data_t *worker_data = (mag_worker_data_t *)(data);

	float mag_sphere_radius = get_sphere_radius();

	// notify user to start rotating
	set_tune(tune_control_s::TUNE_ID_SINGLE_BEEP);

	calibration_log_info(worker_data->mavlink_log_pub, "[cal] Rotate vehicle");

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

	uORB::SubscriptionBlocking<sensor_mag_s> mag_sub[MAX_MAGS] {
		{ORB_ID(sensor_mag), 0, 0},
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
				if (worker_data->calibration[cur_mag].device_id() != 0) {
					bool updated = false;
					sensor_mag_s mag;

					while (mag_sub[cur_mag].update(&mag)) {
						if (worker_data->append_to_existing_calibration) {
							// keep and update the existing calibration when we are not doing a full 6-axis calibration
							const Matrix3f &scale = worker_data->calibration[cur_mag].scale();
							const Vector3f &offset = worker_data->calibration[cur_mag].offset();

							const Vector3f m{scale *(Vector3f{mag.x, mag.y, mag.z} - offset)};

							mag.x = m(0);
							mag.y = m(1);
							mag.z = m(2);
						}

						// Check if this measurement is good to go in
						bool reject = reject_sample(mag.x, mag.y, mag.z,
									    worker_data->x[cur_mag], worker_data->y[cur_mag], worker_data->z[cur_mag],
									    worker_data->calibration_counter_total[cur_mag],
									    worker_data->calibration_sides * worker_data->calibration_points_perside,
									    mag_sphere_radius);

						if (!reject) {
							new_samples[cur_mag] = Vector3f{mag.x, mag.y, mag.z};
							updated = true;
							break;
						}
					}

					// require an update for all valid mags
					if (!updated) {
						rejected = true;
					}
				}
			}

			// Keep calibration of all mags in lockstep
			if (!rejected) {
				for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (worker_data->calibration[cur_mag].device_id() != 0) {
						worker_data->x[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](0);
						worker_data->y[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](1);
						worker_data->z[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](2);

						worker_data->calibration_counter_total[cur_mag]++;
					}
				}

				hrt_abstime now = hrt_absolute_time();
				mag_worker_data_s status;
				status.timestamp = now;
				status.timestamp_sample = now;
				status.done_count = worker_data->done_count;
				status.calibration_points_perside = worker_data->calibration_points_perside;
				status.calibration_interval_perside_us = worker_data->calibration_interval_perside_us;

				for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					status.calibration_counter_total[cur_mag] = worker_data->calibration_counter_total[cur_mag];
					status.side_data_collected[cur_mag] = worker_data->side_data_collected[cur_mag];

					if (worker_data->calibration[cur_mag].device_id() != 0) {
						const unsigned int sample = worker_data->calibration_counter_total[cur_mag] - 1;
						status.x[cur_mag] = worker_data->x[cur_mag][sample];
						status.y[cur_mag] = worker_data->y[cur_mag][sample];
						status.z[cur_mag] = worker_data->z[cur_mag][sample];

					} else {
						status.x[cur_mag] = 0.f;
						status.y[cur_mag] = 0.f;
						status.z[cur_mag] = 0.f;
					}
				}

				if (worker_data->mag_worker_data_pub == nullptr) {
					worker_data->mag_worker_data_pub = orb_advertise(ORB_ID(mag_worker_data), &status);

				} else {
					orb_publish(ORB_ID(mag_worker_data), worker_data->mag_worker_data_pub, &status);
				}

				calibration_counter_side++;

				unsigned new_progress = progress_percentage(worker_data) +
							(unsigned)((100 / worker_data->calibration_sides) * ((float)calibration_counter_side / (float)
									worker_data->calibration_points_perside));

				if (new_progress - worker_data->last_mag_progress > 0) {
					// Progress indicator for side
					calibration_log_info(worker_data->mavlink_log_pub,
							     "[cal] %s side calibration: progress <%u>",
							     detect_orientation_str(orientation), new_progress);
					px4_usleep(10000);

					worker_data->last_mag_progress = new_progress;
				}
			}

			PX4_DEBUG("side counter %u / %u", calibration_counter_side, worker_data->calibration_points_perside);

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
	// We should not try to subscribe if the topic doesn't actually exist and can be counted.
	const unsigned orb_mag_count = orb_group_count(ORB_ID(sensor_mag));

	// Warn that we will not calibrate more than MAX_GYROS gyroscopes
	if (orb_mag_count > MAX_MAGS) {
		calibration_log_critical(mavlink_log_pub, "Detected %u mags, but will calibrate only %u", orb_mag_count, MAX_MAGS);

	} else if (orb_mag_count < 1) {
		calibration_log_critical(mavlink_log_pub, "No mags found");
		return calibrate_return_error;
	}

	calibrate_return result = calibrate_return_ok;

	mag_worker_data_t worker_data{};

	worker_data.mag_worker_data_pub = nullptr;

	// keep and update the existing calibration when we are not doing a full 6-axis calibration
	worker_data.append_to_existing_calibration = cal_mask < ((1 << 6) - 1);
	worker_data.mavlink_log_pub = mavlink_log_pub;
	worker_data.last_mag_progress = 0;
	worker_data.calibration_sides = 0;
	worker_data.done_count = 0;
	worker_data.calibration_points_perside = calibration_total_points / detect_orientation_side_count;
	worker_data.calibration_interval_perside_us = (calibraton_duration_s / detect_orientation_side_count) * 1000 * 1000;

	for (unsigned i = 0; i < (sizeof(worker_data.side_data_collected) / sizeof(worker_data.side_data_collected[0])); i++) {

		if ((cal_mask & (1 << i)) > 0) {
			// mark as missing
			worker_data.side_data_collected[i] = false;
			worker_data.calibration_sides++;

		} else {
			// mark as completed from the beginning
			worker_data.side_data_collected[i] = true;

			calibration_log_info(mavlink_log_pub,
					     "[cal] %s side done, rotate to a different side",
					     detect_orientation_str(static_cast<enum detect_orientation_return>(i)));
			px4_usleep(100000);
		}
	}

	for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		// Initialize to no memory allocated
		worker_data.x[cur_mag] = nullptr;
		worker_data.y[cur_mag] = nullptr;
		worker_data.z[cur_mag] = nullptr;
		worker_data.calibration_counter_total[cur_mag] = 0;
	}

	const unsigned int calibration_points_maxcount = worker_data.calibration_sides * worker_data.calibration_points_perside;

	for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

		uORB::SubscriptionData<sensor_mag_s> mag_sub{ORB_ID(sensor_mag), cur_mag};

		if (mag_sub.advertised() && (mag_sub.get().device_id != 0) && (mag_sub.get().timestamp > 0)) {
			worker_data.calibration[cur_mag].set_device_id(mag_sub.get().device_id);
		}

		// reset calibration index to match uORB numbering
		worker_data.calibration[cur_mag].set_calibration_index(cur_mag);

		if (worker_data.calibration[cur_mag].device_id() != 0) {
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

	// calibration values for each mag
	Vector3f sphere[MAX_MAGS];
	Vector3f diag[MAX_MAGS];
	Vector3f offdiag[MAX_MAGS];
	float sphere_radius[MAX_MAGS];

	const float mag_sphere_radius = get_sphere_radius();

	for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		sphere_radius[cur_mag] = mag_sphere_radius;
		sphere[cur_mag].zero();
		diag[cur_mag] = Vector3f{1.f, 1.f, 1.f};
		offdiag[cur_mag].zero();
	}

	if (result == calibrate_return_ok) {
		// Sphere fit the data to get calibration values
		for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (worker_data.calibration[cur_mag].device_id() != 0) {
				// Mag in this slot is available and we should have values for it to calibrate

				// Estimate only the offsets if two-sided calibration is selected, as the problem is not constrained
				// enough to reliably estimate both scales and offsets with 2 sides only (even if the existing calibration
				// is already close)
				bool sphere_fit_only = worker_data.calibration_sides <= 2;

				sphere_params sphere_data;
				sphere_data.radius = sphere_radius[cur_mag];
				sphere_data.offset = matrix::Vector3f(sphere[cur_mag](0), sphere[cur_mag](1), sphere[cur_mag](2));
				sphere_data.diag = matrix::Vector3f(diag[cur_mag](0), diag[cur_mag](1), diag[cur_mag](2));
				sphere_data.offdiag = matrix::Vector3f(offdiag[cur_mag](0), offdiag[cur_mag](1), offdiag[cur_mag](2));

				bool sphere_fit_success = false;
				bool ellipsoid_fit_success = false;
				int ret = lm_mag_fit(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
						     worker_data.calibration_counter_total[cur_mag], sphere_data, false);

				if (ret == PX4_OK) {
					sphere_fit_success = true;
					PX4_INFO("Mag: %" PRIu8 " sphere radius: %.4f", cur_mag, (double)sphere_data.radius);

					if (!sphere_fit_only) {
						int ellipsoid_ret = lm_mag_fit(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
									       worker_data.calibration_counter_total[cur_mag], sphere_data, true);

						if (ellipsoid_ret == PX4_OK) {
							ellipsoid_fit_success = true;
						}
					}
				}

				if (!sphere_fit_success && !ellipsoid_fit_success) {
					if (worker_data.calibration[cur_mag].enabled()) {
						calibration_log_emergency(mavlink_log_pub, "Retry calibration (unable to fit mag %" PRIu8 ")", cur_mag);
						result = calibrate_return_error;
						break;

					} else {
						calibration_log_info(mavlink_log_pub, "Retry calibration (unable to fit mag %" PRIu8 ")", cur_mag);
						continue;
					}
				}

				sphere_radius[cur_mag] = sphere_data.radius;

				for (int i = 0; i < 3; i++) {
					sphere[cur_mag](i) = sphere_data.offset(i);
					diag[cur_mag](i) = sphere_data.diag(i);
					offdiag[cur_mag](i) = sphere_data.offdiag(i);
				}

				result = check_calibration_result(sphere[cur_mag](0), sphere[cur_mag](1), sphere[cur_mag](2),
								  sphere_radius[cur_mag],
								  diag[cur_mag](0), diag[cur_mag](1), diag[cur_mag](2),
								  offdiag[cur_mag](0), offdiag[cur_mag](1), offdiag[cur_mag](2),
								  mavlink_log_pub, cur_mag);

				if (result == calibrate_return_error) {
					// tolerate mag cal failures if this sensor is disabled
					if (worker_data.calibration[cur_mag].enabled()) {
						break;

					} else {
						// reset
						sphere[cur_mag].zero();
						diag[cur_mag] = Vector3f{1.f, 1.f, 1.f};
						offdiag[cur_mag].zero();
					}
				}
			}
		}
	}


#if 0

	// DO NOT REMOVE! Critical validation data!
	if (result == calibrate_return_ok) {
		// Print uncalibrated data points
		for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (worker_data.calibration_counter_total[cur_mag] == 0) {
				continue;
			}

			printf("MAG %" PRIu8 " with %u samples:\n", cur_mag, worker_data.calibration_counter_total[cur_mag]);
			printf("RAW -> CALIBRATED\n");

			float scale_data[9] {
				diag[cur_mag](0),    offdiag[cur_mag](0), offdiag[cur_mag](1),
				offdiag[cur_mag](0),    diag[cur_mag](1), offdiag[cur_mag](2),
				offdiag[cur_mag](1), offdiag[cur_mag](2),    diag[cur_mag](2)
			};

			const Matrix3f scale{scale_data};
			const Vector3f &offset = sphere[cur_mag];

			for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
				float x = worker_data.x[cur_mag][i];
				float y = worker_data.y[cur_mag][i];
				float z = worker_data.z[cur_mag][i];

				// apply calibration
				const Vector3f cal{scale *(Vector3f{x, y, z} - offset)};

				printf("[%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]\n",
				       (double)x, (double)y, (double)z,
				       (double)cal(0), (double)cal(1), (double)cal(2));
			}

			printf("SPHERE RADIUS: %8.4f\n", (double)sphere_radius[cur_mag]);
		}
	}

#endif // DO NOT REMOVE! Critical validation data!


	// Attempt to automatically determine external mag rotations
	if (result == calibrate_return_ok) {
		int32_t param_sens_mag_autorot = 0;
		param_get(param_find("SENS_MAG_AUTOROT"), &param_sens_mag_autorot);

		if ((worker_data.calibration_sides >= 3) && (param_sens_mag_autorot == 1)) {

			// find first internal mag to use as reference
			int internal_index = -1;

			for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
				if (!worker_data.calibration[cur_mag].external() && (worker_data.calibration[cur_mag].device_id() != 0)) {
					internal_index = cur_mag;
					break;
				}
			}

			// only proceed if there's a valid internal
			if (internal_index >= 0) {

				const Dcmf board_rotation = calibration::GetBoardRotationMatrix();

				// apply new calibrations to all raw sensor data before comparison
				for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (worker_data.calibration[cur_mag].device_id() != 0) {

						float scale_data[9] {
							diag[cur_mag](0),    offdiag[cur_mag](0), offdiag[cur_mag](1),
							offdiag[cur_mag](0),    diag[cur_mag](1), offdiag[cur_mag](2),
							offdiag[cur_mag](1), offdiag[cur_mag](2),    diag[cur_mag](2)
						};
						const Matrix3f scale{scale_data};

						const Vector3f &offset{sphere[cur_mag]};

						for (unsigned i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {

							const Vector3f sample{worker_data.x[cur_mag][i], worker_data.y[cur_mag][i], worker_data.z[cur_mag][i]};

							// apply calibration
							Vector3f m{scale *(sample - offset)};

							if (!worker_data.calibration[cur_mag].external()) {
								// rotate internal mag data to board
								m = board_rotation * m;
							}

							// store back in worker_data
							worker_data.x[cur_mag][i] = m(0);
							worker_data.y[cur_mag][i] = m(1);
							worker_data.z[cur_mag][i] = m(2);
						}
					}
				}

				// external mags try all rotations and compute mean square error (MSE) compared with first internal mag
				for (int cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if ((worker_data.calibration[cur_mag].device_id() != 0) && (cur_mag != internal_index)) {

						const int last_sample_index = math::min(worker_data.calibration_counter_total[internal_index],
											worker_data.calibration_counter_total[cur_mag]);

						float MSE[ROTATION_MAX] {}; // mean square error for each rotation

						float min_mse = FLT_MAX;
						Rotation best_rotation = ROTATION_NONE;

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {

							switch (r) {
							case ROTATION_ROLL_90_PITCH_68_YAW_293: // skip

							// FALLTHROUGH
							case ROTATION_PITCH_180_YAW_90:  // skip 26, same as 14 ROTATION_ROLL_180_YAW_270

							// FALLTHROUGH
							case ROTATION_PITCH_180_YAW_270: // skip 27, same as 10 ROTATION_ROLL_180_YAW_90

							// FALLTHROUGH
							case ROTATION_CUSTOM: // Skip, as we currently don't support detecting arbitrary euler angle orientation

								MSE[r] = FLT_MAX;
								break;

							default:
								float diff_sum = 0.f;

								for (int i = 0; i < last_sample_index; i++) {
									float x = worker_data.x[cur_mag][i];
									float y = worker_data.y[cur_mag][i];
									float z = worker_data.z[cur_mag][i];
									rotate_3f((enum Rotation)r, x, y, z);

									Vector3f diff = Vector3f{x, y, z} - Vector3f{worker_data.x[internal_index][i], worker_data.y[internal_index][i], worker_data.z[internal_index][i]};

									diff_sum += diff.norm_squared();
								}

								// compute mean squared error
								MSE[r] = diff_sum / last_sample_index;

								if (MSE[r] < min_mse) {
									min_mse = MSE[r];
									best_rotation = (Rotation)r;
								}

								break;
							}
						}


						// Check that the best rotation is at least twice as good as the next best
						bool smallest_check_passed = true;

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
							if (r != best_rotation) {
								if (MSE[r] < (min_mse * 2.f)) {
									smallest_check_passed = false;
								}
							}
						}


						// Check that the average error across all samples (relative to internal mag) is less than the minimum earth field (~0.25 Gauss)
						const float mag_error_gs = sqrt(min_mse / last_sample_index);
						bool total_error_check_passed = (mag_error_gs < 0.25f);

#if defined(DEBUG_BUILD)
						bool print_all_mse = true;
#else
						bool print_all_mse = false;
#endif // DEBUG_BUILD

						if (worker_data.calibration[cur_mag].external()) {

							switch (worker_data.calibration[cur_mag].rotation_enum()) {
							case ROTATION_ROLL_90_PITCH_68_YAW_293:
								PX4_INFO("[cal] External Mag: %d (%" PRIu32 "), keeping manually configured rotation %" PRIu8, cur_mag,
									 worker_data.calibration[cur_mag].device_id(), worker_data.calibration[cur_mag].rotation_enum());
								continue;

							case ROTATION_CUSTOM:
								PX4_INFO("[cal] External Mag: %d (%" PRIu32 "), not setting rotation enum since it's specified by Euler Angle",
									 cur_mag, worker_data.calibration[cur_mag].device_id());
								continue; // Continue to the next mag loop

							default:
								break;
							}

							if (smallest_check_passed && total_error_check_passed) {
								if (best_rotation != worker_data.calibration[cur_mag].rotation_enum()) {
									calibration_log_info(mavlink_log_pub, "[cal] External Mag: %d (%" PRIu32 ")), determined rotation: %" PRIu8, cur_mag,
											     worker_data.calibration[cur_mag].device_id(), best_rotation);

									worker_data.calibration[cur_mag].set_rotation(best_rotation);

								} else {
									PX4_INFO("[cal] External Mag: %d (%" PRIu32 ")), no rotation change: %d", cur_mag,
										 worker_data.calibration[cur_mag].device_id(), best_rotation);
								}

							} else {
								PX4_ERR("External Mag: %d (%" PRIu32 ")), determining rotation failed", cur_mag,
									worker_data.calibration[cur_mag].device_id());
								print_all_mse = true;
							}

						} else {
							// non-primary internal mags, warn if there seems to be a rotation relative to the first primary (internal_index)
							if (best_rotation != ROTATION_NONE) {
								calibration_log_critical(mavlink_log_pub,
											 "[cal] Internal Mag: %d (%" PRIu32 ") rotation %d relative to primary %d (%" PRIu32 ")",
											 cur_mag, worker_data.calibration[cur_mag].device_id(), best_rotation,
											 internal_index, worker_data.calibration[internal_index].device_id());

								print_all_mse = true;
							}
						}

						if (print_all_mse) {
							for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
								PX4_ERR("%s Mag: %d (%" PRIu32 "), rotation: %d, MSE: %.3f",
									worker_data.calibration[cur_mag].external() ? "External" : "Internal",
									cur_mag, worker_data.calibration[cur_mag].device_id(), r, (double)MSE[r]);
							}
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

	FactoryCalibrationStorage factory_storage;

	if (result == calibrate_return_ok && factory_storage.open() != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, "ERROR: cannot open calibration storage");
		result = calibrate_return_error;
	}

	if (result == calibrate_return_ok) {
		bool param_save = false;
		bool failed = true;

		bool external_mag_available = false;

		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (worker_data.calibration[cur_mag].external() && worker_data.calibration[cur_mag].enabled()) {
				external_mag_available = true;
				break;
			}
		}

		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

			calibration::Magnetometer &current_cal = worker_data.calibration[cur_mag];

			if (current_cal.device_id() != 0) {
				if (worker_data.append_to_existing_calibration) {
					// Update calibration
					// The formula for applying the calibration is:
					//   mag_value = (mag_readout - (offset_existing + offset_new/scale_existing)) * scale_existing
					Vector3f offset = current_cal.offset() + sphere[cur_mag].edivide(current_cal.scale().diag());
					current_cal.set_offset(offset);

				} else {
					current_cal.set_offset(sphere[cur_mag]);
					current_cal.set_scale(diag[cur_mag]);
					current_cal.set_offdiagonal(offdiag[cur_mag]);
				}

				if (external_mag_available && !current_cal.external()) {
					// automatically disable the internal mags as they should not be used for navigation
					current_cal.disable();
				}

				current_cal.PrintStatus();

				if (current_cal.ParametersSave(cur_mag, true)) {
					param_save = true;
					failed = false;

				} else {
					failed = true;
					calibration_log_critical(mavlink_log_pub, "calibration save failed");
					break;
				}
			}
		}

		if (!failed && factory_storage.store() != PX4_OK) {
			failed = true;
		}

		if (param_save) {
			param_notify_changes();
		}

		if (failed) {
			result = calibrate_return_error;
		}
	}

	return result;
}

int do_mag_calibration_quick(orb_advert_t *mavlink_log_pub, float heading_radians,
			     float latitude_deg, float longitude_deg)
{
	// magnetometer quick calibration
	//  if GPS available use world magnetic model to zero mag offsets
	bool mag_earth_available = false;

	if (PX4_ISFINITE(latitude_deg) && PX4_ISFINITE(longitude_deg)) {
		mag_earth_available = true;

	} else {
		uORB::Subscription vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
		sensor_gps_s gps;

		if (vehicle_gps_position_sub.copy(&gps)) {
			if ((gps.timestamp != 0) && (gps.eph < 1000)) {
				latitude_deg = (float)gps.latitude_deg;
				longitude_deg = (float)gps.longitude_deg;
				mag_earth_available = true;
			}
		}
	}

	if (!mag_earth_available) {
		calibration_log_critical(mavlink_log_pub, "GPS required for mag quick cal");
		return PX4_ERROR;

	} else {
		// magnetic field data returned by the geo library using the current GPS position
		const float declination_rad = math::radians(get_mag_declination_degrees(latitude_deg, longitude_deg));
		const float inclination_rad = math::radians(get_mag_inclination_degrees(latitude_deg, longitude_deg));
		const float field_strength_gauss = get_mag_strength_gauss(latitude_deg, longitude_deg);

		const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -inclination_rad, declination_rad))
						* Vector3f(field_strength_gauss, 0, 0);

		uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
		vehicle_attitude_s attitude{};
		vehicle_attitude_sub.copy(&attitude);

		if (hrt_elapsed_time(&attitude.timestamp) > 1_s) {
			calibration_log_critical(mavlink_log_pub, "attitude required for mag quick cal");
			return PX4_ERROR;
		}

		FactoryCalibrationStorage factory_storage;

		if (factory_storage.open() != PX4_OK) {
			calibration_log_critical(mavlink_log_pub, "ERROR: cannot open calibration storage");
			return PX4_ERROR;
		}

		calibration_log_info(mavlink_log_pub, "Assuming vehicle is facing heading %.1f degrees",
				     (double)math::degrees(heading_radians));

		matrix::Eulerf euler{matrix::Quatf{attitude.q}};
		euler(2) = heading_radians;

		const Vector3f expected_field = Dcmf(euler).transpose() * mag_earth_pred;

		bool param_save = false;
		bool failed = true;

		for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			uORB::Subscription mag_sub{ORB_ID(sensor_mag), cur_mag};
			sensor_mag_s mag{};
			mag_sub.copy(&mag);

			if (mag_sub.advertised() && (mag.timestamp != 0) && (mag.device_id != 0)) {

				calibration::Magnetometer cal{mag.device_id};

				// use any existing scale and store the offset to the expected earth field
				const Vector3f offset = Vector3f{mag.x, mag.y, mag.z} - (cal.scale().I() * cal.rotation().transpose() * expected_field);
				cal.set_offset(offset);

				// save new calibration
				if (cal.ParametersSave(cur_mag)) {
					cal.PrintStatus();
					param_save = true;
					failed = false;

				} else {
					failed = true;
					calibration_log_critical(mavlink_log_pub, "calibration save failed");
					break;
				}
			}
		}

		if (param_save) {
			param_notify_changes();
		}

		if (!failed && factory_storage.store() != PX4_OK) {
			failed = true;
		}

		if (!failed) {
			calibration_log_info(mavlink_log_pub, "Mag quick calibration finished");
			return PX4_OK;
		}
	}

	calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	return PX4_ERROR;
}
