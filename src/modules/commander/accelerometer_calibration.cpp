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
#include "calibration_routines.h"
#include "commander_helper.h"
#include "factory_calibration_storage.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

#include "lm_fit.hpp"

#include <drivers/drv_hrt.h>
#include <include/containers/Bitset.hpp>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <matrix/math.hpp>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/parameters/param.h>
#include <lib/systemlib/err.h>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace matrix;
using namespace time_literals;

static constexpr char sensor_name[] {"accel"};
static constexpr uint8_t MAX_ACCEL_SENS = 4;

/// Data passed to calibration worker routine
struct accel_worker_data_s {
	orb_advert_t *mavlink_log_pub{nullptr};
	unsigned done_count{0};
	matrix::Vector3f accel_ref[MAX_ACCEL_SENS][detect_orientation_side_count] {};
	float accel_temperature_ref[MAX_ACCEL_SENS] {NAN, NAN, NAN, NAN};

	unsigned calibration_sides{0}; ///< The total number of sides
	bool side_data_collected[detect_orientation_side_count] {};
};

// Read specified number of accelerometer samples, calculate average and dispersion.
static calibrate_return read_accelerometer_avg(Vector3f(&accel_avg)[MAX_ACCEL_SENS][detect_orientation_side_count],
		float (&accel_temperature_avg)[MAX_ACCEL_SENS], unsigned orient)
{
	calibration::Accelerometer calibrations[MAX_ACCEL_SENS] {};
	uORB::SubscriptionMultiArray<sensor_accel_s, MAX_ACCEL_SENS> accel_subs{ORB_ID::sensor_accel};
	math::WelfordMean<Vector3f> mean[MAX_ACCEL_SENS] {};

	Vector3f accel_prev[MAX_ACCEL_SENS] {};

	float temperature_sum[MAX_ACCEL_SENS] {NAN, NAN, NAN, NAN};
	unsigned temperature_count[MAX_ACCEL_SENS] {};

	/* use the first sensor to pace the readout, but do per-sensor counts */
	bool done = false;

	const unsigned num_accels = math::min(accel_subs.advertised_count(), MAX_ACCEL_SENS);

	const hrt_abstime timestamp_start = hrt_absolute_time();

	while (!done && (hrt_elapsed_time(&timestamp_start) < 5_s)) {
		unsigned good_count = 0;

		for (unsigned accel_index = 0; accel_index < num_accels; accel_index++) {
			sensor_accel_s arp;

			while (accel_subs[accel_index].update(&arp)) {
				calibrations[accel_index].set_device_id(arp.device_id);
				calibrations[accel_index].SensorCorrectionsUpdate();

				// fetch optional thermal offset corrections in sensor/board frame
				const Vector3f accel{Vector3f{arp.x, arp.y, arp.z} - calibrations[accel_index].thermal_offset()};

				if ((accel - accel_prev[accel_index]).longerThan(0.5f)) {
					mean[accel_index].reset();
					accel_prev[accel_index] = accel;

				} else {
					mean[accel_index].update(accel);
				}

				if (!PX4_ISFINITE(temperature_sum[accel_index])) {
					// set first valid value
					temperature_sum[accel_index] = arp.temperature;
					temperature_count[accel_index] = 1;

				} else {
					temperature_sum[accel_index] += arp.temperature;
					temperature_count[accel_index]++;
				}
			}

			if ((mean[accel_index].count() > 1000) && !mean[accel_index].variance().longerThan(0.01)) {
				good_count++;
			}
		}

		if ((good_count > 0) && (good_count == num_accels)) {
			PX4_INFO("finished early: %d", mean[0].count());
			done = true;

		} else {
			px4_usleep(2000);
		}
	}

	unsigned good_count = 0;

	for (unsigned s = 0; s < num_accels; s++) {
		if (mean[s].valid()) {
			const Vector3f avg = mean[s].mean();
			const Vector3f var = mean[s].variance();

			PX4_INFO("O: %d, Accel: %d, Mean: [%.6f, %.6f, %.6f], Variance: [%.6f, %.6f, %.6f]", orient, s,
				 (double)avg(0), (double)avg(1), (double)avg(2),
				 (double)var(0), (double)var(1), (double)var(2)
				);

			if (!var.longerThan(0.01f)) {
				good_count++;

			} else {
				break;
			}

			accel_avg[s][orient] = mean[s].mean();

		} else {
			accel_avg[s][orient].zero();
		}
	}

	for (unsigned s = 0; s < MAX_ACCEL_SENS; s++) {
		if (temperature_count[s] > 0) {
			accel_temperature_avg[s] = temperature_sum[s] / temperature_count[s];

		} else {
			accel_temperature_avg[s] = NAN;
		}
	}

	return (good_count == num_accels) ? calibrate_return_ok : calibrate_return_error;
}

static calibrate_return accel_calibration_worker(detect_orientation_return orientation, void *data)
{
	accel_worker_data_s *worker_data = (accel_worker_data_s *)(data);

	int attempt = 0;

	while (attempt < 10) {
		calibration_log_info(worker_data->mavlink_log_pub, "[cal] Hold still, measuring %s side",
				     detect_orientation_str(orientation));

		if (read_accelerometer_avg(worker_data->accel_ref, worker_data->accel_temperature_ref,
					   orientation) == calibrate_return_ok) {

			calibration_log_info(worker_data->mavlink_log_pub, "[cal] %s side result: [%.3f %.3f %.3f]",
					     detect_orientation_str(orientation),
					     (double)worker_data->accel_ref[0][orientation](0),
					     (double)worker_data->accel_ref[0][orientation](1),
					     (double)worker_data->accel_ref[0][orientation](2));

			worker_data->done_count++;
			calibration_log_info(worker_data->mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 17 * worker_data->done_count);

			return calibrate_return_ok;
		}

		attempt++;
	}

	return calibrate_return_error;
}

int do_accel_calibration(orb_advert_t *mavlink_log_pub)
{
	// We should not try to subscribe if the topic doesn't actually exist and can be counted.
	const unsigned orb_accel_count = orb_group_count(ORB_ID(sensor_accel));

	// Warn that we will not calibrate more than MAX_GYROS gyroscopes
	if (orb_accel_count > MAX_ACCEL_SENS) {
		calibration_log_critical(mavlink_log_pub, "Detected %u accels, but will calibrate only %u", orb_accel_count,
					 MAX_ACCEL_SENS);

	} else if (orb_accel_count < 1) {
		calibration_log_critical(mavlink_log_pub, "No accels found");
		return PX4_ERROR;
	}

	// Collect: As defined by configuration
	// start with a full mask, all six bits set
	int32_t cal_mask = (1 << 6) - 1;
	param_get(param_find("CAL_ACC_SIDES"), &cal_mask);

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	calibration::Accelerometer calibrations[MAX_ACCEL_SENS] {};
	unsigned active_sensors = 0;

	for (uint8_t cur_accel = 0; cur_accel < MAX_ACCEL_SENS; cur_accel++) {
		uORB::SubscriptionData<sensor_accel_s> accel_sub{ORB_ID(sensor_accel), cur_accel};

		if (accel_sub.advertised() && (accel_sub.get().device_id != 0) && (accel_sub.get().timestamp > 0)) {
			calibrations[cur_accel].set_device_id(accel_sub.get().device_id);
			active_sensors++;

		} else {
			calibrations[cur_accel].Reset();
		}

		// reset calibration index to match uORB numbering
		calibrations[cur_accel].set_calibration_index(cur_accel);
	}

	if (active_sensors == 0) {
		calibration_log_critical(mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
		return PX4_ERROR;
	}

	FactoryCalibrationStorage factory_storage;

	if (factory_storage.open() != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, "ERROR: cannot open calibration storage");
		return PX4_ERROR;
	}

	/* measure and calculate offsets & scales */
	accel_worker_data_s worker_data{};
	worker_data.mavlink_log_pub = mavlink_log_pub;

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

	if (calibrate_from_orientation(mavlink_log_pub, worker_data.side_data_collected, accel_calibration_worker,
				       &worker_data) == calibrate_return_ok) {

		const Rotation board_rotation = calibration::GetBoardRotation();

		bool param_save = false;
		bool failed = true;

		int internal_rotation_count{0};
		Rotation internal_rotations[MAX_ACCEL_SENS] {};

		for (uint8_t cur_accel = 0; cur_accel < active_sensors; cur_accel++) {
			int accel_data_count = 0;

			matrix::Vector3f accel_data[6] {};

			for (int orientation = 0; orientation < 6; orientation++) {
				if ((cal_mask & (1 << orientation)) > 0) {
					accel_data[accel_data_count] = worker_data.accel_ref[cur_accel][orientation];
					accel_data_count++;
				}
			}

			bool sphere_fit_only = false;
			bool sphere_fit_success = false;
			bool ellipsoid_fit_success = false;

			sphere_params sphere_data{};
			sphere_data.radius = CONSTANTS_ONE_G;
			int ret = lm_fit(worker_data.accel_ref[cur_accel], 6, sphere_data, false);

			if (ret == PX4_OK) {
				sphere_fit_success = true;

				if (!sphere_fit_only) {
					int ellipsoid_ret = lm_fit(accel_data, accel_data_count, sphere_data, true);

					if (ellipsoid_ret == PX4_OK) {
						ellipsoid_fit_success  = true;
					}
				}

				PX4_INFO("Accel: %" PRIu8 " sphere radius: %.4f", cur_accel, (double)sphere_data.radius);
			}

			static constexpr float scale_min = 0.9f;
			static constexpr float scale_max = 1.1f;

			for (int axis = 0; axis < 3; axis++) {
				if ((sphere_data.diag(axis) < scale_min) || (sphere_data.diag(axis) > scale_max)) {
					PX4_WARN("bad scale factor"); // TODO, abort and report failure
				}
			}

			bool calibration_valid = sphere_fit_success || ellipsoid_fit_success; // TODO: scale range, offset range, etc

#if defined(DEBUD_BUILD)
			PX4_INFO("accel %d offset", cur_accel);
			sphere_data.offset.print();

			PX4_INFO("accel %d scale", cur_accel);
			sphere_data.diag.print();

			PX4_INFO("accel %d offdiagonal", cur_accel);
			sphere_data.offdiag.print();
#endif // DEBUD_BUILD

			if (!calibration_valid) {
				failed = true;
				break;
			}

			// determine best rotation
			float calibration_metric[ROTATION_MAX] {};
			float min_error = FLT_MAX;
			Rotation best_rotation = ROTATION_NONE;

			for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
				calibration_metric[r] = FLT_MAX;

				// try all rotations
				switch (r) {
				case ROTATION_ROLL_90_PITCH_68_YAW_293: // skip
				case ROTATION_PITCH_180_YAW_90:  // skip 26, same as 14 ROTATION_ROLL_180_YAW_270
				case ROTATION_PITCH_180_YAW_270: // skip 27, same as 10 ROTATION_ROLL_180_YAW_90
					break;

				default: {
						matrix::Matrix<float, 6, 3> Y;
						static constexpr float g = CONSTANTS_ONE_G;
						Y.row(0) = Vector3f{ g,  0,  0}; // ORIENTATION_TAIL_DOWN
						Y.row(1) = Vector3f{-g,  0,  0}; // ORIENTATION_NOSE_DOWN,
						Y.row(2) = Vector3f{ 0,  g,  0}; // ORIENTATION_LEFT,
						Y.row(3) = Vector3f{ 0, -g,  0}; // ORIENTATION_RIGHT,
						Y.row(4) = Vector3f{ 0,  0,  g}; // ORIENTATION_UPSIDE_DOWN,
						Y.row(5) = Vector3f{ 0,  0, -g}; // ORIENTATION_RIGHTSIDE_UP

						// apply calibration and compare data with expected direction
						for (int orientation = 0; orientation < 6; orientation++) {

							if ((cal_mask & (1 << orientation)) > 0) {
								Vector3f calibrated_accel{sphere_data.diag.emult(worker_data.accel_ref[cur_accel][orientation] - sphere_data.offset)};

								const Vector3f rotated_data{get_rot_matrix((Rotation)r) *calibrated_accel};

								calibration_metric[r] += (rotated_data - Y.row(orientation)).norm();
							}
						}
					}
				}


				if (calibration_metric[r] < min_error) {
					min_error = calibration_metric[r];
					best_rotation = (Rotation)r;
				}

			}

			PX4_INFO("best rotation: %d", best_rotation);

			if (!calibrations[cur_accel].external()) {
				internal_rotations[internal_rotation_count] = best_rotation;
				internal_rotation_count++;
			}

			bool print_all_errors = true;

			if (calibrations[cur_accel].external()) {
				switch (calibrations[cur_accel].rotation_enum()) {
				case ROTATION_ROLL_90_PITCH_68_YAW_293:
					PX4_INFO("[cal] External Accel: %" PRIu8 " (%" PRIu32 "), keeping manually configured rotation %" PRIu8,
						 cur_accel, calibrations[cur_accel].device_id(), calibrations[cur_accel].rotation_enum());
					continue;

				default: {
						if (best_rotation != calibrations[cur_accel].rotation_enum()) {
							calibration_log_info(mavlink_log_pub, "[cal] External Accel: %" PRIu8 " (%" PRIu32 "), determined rotation: %" PRIu8,
									     cur_accel, calibrations[cur_accel].device_id(), best_rotation);
							calibrations[cur_accel].set_rotation(best_rotation);

						} else {
							PX4_INFO("[cal] External Accel: %" PRIu8 " (%" PRIu32 "), no rotation change: %" PRIu8,
								 cur_accel, calibrations[cur_accel].device_id(), best_rotation);
						}
					}
					break;
				}
			}

			if (print_all_errors) {
				for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
					if (calibration_metric[r] < FLT_MAX) {
						PX4_ERR("Accel: %" PRIu8 " (%" PRIu32 "), rotation: %" PRIu32 ", error: %.3f",
							cur_accel, calibrations[cur_accel].device_id(), (uint32_t)r, (double)calibration_metric[r]);
					}
				}
			}

			// update calibration
			calibrations[cur_accel].set_offset(sphere_data.offset);
			calibrations[cur_accel].set_scale(sphere_data.diag);
			calibrations[cur_accel].set_temperature(worker_data.accel_temperature_ref[cur_accel]);

			calibrations[cur_accel].PrintStatus();

			// save all calibrations including empty slots
			if (calibrations[cur_accel].ParametersSave()) {
				param_save = true;
				failed = false;

			} else {
				failed = true;
				calibration_log_critical(mavlink_log_pub, "calibration save failed");
				break;
			}
		}

		// review SENS_BOARD_ROT, do all internal accels agree on SENS_BOARD_ROT?
		if (!failed && (internal_rotation_count > 0)) {
			Rotation best_rotation = internal_rotations[0];
			bool same = true;

			for (unsigned i = 0; i < MAX_ACCEL_SENS; i++) {
				if (internal_rotations[0] != internal_rotations[i]) {
					same = false;
					break;
				}
			}

			if (same && (best_rotation != board_rotation)) {
				PX4_ERR("Incorrect board rotation: %d", board_rotation);
				int32_t sens_board_rot = best_rotation;
				param_set_no_notification(param_find("SENS_BOARD_ROT"), &sens_board_rot);
			}
		}


		if (!failed && factory_storage.store() != PX4_OK) {
			failed = true;
		}

		if (param_save) {
			param_notify_changes();
		}

		if (!failed) {
			calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
			px4_usleep(600000); // give this message enough time to propagate
			return PX4_OK;
		}
	}

	calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	px4_usleep(600000); // give this message enough time to propagate
	return PX4_ERROR;
}

int do_accel_calibration_quick(orb_advert_t *mavlink_log_pub)
{
#if !defined(CONSTRAINED_FLASH)
	PX4_INFO("Accelerometer quick calibration");

	bool failed = true;

	FactoryCalibrationStorage factory_storage;

	if (factory_storage.open() != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, "ERROR: cannot open calibration storage");
		return PX4_ERROR;
	}

	calibration::Accelerometer calibrations[MAX_ACCEL_SENS] {};
	math::WelfordMean<Vector3f> mean[MAX_ACCEL_SENS] {};
	uORB::SubscriptionMultiArray<sensor_accel_s, MAX_ACCEL_SENS> accel_subs{ORB_ID::sensor_accel};
	px4::Bitset<MAX_ACCEL_SENS> valid_cal;

	const hrt_abstime start_time = hrt_absolute_time();

	Vector3f accel_prev[MAX_ACCEL_SENS] {};

	while ((hrt_elapsed_time(&start_time) < 3_s) && (valid_cal.count() < accel_subs.advertised_count())) {
		for (unsigned accel_index = 0; accel_index < MAX_ACCEL_SENS; accel_index++) {
			sensor_accel_s sensor_accel;

			while (accel_subs[accel_index].update(&sensor_accel)) {
				if ((sensor_accel.timestamp > 0) && (sensor_accel.device_id != 0)) {
					calibrations[accel_index].set_device_id(sensor_accel.device_id);
					calibrations[accel_index].SensorCorrectionsUpdate();

					const Vector3f accel{Vector3f{sensor_accel.x, sensor_accel.y, sensor_accel.z} - calibrations[accel_index].thermal_offset()};

					if ((accel - accel_prev[accel_index]).longerThan(0.5f)) {
						mean[accel_index].reset();
						accel_prev[accel_index] = accel;

					} else {
						mean[accel_index].update(accel);
					}

					if (mean[accel_index].count() > 300 && !mean[accel_index].variance().longerThan(0.01f)) {
						valid_cal.set(accel_index, true);
					}
				}
			}
		}

		px4_usleep(1000);

		for (unsigned accel_index = 0; accel_index < MAX_ACCEL_SENS; accel_index++) {
			if (mean[accel_index].count() > 0) {
				PX4_DEBUG("accel %d/%d valid %d var: %.6f", accel_index, accel_subs.advertised_count(), mean[accel_index].count(),
					  (double)mean[accel_index].variance().length());
			}
		}
	}

	bool calibration_updated = false;

	px4::Bitset<MAX_ACCEL_SENS> calibrated;

	static constexpr float g = CONSTANTS_ONE_G;
	matrix::Matrix<float, 6, 3> Y;
	Y.row(0) = Vector3f{ g,  0,  0}; // ORIENTATION_TAIL_DOWN
	Y.row(1) = Vector3f{-g,  0,  0}; // ORIENTATION_NOSE_DOWN,
	Y.row(2) = Vector3f{ 0,  g,  0}; // ORIENTATION_LEFT,
	Y.row(3) = Vector3f{ 0, -g,  0}; // ORIENTATION_RIGHT,
	Y.row(4) = Vector3f{ 0,  0,  g}; // ORIENTATION_UPSIDE_DOWN,
	Y.row(5) = Vector3f{ 0,  0, -g}; // ORIENTATION_RIGHTSIDE_UP

	for (unsigned accel_index = 0; accel_index < MAX_ACCEL_SENS; accel_index++) {
		if (valid_cal[accel_index]) {

			const Vector3f accel_avg = mean[accel_index].mean();

			int closest_orientation = -1;
			float smallest_orientation_error = INFINITY;

			for (int i = 0; i < 6; i++) {
				// assume sensor/board is in one of these orientations
				float accel_error = (Vector3f{Y.row(i)} - accel_avg).length();

				if (accel_error < smallest_orientation_error) {
					closest_orientation = i;
					smallest_orientation_error = accel_error;
				}
			}

			Vector3f offset{0.f, 0.f, 0.f};

			if (closest_orientation != -1) {
				PX4_INFO("assuming accel %d is orientation %s (%d)", accel_index,
					 detect_orientation_str((enum detect_orientation_return)closest_orientation),
					 closest_orientation);

				offset = accel_avg - (calibrations[accel_index].scale().emult(Vector3f{Y.row(closest_orientation)}));
				calibrated.set(accel_index, true);
			}

			const Vector3f accel_avg_calibrated = calibrations[accel_index].scale().emult(accel_avg - offset);

			if (!calibrated[accel_index] || !accel_avg_calibrated.longerThan(CONSTANTS_ONE_G * 0.9f)
			    || accel_avg_calibrated.longerThan(CONSTANTS_ONE_G * 1.1f)
			    || !PX4_ISFINITE(offset(0)) || !PX4_ISFINITE(offset(1)) || !PX4_ISFINITE(offset(2))) {

				PX4_ERR("accel %d quick calibrate failed", accel_index);
				failed = true;
				break;

			} else {
				// reset cal index to uORB
				calibrations[accel_index].set_calibration_index(accel_index);

				if (calibrations[accel_index].set_offset(offset)) {
					calibrations[accel_index].PrintStatus();
					calibration_updated = true;
				}

				failed = false;
			}
		}
	}

	if (!failed && factory_storage.store() != PX4_OK) {
		failed = true;
	}

	if (!failed) {
		if (calibration_updated) {
			bool param_save = false;

			for (unsigned accel_index = 0; accel_index < MAX_ACCEL_SENS; accel_index++) {
				if (calibrated[accel_index]) {
					if (calibrations[accel_index].ParametersSave()) {
						param_save = true;
					}
				}
			}

			if (param_save) {
				param_notify_changes();
			}
		}

		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
		return PX4_OK;
	}

#endif // !CONSTRAINED_FLASH

	calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	return PX4_ERROR;
}
