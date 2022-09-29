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
 * @file gyro_calibration.cpp
 *
 * Gyroscope calibration routine
 */

#include <px4_platform_common/px4_config.h>
#include "factory_calibration_storage.h"
#include "gyro_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/MedianFilter.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_imu_fifo.h>
#include <uORB/SubscriptionMultiArray.hpp>

static constexpr char sensor_name[] {"gyro"};
static constexpr unsigned MAX_GYROS = 4;

using matrix::Vector3f;

/// Data passed to calibration worker routine
struct gyro_worker_data_t {
	orb_advert_t *mavlink_log_pub{nullptr};

	calibration::Gyroscope calibrations[MAX_GYROS] {};

	Vector3f offset[MAX_GYROS] {};

	math::MedianFilter<float, 9> filter[3] {};
};

static calibrate_return gyro_calibration_worker(gyro_worker_data_t &worker_data)
{
	const hrt_abstime calibration_started = hrt_absolute_time();
	unsigned calibration_counter[MAX_GYROS] {};
	static constexpr unsigned CALIBRATION_COUNT = 250;
	unsigned poll_errcount = 0;

	uORB::SubscriptionMultiArray<sensor_gyro_s, MAX_GYROS> sensor_gyro_subs{ORB_ID::sensor_gyro};
	uORB::SubscriptionMultiArray<sensor_imu_fifo_s, MAX_GYROS> sensor_imu_fifo_subs{ORB_ID::sensor_imu_fifo};

	/* use slowest gyro to pace, but count correctly per-gyro for statistics */
	unsigned slow_count = 0;

	while (slow_count < CALIBRATION_COUNT) {
		if (calibrate_cancel_check(worker_data.mavlink_log_pub, calibration_started)) {
			return calibrate_return_cancelled;
		}

		if (sensor_gyro_subs.updated() || sensor_imu_fifo_subs.updated()) {
			unsigned update_count = CALIBRATION_COUNT;

			int total_gyros = 0;

			for (unsigned imu_fifo_index = 0; imu_fifo_index < sensor_imu_fifo_subs.size(); imu_fifo_index++) {
				if ((worker_data.calibrations[total_gyros].device_id() != 0)
				    && (calibration_counter[total_gyros] < CALIBRATION_COUNT)) {

					sensor_imu_fifo_s sensor_imu_fifo;

					while (sensor_imu_fifo_subs[imu_fifo_index].update(&sensor_imu_fifo)) {
						// fetch optional thermal offset corrections in sensor frame
						const Vector3f &thermal_offset{worker_data.calibrations[total_gyros].thermal_offset()};

						const Vector3f gyro_sample = calibration::AverageFifoGyro(sensor_imu_fifo);

						worker_data.offset[total_gyros] += gyro_sample - thermal_offset;

						calibration_counter[total_gyros]++;

						if (total_gyros == 0) {
							worker_data.filter[0].insert(gyro_sample(0) - thermal_offset(0));
							worker_data.filter[1].insert(gyro_sample(1) - thermal_offset(1));
							worker_data.filter[2].insert(gyro_sample(2) - thermal_offset(2));
						}
					}

					// Maintain the sample count of the slowest sensor
					if (calibration_counter[total_gyros] && calibration_counter[total_gyros] < update_count) {
						update_count = calibration_counter[total_gyros];
					}
				}

				if (sensor_imu_fifo_subs[imu_fifo_index].advertised()) {
					total_gyros++;
				}
			}

			for (unsigned gyro_index = 0; gyro_index < sensor_gyro_subs.size(); gyro_index++) {
				if ((worker_data.calibrations[total_gyros].device_id() != 0)
				    && (calibration_counter[total_gyros] < CALIBRATION_COUNT)) {

					sensor_gyro_s sensor_gyro;

					while (sensor_gyro_subs[gyro_index].update(&sensor_gyro)) {
						// fetch optional thermal offset corrections in sensor frame
						const Vector3f &thermal_offset{worker_data.calibrations[total_gyros].thermal_offset()};

						worker_data.offset[total_gyros] += Vector3f{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z} - thermal_offset;

						calibration_counter[total_gyros]++;

						if (total_gyros == 0) {
							worker_data.filter[0].insert(sensor_gyro.x - thermal_offset(0));
							worker_data.filter[1].insert(sensor_gyro.y - thermal_offset(1));
							worker_data.filter[2].insert(sensor_gyro.z - thermal_offset(2));
						}
					}

					// Maintain the sample count of the slowest sensor
					if (calibration_counter[total_gyros] && calibration_counter[total_gyros] < update_count) {
						update_count = calibration_counter[total_gyros];
					}
				}

				if (sensor_gyro_subs[gyro_index].advertised()) {
					total_gyros++;
				}
			}

			const unsigned progress = (update_count * 100) / CALIBRATION_COUNT;

			if (progress % 10 == 0) {
				calibration_log_info(worker_data.mavlink_log_pub, CAL_QGC_PROGRESS_MSG, progress);
			}

			// Propagate out the slowest sensor's count
			if (slow_count < update_count) {
				slow_count = update_count;
			}

			px4_usleep(10'000);

		} else {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {
			calibration_log_critical(worker_data.mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			return calibrate_return_error;
		}
	}

	for (unsigned i = 0; i < MAX_GYROS; i++) {
		if ((worker_data.calibrations[i].device_id() != 0) && (calibration_counter[i] < CALIBRATION_COUNT / 2)) {
			calibration_log_critical(worker_data.mavlink_log_pub, "ERROR: missing data, sensor %d", i)
			return calibrate_return_error;
		}

		worker_data.offset[i] /= calibration_counter[i];
	}

	return calibrate_return_ok;
}

int do_gyro_calibration(orb_advert_t *mavlink_log_pub)
{
	int res = PX4_OK;

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	gyro_worker_data_t worker_data{};
	worker_data.mavlink_log_pub = mavlink_log_pub;

	// We should not try to subscribe if the topic doesn't actually exist and can be counted.
	const unsigned orb_gyro_count = orb_group_count(ORB_ID(sensor_imu_fifo)) + orb_group_count(ORB_ID(sensor_gyro));

	// Warn that we will not calibrate more than MAX_GYROS gyroscopes
	if (orb_gyro_count > MAX_GYROS) {
		calibration_log_critical(mavlink_log_pub, "Detected %u gyros, but will calibrate only %u", orb_gyro_count, MAX_GYROS);

	} else if (orb_gyro_count < 1) {
		calibration_log_critical(mavlink_log_pub, "No gyros found");
		return PX4_ERROR;
	}

	int total_gyros = 0;

	uORB::SubscriptionMultiArray<sensor_imu_fifo_s, MAX_GYROS> sensor_imu_fifo_subs{ORB_ID::sensor_imu_fifo};

	for (uint8_t i = 0; i < sensor_imu_fifo_subs.size(); i++) {
		sensor_imu_fifo_s sensor_imu_fifo;

		if (sensor_imu_fifo_subs[i].update(&sensor_imu_fifo) && sensor_imu_fifo.device_id != 0) {
			worker_data.calibrations[total_gyros].set_device_id(sensor_imu_fifo.device_id);
			total_gyros++;
		}
	}

	uORB::SubscriptionMultiArray<sensor_gyro_s, MAX_GYROS> sensor_gyro_subs{ORB_ID::sensor_gyro};

	for (uint8_t i = 0; i < sensor_gyro_subs.size(); i++) {
		sensor_gyro_s sensor_gyro;

		if (sensor_gyro_subs[i].update(&sensor_gyro) && sensor_gyro.device_id != 0) {
			worker_data.calibrations[total_gyros].set_device_id(sensor_gyro.device_id);
			total_gyros++;
		}
	}

	unsigned try_count = 0;
	unsigned max_tries = 20;
	res = PX4_ERROR;

	do {
		// Calibrate gyro and ensure user didn't move
		calibrate_return cal_return = gyro_calibration_worker(worker_data);

		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already sent, we are done here
			res = PX4_ERROR;
			break;

		} else if (cal_return == calibrate_return_error) {
			res = PX4_ERROR;

		} else {
			/* check offsets using a median filter */
			float xdiff = worker_data.filter[0].median() - worker_data.offset[0](0);
			float ydiff = worker_data.filter[1].median() - worker_data.offset[0](1);
			float zdiff = worker_data.filter[2].median() - worker_data.offset[0](2);

			/* maximum allowable calibration error */
			static constexpr float maxoff = math::radians(0.6f);

			if (!PX4_ISFINITE(worker_data.offset[0](0)) ||
			    !PX4_ISFINITE(worker_data.offset[0](1)) ||
			    !PX4_ISFINITE(worker_data.offset[0](2)) ||
			    fabsf(xdiff) > maxoff || fabsf(ydiff) > maxoff || fabsf(zdiff) > maxoff) {

				calibration_log_critical(mavlink_log_pub, "motion, retrying..");
				res = PX4_ERROR;

			} else {
				res = PX4_OK;
			}
		}

		try_count++;

	} while (res == PX4_ERROR && try_count <= max_tries);

	if (try_count >= max_tries) {
		calibration_log_critical(mavlink_log_pub, "ERROR: Motion during calibration");
		res = PX4_ERROR;
	}

	FactoryCalibrationStorage factory_storage;

	if (factory_storage.open() != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, "ERROR: cannot open calibration storage");
		res = PX4_ERROR;
	}

	if (res == PX4_OK) {
		// set offset parameters to new values
		bool param_save = false;
		bool failed = true;

		for (unsigned i = 0; i < MAX_GYROS; i++) {

			auto &calibration = worker_data.calibrations[i];

			if (calibration.device_id() != 0) {
				calibration.set_offset(worker_data.offset[i]);
				calibration.PrintStatus();

				if (calibration.ParametersSave(i, true)) {
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
