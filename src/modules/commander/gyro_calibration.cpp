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
#include "gyro_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>

static constexpr char sensor_name[] {"gyro"};
static constexpr unsigned MAX_GYROS = 3;

using matrix::Vector3f;

/// Data passed to calibration worker routine
struct gyro_worker_data_t {
	orb_advert_t *mavlink_log_pub{nullptr};
	int32_t device_id[MAX_GYROS] {};
	Vector3f offset[MAX_GYROS] {};

	static constexpr int last_num_samples = 9; ///< number of samples for the motion detection median filter
	float last_sample_0_x[last_num_samples];
	float last_sample_0_y[last_num_samples];
	float last_sample_0_z[last_num_samples];
	int last_sample_0_idx;
};

static int float_cmp(const void *elem1, const void *elem2)
{
	if (*(const float *)elem1 < * (const float *)elem2) {
		return -1;
	}

	return *(const float *)elem1 > *(const float *)elem2;
}

static calibrate_return gyro_calibration_worker(gyro_worker_data_t &worker_data)
{
	const hrt_abstime calibration_started = hrt_absolute_time();
	unsigned calibration_counter[MAX_GYROS] {};
	static constexpr unsigned CALIBRATION_COUNT = 250;
	unsigned poll_errcount = 0;

	uORB::Subscription sensor_correction_sub{ORB_ID(sensor_correction)};
	sensor_correction_s sensor_correction{}; /**< sensor thermal corrections */

	uORB::SubscriptionBlocking<sensor_gyro_s> gyro_sub[MAX_GYROS] {
		{ORB_ID(sensor_gyro), 0, 0},
		{ORB_ID(sensor_gyro), 0, 1},
		{ORB_ID(sensor_gyro), 0, 2},
	};

	memset(&worker_data.last_sample_0_x, 0, sizeof(worker_data.last_sample_0_x));
	memset(&worker_data.last_sample_0_y, 0, sizeof(worker_data.last_sample_0_y));
	memset(&worker_data.last_sample_0_z, 0, sizeof(worker_data.last_sample_0_z));
	worker_data.last_sample_0_idx = 0;

	/* use slowest gyro to pace, but count correctly per-gyro for statistics */
	unsigned slow_count = 0;

	while (slow_count < CALIBRATION_COUNT) {
		if (calibrate_cancel_check(worker_data.mavlink_log_pub, calibration_started)) {
			return calibrate_return_cancelled;
		}

		if (gyro_sub[0].updatedBlocking(100000)) {
			unsigned update_count = CALIBRATION_COUNT;

			for (unsigned gyro_index = 0; gyro_index < MAX_GYROS; gyro_index++) {
				if (worker_data.device_id[gyro_index] != 0) {

					if (calibration_counter[gyro_index] >= CALIBRATION_COUNT) {
						// Skip if instance has enough samples
						continue;
					}

					sensor_gyro_s gyro_report;

					if (gyro_sub[gyro_index].update(&gyro_report)) {

						// fetch optional thermal offset corrections in sensor/board frame
						Vector3f offset{0, 0, 0};
						sensor_correction_sub.update(&sensor_correction);

						if (sensor_correction.timestamp > 0 && gyro_report.device_id != 0) {
							for (uint8_t correction_index = 0; correction_index < MAX_GYROS; correction_index++) {
								if (sensor_correction.gyro_device_ids[correction_index] == gyro_report.device_id) {
									switch (correction_index) {
									case 0:
										offset = Vector3f{sensor_correction.gyro_offset_0};
										break;
									case 1:
										offset = Vector3f{sensor_correction.gyro_offset_1};
										break;
									case 2:
										offset = Vector3f{sensor_correction.gyro_offset_2};
										break;
									}
								}
							}
						}

						worker_data.offset[gyro_index] += Vector3f{gyro_report.x, gyro_report.y, gyro_report.z} - offset;
						calibration_counter[gyro_index]++;

						if (gyro_index == 0) {
							worker_data.last_sample_0_x[worker_data.last_sample_0_idx] = gyro_report.x - offset(0);
							worker_data.last_sample_0_y[worker_data.last_sample_0_idx] = gyro_report.y - offset(1);
							worker_data.last_sample_0_z[worker_data.last_sample_0_idx] = gyro_report.z - offset(2);
							worker_data.last_sample_0_idx = (worker_data.last_sample_0_idx + 1) % gyro_worker_data_t::last_num_samples;
						}
					}

					// Maintain the sample count of the slowest sensor
					if (calibration_counter[gyro_index] && calibration_counter[gyro_index] < update_count) {
						update_count = calibration_counter[gyro_index];
					}
				}
			}

			if (update_count % (CALIBRATION_COUNT / 20) == 0) {
				calibration_log_info(worker_data.mavlink_log_pub, CAL_QGC_PROGRESS_MSG, (update_count * 100) / CALIBRATION_COUNT);
			}

			// Propagate out the slowest sensor's count
			if (slow_count < update_count) {
				slow_count = update_count;
			}

		} else {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {
			calibration_log_critical(worker_data.mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			return calibrate_return_error;
		}
	}

	for (unsigned s = 0; s < MAX_GYROS; s++) {
		if ((worker_data.device_id[s] != 0) && (calibration_counter[s] < CALIBRATION_COUNT / 2)) {
			calibration_log_critical(worker_data.mavlink_log_pub, "ERROR: missing data, sensor %d", s)
			return calibrate_return_error;
		}

		worker_data.offset[s] /= calibration_counter[s];
	}

	return calibrate_return_ok;
}

int do_gyro_calibration(orb_advert_t *mavlink_log_pub)
{
	int res = PX4_OK;

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	gyro_worker_data_t worker_data{};
	worker_data.mavlink_log_pub = mavlink_log_pub;

	int32_t enabled[MAX_GYROS] {1, 1, 1};

	ORB_PRIO device_prio_max = ORB_PRIO_UNINITIALIZED;
	int32_t device_id_primary = 0;

	for (uint8_t cur_gyro = 0; cur_gyro < MAX_GYROS; cur_gyro++) {

		uORB::SubscriptionData<sensor_gyro_s> gyro_sub{ORB_ID(sensor_gyro), cur_gyro};
		gyro_sub.update();

		if (gyro_sub.advertised() && (gyro_sub.get().device_id != 0) && (gyro_sub.get().timestamp > 0)) {

			worker_data.device_id[cur_gyro] = gyro_sub.get().device_id;

			// Get priority
			ORB_PRIO prio = gyro_sub.get_priority();

			if (prio > device_prio_max) {
				device_prio_max = prio;
				device_id_primary = worker_data.device_id[cur_gyro];
			}

			// preserve existing CAL_GYROx_EN parameter
			for (uint8_t cal_index = 0; cal_index < MAX_GYROS; cal_index++) {
				char str[20] {};
				sprintf(str, "CAL_%s%u_ID", "GYRO", cal_index);
				int32_t cal_device_id = 0;

				if (param_get(param_find(str), &cal_device_id) == PX4_OK) {
					if ((cal_device_id != 0) && (cal_device_id == worker_data.device_id[cur_gyro])) {
						// CAL_GYROx_EN
						sprintf(str, "CAL_%s%u_EN", "GYRO", cal_index);
						param_get(param_find(str), &enabled[cur_gyro]);
					}
				}
			}

		} else {
			worker_data.device_id[cur_gyro] = 0;
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
			qsort(worker_data.last_sample_0_x, gyro_worker_data_t::last_num_samples, sizeof(float), float_cmp);
			qsort(worker_data.last_sample_0_y, gyro_worker_data_t::last_num_samples, sizeof(float), float_cmp);
			qsort(worker_data.last_sample_0_z, gyro_worker_data_t::last_num_samples, sizeof(float), float_cmp);

			float xdiff = worker_data.last_sample_0_x[gyro_worker_data_t::last_num_samples / 2] - worker_data.offset[0](0);
			float ydiff = worker_data.last_sample_0_y[gyro_worker_data_t::last_num_samples / 2] - worker_data.offset[0](1);
			float zdiff = worker_data.last_sample_0_z[gyro_worker_data_t::last_num_samples / 2] - worker_data.offset[0](2);

			/* maximum allowable calibration error */
			const float maxoff = math::radians(0.6f);

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

	if (res == PX4_OK) {

		/* set offset parameters to new values */
		bool failed = (PX4_OK != param_set_no_notification(param_find("CAL_GYRO_PRIME"), &device_id_primary));

		for (unsigned uorb_index = 0; uorb_index < MAX_GYROS; uorb_index++) {

			int32_t device_id = 0;
			Vector3f offset;

			if (worker_data.device_id[uorb_index] != 0) {
				device_id = worker_data.device_id[uorb_index];
				offset = worker_data.offset[uorb_index];

				PX4_INFO("[cal] %s %u offset: [%.4f %.4f %.4f]", "GYRO", device_id,
					 (double)offset(0), (double)offset(1), (double)offset(2));

			} else {
				// all unused parameters set to default values
				device_id = 0;
				enabled[uorb_index] = 1;
				offset.zero();
			}

			char str[20] {};

			sprintf(str, "CAL_%s%u_ID", "GYRO", uorb_index);
			param_set_no_notification(param_find(str), &device_id);
			sprintf(str, "CAL_%s%u_EN", "GYRO", uorb_index);
			param_set_no_notification(param_find(str), &enabled[uorb_index]);

			for (int axis = 0; axis < 3; axis++) {
				char axis_char = 'X' + axis;

				// offsets
				sprintf(str, "CAL_%s%u_%cOFF", "GYRO", uorb_index, axis_char);
				param_set_no_notification(param_find(str), &offset(axis));
			}
		}

		if (failed) {
			calibration_log_critical(mavlink_log_pub, "ERROR: failed to set offset params");
			res = PX4_ERROR;
		}
	}

	param_notify_changes();

	/* if there is a any preflight-check system response, let the barrage of messages through */
	px4_usleep(200000);

	if (res == PX4_OK) {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);

	} else {
		calibration_log_info(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	}

	/* give this message enough time to propagate */
	px4_usleep(600000);

	return res;
}
