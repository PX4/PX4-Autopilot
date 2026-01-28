/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file baro_calibration.cpp
 *
 * Simple barometer calibration routine
 */

#include "baro_calibration.h"

#include "commander_helper.h"
#include "calibration_routines.h"
#include "calibration_messages.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <lib/atmosphere/atmosphere.h>
#include <lib/sensor_calibration/Barometer.hpp>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <lib/systemlib/err.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>

using namespace matrix;
using namespace time_literals;
using namespace atmosphere;

static constexpr char sensor_name[] {"baro"};

static constexpr int MAX_SENSOR_COUNT = 4;

int do_baro_calibration(orb_advert_t *mavlink_log_pub)
{
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	// GPS (used for reference)
	uORB::SubscriptionMultiArray<sensor_gps_s, 3> sensor_gps_subs{ORB_ID::sensor_gps};
	float gps_altitude_sum = NAN;
	int gps_altitude_sum_count = 0;


	uORB::SubscriptionMultiArray<sensor_baro_s, MAX_SENSOR_COUNT> sensor_baro_subs{ORB_ID::sensor_baro};
	calibration::Barometer calibration[MAX_SENSOR_COUNT] {};

	uint64_t timestamp_sample_sum[MAX_SENSOR_COUNT] {0};
	float data_sum[MAX_SENSOR_COUNT] {};
	float temperature_sum[MAX_SENSOR_COUNT] {};
	int data_sum_count[MAX_SENSOR_COUNT] {};

	const hrt_abstime time_start_us = hrt_absolute_time();

	while (hrt_elapsed_time(&time_start_us) < 3_s) {

		for (int instance = 0; instance < MAX_SENSOR_COUNT; instance++) {
			sensor_baro_s sensor_baro;

			while (sensor_baro_subs[instance].update(&sensor_baro)) {
				calibration[instance].set_device_id(sensor_baro.device_id);

				// pressure corrected with offset (if available)
				const float pressure_corrected = calibration[instance].Correct(sensor_baro.pressure);

				timestamp_sample_sum[instance] += sensor_baro.timestamp_sample;
				data_sum[instance] += pressure_corrected;
				temperature_sum[instance] += sensor_baro.temperature;
				data_sum_count[instance]++;
			}
		}

		for (auto &gps_sub : sensor_gps_subs) {
			sensor_gps_s sensor_gps;

			if (gps_sub.update(&sensor_gps)) {
				if ((hrt_elapsed_time(&sensor_gps.timestamp) < 1_s)
				    && (sensor_gps.fix_type >= 2) && (sensor_gps.epv < 100)) {

					float alt = (float)sensor_gps.altitude_msl_m;

					if (PX4_ISFINITE(gps_altitude_sum)) {
						gps_altitude_sum += alt;
						gps_altitude_sum_count++;

					} else {
						gps_altitude_sum = alt;
						gps_altitude_sum_count = 1;
					}
				}
			}
		}

		px4_usleep(100_ms);
	}

	float gps_altitude = NAN;

	if (PX4_ISFINITE(gps_altitude_sum) && (gps_altitude_sum_count > 0)) {
		gps_altitude = gps_altitude_sum / gps_altitude_sum_count;
	}

	if (!PX4_ISFINITE(gps_altitude)) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "GPS required for baro cal");
		return PX4_ERROR;
	}

	bool param_save = false;

	for (int instance = 0; instance < MAX_SENSOR_COUNT; instance++) {
		if ((calibration[instance].device_id() != 0) && (data_sum_count[instance] > 0)) {

			const float pressure_pa = data_sum[instance] / data_sum_count[instance];
			const float temperature = temperature_sum[instance] / data_sum_count[instance];

			float pressure_altitude = getAltitudeFromPressure(pressure_pa, temperature);

			// Use GPS altitude as a reference to compute the baro bias measurement
			const float baro_bias = pressure_altitude - gps_altitude;

			float altitude = pressure_altitude - baro_bias;

			// find pressure offset that aligns baro altitude with GPS via binary search
			float front = -10000.f;
			float middle = NAN;
			float last = 10000.f;

			float bias = NAN;

			// perform a binary search
			while (front <= last) {
				middle = front + (last - front) / 2;
				float altitude_calibrated = getAltitudeFromPressure(pressure_pa - middle, temperature);

				if (altitude_calibrated > altitude + 0.1f) {
					last = middle;

				} else if (altitude_calibrated < altitude - 0.1f) {
					front = middle;

				} else {
					bias = middle;
					break;
				}
			}

			if (PX4_ISFINITE(bias)) {
				float offset = calibration[instance].BiasCorrectedSensorOffset(bias);

				calibration[instance].set_offset(offset);

				if (calibration[instance].ParametersSave(instance, true)) {
					calibration[instance].PrintStatus();
					param_save = true;
				}
			}
		}
	}

	if (param_save) {
		param_notify_changes();
	}

	calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
	return PX4_OK;
}
