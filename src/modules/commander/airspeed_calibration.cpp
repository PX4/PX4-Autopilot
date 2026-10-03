/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file airspeed_calibration.cpp
 * Airspeed sensor calibration routine
 */

#include "airspeed_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/sensor_calibration/DifferentialPressure.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/differential_pressure.h>
#include <systemlib/mavlink_log.h>
#include <parameters/param.h>
#include <systemlib/err.h>

using namespace time_literals;

static const char *sensor_name = "airspeed";

static void feedback_calibration_failed(orb_advert_t *mavlink_log_pub)
{
	px4_sleep(5);
	calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
}

static constexpr int MAX_SENSORS = calibration::DifferentialPressure::MAX_SENSOR_COUNT;

struct AirspeedSensor {
	uORB::SubscriptionData<differential_pressure_s> sub{ORB_ID(differential_pressure)};
	uint32_t device_id{0};
	float offset{0.f};
	bool connected{false};
};

/**
 * Collect the zero reading of every connected sensor simultaneously.
 */
static int collect_zero_offsets(orb_advert_t *mavlink_log_pub, const hrt_abstime &calibration_started,
				AirspeedSensor sensors[MAX_SENSORS], int num_sensors)
{
	static constexpr unsigned calibration_count = (500 * 2) / 3;

	unsigned calibration_counter = 0;
	float sums[MAX_SENSORS] {};
	unsigned counts[MAX_SENSORS] {};

	while (calibration_counter < calibration_count) {

		if (calibrate_cancel_check(mavlink_log_pub, calibration_started)) {
			return PX4_ERROR;
		}

		bool any_update = false;

		for (int i = 0; i < num_sensors; i++) {
			if (sensors[i].sub.update()) {
				sums[i] += sensors[i].sub.get().differential_pressure_pa;
				counts[i]++;
				any_update = true;
			}
		}

		if (any_update) {
			calibration_counter++;

			if (calibration_counter % (calibration_count / 20) == 0) {
				calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, (calibration_counter * 80) / calibration_count);
			}
		}

		if (hrt_elapsed_time(&calibration_started) > 30_s) {
			feedback_calibration_failed(mavlink_log_pub);
			return PX4_ERROR;
		}

		px4_usleep(10000);
	}

	for (int i = 0; i < num_sensors; i++) {
		if (counts[i] == 0) {
			calibration_log_critical(mavlink_log_pub, "[cal] Airspeed sensor %d stopped reporting", i + 1);
			feedback_calibration_failed(mavlink_log_pub);
			return PX4_ERROR;
		}

		sensors[i].offset = sums[i] / counts[i];

		if (!PX4_ISFINITE(sensors[i].offset)) {
			feedback_calibration_failed(mavlink_log_pub);
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

/** Store (or clear, with an offset of 0) the zero offset of one sensor. */
static bool save_offset(uint32_t device_id, float offset)
{
	calibration::DifferentialPressure calibration{device_id};
	calibration.set_offset(offset);

	return calibration.ParametersSave();
}

/**
 * Only the sensor that failed is cleared
 */
static void clear_offset(const AirspeedSensor &sensor)
{
	save_offset(sensor.device_id, 0.f);

	param_notify_changes();
}

/**
 * Verify that positive pressure reaches the sensor, which catches swapped static and dynamic
 * ports. Each sensor has its own pitot, so this has to be repeated for each of them.
 */
static int check_pitot_direction(orb_advert_t *mavlink_log_pub, const hrt_abstime &calibration_started,
				 AirspeedSensor sensors[MAX_SENSORS], int num_sensors, int index)
{
	static constexpr unsigned maxcount = 500;

	AirspeedSensor &sensor = sensors[index];

	if (num_sensors > 1) {
		calibration_log_critical(mavlink_log_pub, "[cal] Blow into front of pitot %d without touching", index + 1);

	} else {
		calibration_log_critical(mavlink_log_pub, "[cal] Blow into front of pitot without touching");
	}

	const hrt_abstime sensor_started = hrt_absolute_time();

	float differential_pressure_sum = 0.f;
	int differential_pressure_sum_count = 0;
	unsigned calibration_counter = 0;

	while (calibration_counter < maxcount) {

		if (calibrate_cancel_check(mavlink_log_pub, calibration_started)) {
			return PX4_ERROR;
		}

		if (sensor.sub.update()) {
			differential_pressure_sum += sensor.sub.get().differential_pressure_pa;
			differential_pressure_sum_count++;

			const float differential_pressure_pa = (differential_pressure_sum / differential_pressure_sum_count) -
							       sensor.offset;

			if ((differential_pressure_sum_count > 10) && (fabsf(differential_pressure_pa) > 50.f)) {
				if (differential_pressure_pa > 0) {
					calibration_log_info(mavlink_log_pub, "[cal] Positive pressure: OK (%d Pa)", (int)differential_pressure_pa);
					return PX4_OK;
				}

				/* do not allow negative values */
				calibration_log_critical(mavlink_log_pub, "[cal] Negative pressure difference detected (%d Pa)",
							 (int)differential_pressure_pa);
				calibration_log_critical(mavlink_log_pub, "[cal] Swap static and dynamic ports or set SENS_DPRES_REV");

				clear_offset(sensor);
				calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 0);
				feedback_calibration_failed(mavlink_log_pub);
				return PX4_ERROR;
			}

			if (calibration_counter % 300 == 0) {
				calibration_log_info(mavlink_log_pub, "[cal] Create air pressure! (got %d, wanted: 50 Pa)",
						     (int)differential_pressure_pa);
				tune_neutral(true);

				// reset average
				differential_pressure_sum = 0.f;
				differential_pressure_sum_count = 0;
			}

			calibration_counter++;
		}

		if (hrt_elapsed_time(&sensor_started) > 90_s) {
			clear_offset(sensor);
			calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 0);
			feedback_calibration_failed(mavlink_log_pub);
			return PX4_ERROR;
		}

		px4_usleep(10000);
	}

	clear_offset(sensor);
	calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 0);
	feedback_calibration_failed(mavlink_log_pub);
	return PX4_ERROR;
}

int do_airspeed_calibration(orb_advert_t *mavlink_log_pub)
{
	const hrt_abstime calibration_started = hrt_absolute_time();

	/* give directions */
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	// find every connected differential pressure sensor.
	AirspeedSensor sensors[MAX_SENSORS] {};
	int num_sensors = 0;

	for (int i = 0; i < MAX_SENSORS; i++) {
		uORB::SubscriptionData<differential_pressure_s> sub{ORB_ID(differential_pressure), (uint8_t)i};

		if (sub.advertised() && sub.get().timestamp != 0) {
			sensors[num_sensors].sub.ChangeInstance(i);
			sensors[num_sensors].device_id = sub.get().device_id;
			sensors[num_sensors].connected = true;
			num_sensors++;
		}
	}

	if (num_sensors == 0) {
		calibration_log_critical(mavlink_log_pub, "[cal] No airspeed sensor found");
		feedback_calibration_failed(mavlink_log_pub);
		return PX4_ERROR;
	}

	if (num_sensors > 1) {
		calibration_log_info(mavlink_log_pub, "[cal] Calibrating %d airspeed sensors", num_sensors);
	}

	calibration_log_critical(mavlink_log_pub, "[cal] Ensure sensor is not measuring wind");
	px4_usleep(500 * 1000);

	if (collect_zero_offsets(mavlink_log_pub, calibration_started, sensors, num_sensors) != PX4_OK) {
		return PX4_ERROR;
	}

	for (int i = 0; i < num_sensors; i++) {
		if (!save_offset(sensors[i].device_id, sensors[i].offset)) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_SET_PARAMS_MSG);
			return PX4_ERROR;
		}

		calibration_log_info(mavlink_log_pub, "[cal] Sensor %d offset of %d Pascal", i + 1, (int)sensors[i].offset);
	}

	param_notify_changes();

	/* wait 500 ms to ensure parameter propagated through the system */
	px4_usleep(500 * 1000);

	for (int i = 0; i < num_sensors; i++) {
		if (check_pitot_direction(mavlink_log_pub, calibration_started, sensors, num_sensors, i) != PX4_OK) {
			return PX4_ERROR;
		}
	}

	calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 100);

	calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
	tune_neutral(true);

	// This give a chance for the log messages to go out of the queue before someone else stomps on then
	px4_usleep(200000);

	return PX4_OK;
}
