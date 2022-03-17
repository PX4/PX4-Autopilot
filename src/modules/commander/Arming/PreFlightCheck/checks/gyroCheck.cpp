/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "../PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <px4_defines.h>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_gyro.h>

using namespace time_literals;

bool PreFlightCheck::isGyroRequired(const uint8_t instance)
{
	uORB::SubscriptionData<sensor_gyro_s> gyro{ORB_ID(sensor_gyro), instance};
	const uint32_t device_id = static_cast<uint32_t>(gyro.get().device_id);

	bool is_used_by_nav = false;

	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		uORB::SubscriptionData<estimator_status_s> estimator_status_sub{ORB_ID(estimator_status), i};

		if (device_id > 0 && estimator_status_sub.get().gyro_device_id == device_id) {
			is_used_by_nav = true;
			break;
		}
	}

	return is_used_by_nav;
}

bool PreFlightCheck::gyroCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
			       const bool is_mandatory, bool &report_fail)
{
	const bool exists = (orb_exists(ORB_ID(sensor_gyro), instance) == PX4_OK);
	const bool is_required = is_mandatory || isGyroRequired(instance);

	bool is_valid = false;
	bool is_calibration_valid = false;

	if (exists) {
		uORB::SubscriptionData<sensor_gyro_s> gyro{ORB_ID(sensor_gyro), instance};

		is_valid = (gyro.get().device_id != 0) && (gyro.get().timestamp != 0)
			   && (hrt_elapsed_time(&gyro.get().timestamp) < 1_s);

		if (status.hil_state == vehicle_status_s::HIL_STATE_ON) {
			is_calibration_valid = true;

		} else {
			is_calibration_valid = (calibration::FindCurrentCalibrationIndex("GYRO", gyro.get().device_id) >= 0);
		}
	}

	if (report_fail && is_required) {
		if (!exists) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Gyro Sensor %u missing", instance);
			report_fail = false;

		} else if (!is_valid) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: no valid data from Gyro %u", instance);
			report_fail = false;

		} else if (!is_calibration_valid) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Gyro %u uncalibrated", instance);
			report_fail = false;
		}
	}

	const bool is_sensor_ok = is_calibration_valid && is_valid;

	return is_sensor_ok || !is_required;
}
