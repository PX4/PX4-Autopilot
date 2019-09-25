/****************************************************************************
*
*   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
* @file PreflightCheck.cpp
*
* Preflight check for main system components
*
* @author Lorenz Meier <lorenz@px4.io>
* @author Johan Jansen <jnsn.johan@gmail.com>
*/

#include "PreflightCheck.h"
#include "health_flag_helper.h"
#include "rc_check.h"

#include <math.h>
#include <mathlib/mathlib.h>

#include <parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>

#include <sensors/common.h>

using namespace time_literals;

namespace Preflight
{

static bool magnometerCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
			    const sensor_preflight_s &sensor_preflight, bool report_fail)
{
	bool failed = false;

	int32_t sys_has_mag = 1;
	param_get(param_find("SYS_HAS_MAG"), &sys_has_mag);

	if (sys_has_mag != 1) {
		return true;
	}

	int32_t required_mags = 1;
	param_get(param_find("PRFLT_REQ_MAG"), &required_mags);

	int32_t angle_difference_limit_deg = 0;
	param_get(param_find("COM_ARM_MAG_ANG"), &angle_difference_limit_deg);

	uint8_t present_cnt = 0;
	uint8_t calibrated_cnt = 0;
	uint8_t healthy_cnt = 0;
	uint8_t enabled_cnt = 0;

	/* run through all mag sensors and check health */
	for (unsigned i = 0; i < sensors::MAG_COUNT_MAX; i++) {
		uint64_t type;

		if (i == 0) { type = subsystem_info_s::SUBSYSTEM_TYPE_MAG; }

		else if (i == 1) { type = subsystem_info_s::SUBSYSTEM_TYPE_MAG2; }

		else if (i == 2) { type = subsystem_info_s::SUBSYSTEM_TYPE_MAG3; }

		else if (i == 3) { type = subsystem_info_s::SUBSYSTEM_TYPE_MAG4; }

		else { continue; }

		if ((status.onboard_control_sensors_present & type) != 0) {
			present_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0) {
			enabled_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (status.onboard_control_sensors_health & type) != 0) {
			healthy_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (sensor_preflight.calibrated & type)) {
			calibrated_cnt++;
		}
	}

	if (present_cnt < required_mags) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d compasses required, %d present", required_mags, present_cnt);
		}

		failed = true;
		goto out;
	}

	if (enabled_cnt < required_mags) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d compasses required, %d enabled", required_mags, enabled_cnt);
		}

		failed = true;
		goto out;
	}

	if (healthy_cnt < required_mags) {
		if (report_fail) {
			if (calibrated_cnt < required_mags) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d compasses required, %d calibrated", required_mags,
						     calibrated_cnt);

			} else {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d compasses required, %d healthy", required_mags, healthy_cnt);
			}
		}

		failed = true;
		goto out;
	}

	if (!sensor_preflight.mag_prime_healthy) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Primary compass failure");
		}

		failed = true;
		goto out;
	}

	/* mag consistency checks (need to be performed after the individual checks) */
	if (sensor_preflight.timestamp != 0 && angle_difference_limit_deg >= 0) {
		// Use the difference between sensors to detect a bad calibration, orientation or magnetic interference.
		// If a single sensor is fitted, the value being checked will be zero so this check will always pass.
		if (sensor_preflight.mag_inconsistency_angle > math::radians<float>(angle_difference_limit_deg)) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Compasses %d° inconsistent",
						     static_cast<int>(math::degrees<float>(sensor_preflight.mag_inconsistency_angle)));
				mavlink_log_critical(mavlink_log_pub, "Please check orientations and recalibrate");
			}

			failed = true;
			goto out;
		}
	}

out:
	return !failed;
}

static bool accelerometerCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
			       const sensor_preflight_s &sensor_preflight, bool dynamic, bool report_fail)
{
	bool failed = false;

	int32_t required_accels = 1;
	param_get(param_find("PRFLT_REQ_ACC"), &required_accels);

	int32_t consistency_test_limit = 0;
	param_get(param_find("COM_ARM_IMU_ACC"), &consistency_test_limit);

	uint8_t present_cnt = 0;
	uint8_t calibrated_cnt = 0;
	uint8_t healthy_cnt = 0;
	uint8_t enabled_cnt = 0;

	/* run through all accel sensors and check health */
	for (unsigned i = 0; i < sensors::ACCEL_COUNT_MAX; i++) {
		uint64_t type;

		if (i == 0) { type = subsystem_info_s::SUBSYSTEM_TYPE_ACC; }

		else if (i == 1) { type = subsystem_info_s::SUBSYSTEM_TYPE_ACC2; }

		else if (i == 2) { type = subsystem_info_s::SUBSYSTEM_TYPE_ACC3; }

		else { continue; }

		if ((status.onboard_control_sensors_present & type) != 0) {
			present_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0) {
			enabled_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (status.onboard_control_sensors_health & type) != 0) {
			healthy_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (sensor_preflight.calibrated & type)) {
			calibrated_cnt++;
		}
	}

	if (present_cnt < required_accels) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d accels required, %d present", required_accels, present_cnt);
		}

		failed = true;
		goto out;
	}

	if (enabled_cnt < required_accels) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d accels required, %d enabled", required_accels, enabled_cnt);
		}

		failed = true;
		goto out;
	}

	if (healthy_cnt < required_accels) {
		if (report_fail) {
			if (calibrated_cnt < required_accels) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d accels required, %d calibrated", required_accels,
						     calibrated_cnt);

			} else {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d accels required, %d healthy", required_accels, healthy_cnt);
			}
		}

		failed = true;
		goto out;
	}

	if (!sensor_preflight.accel_prime_healthy) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Primary accelerometer failure");
		}

		failed = true;
		goto out;
	}



	/* accel consistency checks (need to be performed after the individual checks)
	 * Use the difference between sensors to detect a bad calibration.
	 * If a single sensor is fitted, the value being checked will be zero so this check will always pass.
	 */
	if (sensor_preflight.accel_inconsistency_m_s_s > consistency_test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Accels inconsistent - Check Cal");
		}

		failed = true;
		goto out;

	} else if (sensor_preflight.accel_inconsistency_m_s_s > consistency_test_limit * 0.8f) {
		if (report_fail) {
			mavlink_log_info(mavlink_log_pub, "Preflight Advice: Accels inconsistent - Check Cal");
		}
	}

	if (dynamic && (sensor_preflight.accel_magnitude < 4.0f || sensor_preflight.accel_magnitude > 15.0f /* m/s^2 */)) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Accel Range, hold still on arming");
		}

		failed = false;
		goto out;
	}

out:
	return !failed;
}

static bool gyroCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
		      const sensor_preflight_s &sensor_preflight, bool report_fail)
{
	bool failed = false;

	int32_t required_gyros = 1;
	param_get(param_find("PRFLT_REQ_GYRO"), &required_gyros);

	int32_t consistency_test_limit = 0;
	param_get(param_find("COM_ARM_IMU_GYR"), &consistency_test_limit);

	uint8_t present_cnt = 0;
	uint8_t calibrated_cnt = 0;
	uint8_t healthy_cnt = 0;
	uint8_t enabled_cnt = 0;

	/* run through all gyro sensors and check health */
	for (unsigned i = 0; i < sensors::GYRO_COUNT_MAX ; i++) {
		uint64_t type;

		if (i == 0) { type = subsystem_info_s::SUBSYSTEM_TYPE_GYRO; }

		else if (i == 1) { type = subsystem_info_s::SUBSYSTEM_TYPE_GYRO2; }

		else if (i == 2) { type = subsystem_info_s::SUBSYSTEM_TYPE_GYRO3; }

		else { continue; }

		if ((status.onboard_control_sensors_present & type) != 0) {
			present_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0) {
			enabled_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (status.onboard_control_sensors_health & type) != 0) {
			healthy_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (sensor_preflight.calibrated & type)) {
			calibrated_cnt++;
		}
	}

	if (present_cnt < required_gyros) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d gyros required, %d present", required_gyros, present_cnt);
		}

		failed = true;
		goto out;
	}

	if (enabled_cnt < required_gyros) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d gyros required, %d enabled", required_gyros, enabled_cnt);
		}

		failed = true;
		goto out;
	}

	if (healthy_cnt < required_gyros) {
		if (report_fail) {
			if (calibrated_cnt < required_gyros) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d gyros required, %d calibrated", required_gyros,
						     calibrated_cnt);

			} else {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d gyros required, %d healthy", required_gyros, healthy_cnt);
			}
		}

		failed = true;
		goto out;
	}

	if (!sensor_preflight.gyro_prime_healthy) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Primary gyroscope failure");
		}

		failed = true;
		goto out;
	}



	/* gyro consistency checks (need to be performed after the individual checks)
	 * Use the difference between sensors to detect a bad calibration.
	 * If a single sensor is fitted, the value being checked will be zero so this check will always pass.
	 */
	if (sensor_preflight.gyro_inconsistency_rad_s > consistency_test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Gyros inconsistent - Check Cal");
		}

		failed = true;
		goto out;

	} else if (sensor_preflight.gyro_inconsistency_rad_s > consistency_test_limit * 0.5f) {
		if (report_fail) {
			mavlink_log_info(mavlink_log_pub, "Preflight Advice: Gyros inconsistent - Check Cal");
		}
	}

out:
	return !failed;
}

static bool baroCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
		      const sensor_preflight_s &sensor_preflight, bool report_fail)
{
	bool failed = false;

	int32_t sys_has_baro = 1;
	param_get(param_find("SYS_HAS_BARO"), &sys_has_baro);

	if (sys_has_baro != 1) {
		return true;
	}

	int32_t required_baros = 1;
	param_get(param_find("PRFLT_REQ_BARO"), &required_baros);

	uint8_t present_cnt = 0;
	uint8_t calibrated_cnt = 0;
	uint8_t healthy_cnt = 0;
	uint8_t enabled_cnt = 0;

	/* run through all baro sensors and check health */
	for (unsigned i = 0; i < sensors::BARO_COUNT_MAX; i++) {
		uint64_t type = 0;

		if (i == 0) { type = subsystem_info_s::SUBSYSTEM_TYPE_ABSPRESSURE; }

		else { continue; }

		if ((status.onboard_control_sensors_present & type) != 0) {
			present_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0) {
			enabled_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (status.onboard_control_sensors_health & type) != 0) {
			healthy_cnt++;
		}

		if ((status.onboard_control_sensors_present & type) != 0 &&
		    (status.onboard_control_sensors_enabled & type) != 0 &&
		    (sensor_preflight.calibrated & type)) {
			calibrated_cnt++;
		}
	}

	if (present_cnt < required_baros) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d barometer required, %d present", required_baros, present_cnt);
		}

		failed = true;
		goto out;
	}

	if (enabled_cnt < required_baros) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d barometer required, %d enabled", required_baros, enabled_cnt);
		}

		failed = true;
		goto out;
	}

	if (healthy_cnt < required_baros) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: %d barometer required, %d healthy", required_baros, healthy_cnt);
		}

		failed = true;
		goto out;
	}

	if (!sensor_preflight.baro_prime_healthy) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Primary barometer failure");
		}

		failed = true;
		goto out;
	}

out:
	return !failed;
}

static bool airspeedCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, bool optional, bool report_fail,
			  bool prearm)
{
	bool present = true;
	bool success = true;

	uORB::SubscriptionData<airspeed_s> airspeed_sub{ORB_ID(airspeed)};
	airspeed_sub.update();
	const airspeed_s &airspeed = airspeed_sub.get();

	if (hrt_elapsed_time(&airspeed.timestamp) > 1_s) {
		if (report_fail && !optional) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Airspeed Sensor missing");
		}

		present = false;
		success = false;
		goto out;
	}

	/*
	 * Check if voter thinks the confidence is low. High-end sensors might have virtually zero noise
	 * on the bench and trigger false positives of the voter. Therefore only fail this
	 * for a pre-arm check, as then the cover is off and the natural airflow in the field
	 * will ensure there is not zero noise.
	 */
	if (prearm && fabsf(airspeed.confidence) < 0.95f) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Airspeed Sensor stuck");
		}

		present = true;
		success = false;
		goto out;
	}

	/**
	 * Check if airspeed is higher than 4m/s (accepted max) while the vehicle is landed / not flying
	 * Negative and positive offsets are considered. Do not check anymore while arming because pitot cover
	 * might have been removed.
	 */
	if (fabsf(airspeed.indicated_airspeed_m_s) > 4.0f && !prearm) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: check Airspeed Cal or Pitot");
		}

		present = true;
		success = false;
		goto out;
	}

out:
	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_DIFFPRESSURE, present, !optional, success, status);

	return success;
}

static bool powerCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, bool report_fail, bool prearm)
{
	bool success = true;

	if (!prearm) {
		// Ignore power check after arming.
		return true;

	} else {
		uORB::SubscriptionData<system_power_s> system_power_sub{ORB_ID(system_power)};
		system_power_sub.update();
		const system_power_s &system_power = system_power_sub.get();

		if (hrt_elapsed_time(&system_power.timestamp) < 200_ms) {

			/* copy avionics voltage */
			float avionics_power_rail_voltage = system_power.voltage5v_v;

			// avionics rail
			// Check avionics rail voltages
			if (avionics_power_rail_voltage < 4.5f) {
				success = false;

				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Avionics Power low: %6.2f Volt",
							     (double)avionics_power_rail_voltage);
				}

			} else if (avionics_power_rail_voltage < 4.9f) {
				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "CAUTION: Avionics Power low: %6.2f Volt", (double)avionics_power_rail_voltage);
				}

			} else if (avionics_power_rail_voltage > 5.4f) {
				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "CAUTION: Avionics Power high: %6.2f Volt", (double)avionics_power_rail_voltage);
				}
			}
		}
	}

	return success;
}

static bool ekf2Check(orb_advert_t *mavlink_log_pub, vehicle_status_s &vehicle_status, bool optional, bool report_fail,
		      bool enforce_gps_required)
{
	bool success = true; // start with a pass and change to a fail if any test fails
	bool present = true;
	float test_limit = 1.0f; // pass limit re-used for each test

	bool gps_success = true;
	bool gps_present = true;

	// Get estimator status data if available and exit with a fail recorded if not
	uORB::SubscriptionData<estimator_status_s> status_sub{ORB_ID(estimator_status)};
	status_sub.update();
	const estimator_status_s &status = status_sub.get();

	if (status.timestamp == 0) {
		present = false;
		goto out;
	}

	// Check if preflight check performed by estimator has failed
	if (status.pre_flt_fail) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Position unknown");
		}

		success = false;
		goto out;
	}

	// check vertical position innovation test ratio
	param_get(param_find("COM_ARM_EKF_HGT"), &test_limit);

	if (status.hgt_test_ratio > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Height estimate error");
		}

		success = false;
		goto out;
	}

	// check velocity innovation test ratio
	param_get(param_find("COM_ARM_EKF_VEL"), &test_limit);

	if (status.vel_test_ratio > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Velocity estimate error");
		}

		success = false;
		goto out;
	}

	// check horizontal position innovation test ratio
	param_get(param_find("COM_ARM_EKF_POS"), &test_limit);

	if (status.pos_test_ratio > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Position estimate error");
		}

		success = false;
		goto out;
	}

	// check magnetometer innovation test ratio
	param_get(param_find("COM_ARM_EKF_YAW"), &test_limit);

	if (status.mag_test_ratio > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Yaw estimate error");
		}

		success = false;
		goto out;
	}

	// check accelerometer delta velocity bias estimates
	param_get(param_find("COM_ARM_EKF_AB"), &test_limit);

	for (uint8_t index = 13; index < 16; index++) {
		// allow for higher uncertainty in estimates for axes that are less observable to prevent false positives
		// adjust test threshold by 3-sigma
		float test_uncertainty = 3.0f * sqrtf(fmaxf(status.covariances[index], 0.0f));

		if (fabsf(status.states[index]) > test_limit + test_uncertainty) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: High Accelerometer Bias");
			}

			success = false;
			goto out;
		}
	}

	// check gyro delta angle bias estimates
	param_get(param_find("COM_ARM_EKF_GB"), &test_limit);

	if (fabsf(status.states[10]) > test_limit || fabsf(status.states[11]) > test_limit
	    || fabsf(status.states[12]) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: High Gyro Bias");
		}

		success = false;
		goto out;
	}

	// If GPS aiding is required, declare fault condition if the required GPS quality checks are failing
	if (enforce_gps_required || report_fail) {
		const bool ekf_gps_fusion = status.control_mode_flags & (1 << estimator_status_s::CS_GPS);
		const bool ekf_gps_check_fail = status.gps_check_fail_flags > 0;

		gps_success = ekf_gps_fusion; // default to success if gps data is fused

		if (ekf_gps_check_fail) {
			if (report_fail) {
				// Only report the first failure to avoid spamming
				const char *message = nullptr;

				if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_GPS_FIX)) {
					message = "Preflight%s: GPS fix too low";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MIN_SAT_COUNT)) {
					message = "Preflight%s: not enough GPS Satellites";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MIN_GDOP)) {
					message = "Preflight%s: GPS GDoP too low";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR)) {
					message = "Preflight%s: GPS Horizontal Pos Error too high";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR)) {
					message = "Preflight%s: GPS Vertical Pos Error too high";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR)) {
					message = "Preflight%s: GPS Speed Accuracy too low";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_DRIFT)) {
					message = "Preflight%s: GPS Horizontal Pos Drift too high";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_DRIFT)) {
					message = "Preflight%s: GPS Vertical Pos Drift too high";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR)) {
					message = "Preflight%s: GPS Hor Speed Drift too high";

				} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR)) {
					message = "Preflight%s: GPS Vert Speed Drift too high";

				} else {
					if (!ekf_gps_fusion) {
						// Likely cause unknown
						message = "Preflight%s: Estimator not using GPS";
						gps_present = false;

					} else {
						// if we land here there was a new flag added and the code not updated. Show a generic message.
						message = "Preflight%s: Poor GPS Quality";
					}
				}

				if (message) {
					if (enforce_gps_required) {
						mavlink_log_critical(mavlink_log_pub, message, " Fail");

					} else {
						mavlink_log_warning(mavlink_log_pub, message, "");
					}
				}
			}

			gps_success = false;

			if (enforce_gps_required) {
				success = false;
				goto out;
			}
		}
	}

out:
	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_AHRS, present, !optional, success && present, vehicle_status);
	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_GPS, gps_present, enforce_gps_required, gps_success, vehicle_status);

	return success;
}

static bool failureDetectorCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status, bool report_fail,
				 bool prearm)
{
	bool success = true;

	if (!prearm) {
		// Ignore failure detector check after arming.
		return true;

	}

	if (status.failure_detector_status != vehicle_status_s::FAILURE_NONE) {
		success = false;

		if (report_fail) {
			if (status.failure_detector_status & vehicle_status_s::FAILURE_ROLL) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Roll failure detected");
			}

			if (status.failure_detector_status & vehicle_status_s::FAILURE_PITCH) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Pitch failure detected");
			}

			if (status.failure_detector_status & vehicle_status_s::FAILURE_ALT) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Altitude failure detected");
			}
		}
	}

	return success;
}

bool preflightCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
		    vehicle_status_flags_s &status_flags, bool checkGNSS, bool reportFailures, bool prearm,
		    const hrt_abstime &time_since_boot)
{
	if (time_since_boot < 2_s) {
		// the airspeed driver filter doesn't deliver the actual value yet
		reportFailures = false;
	}

	const bool hil_enabled = (status.hil_state == vehicle_status_s::HIL_STATE_ON);

	bool checkSensors = !hil_enabled;
	const bool checkRC = (status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT);
	const bool checkDynamic = !hil_enabled;
	const bool checkPower = (status_flags.condition_power_input_valid && !status_flags.circuit_breaker_engaged_power_check);
	const bool checkFailureDetector = true;

	bool checkAirspeed = false;

	/* Perform airspeed check only if circuit breaker is not
	 * engaged and it's not a rotary wing */
	if (!status_flags.circuit_breaker_engaged_airspd_check &&
	    (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || status.is_vtol)) {
		checkAirspeed = true;
	}

	reportFailures = (reportFailures && status_flags.condition_system_hotplug_timeout
			  && !status_flags.condition_calibration_enabled);

	// get the sensor preflight data
	uORB::SubscriptionData<sensor_preflight_s> senor_preflight_sub{ORB_ID(sensor_preflight)};
	senor_preflight_sub.update();
	const sensor_preflight_s &sensor_preflight = senor_preflight_sub.get();

	bool failed = false;

	/* ---- MAG ---- */
	if (checkSensors) {
		if (!magnometerCheck(mavlink_log_pub, status, sensor_preflight, reportFailures)) {
			failed = true;
		}
	}

	/* ---- ACCEL ---- */
	if (checkSensors) {
		if (!accelerometerCheck(mavlink_log_pub, status, sensor_preflight, checkDynamic, reportFailures)) {
			failed = true;
		}
	}

	/* ---- GYRO ---- */
	if (checkSensors) {
		if (!gyroCheck(mavlink_log_pub, status, sensor_preflight, reportFailures)) {
			failed = true;
		}
	}

	/* ---- BARO ---- */
	if (checkSensors) {
		if (!baroCheck(mavlink_log_pub, status, sensor_preflight, reportFailures)) {
			failed = true;
		}
	}

	/* ---- AIRSPEED ---- */
	if (checkAirspeed) {
		int32_t optional = 0;
		param_get(param_find("FW_ARSP_MODE"), &optional);

		if (!airspeedCheck(mavlink_log_pub, status, (bool)optional, reportFailures && !failed, prearm) && !(bool)optional) {
			failed = true;
		}
	}

	/* ---- RC CALIBRATION ---- */
	if (checkRC) {
		if (rc_calibration_check(mavlink_log_pub, reportFailures && !failed, status.is_vtol) != OK) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "RC calibration check failed");
			}

			failed = true;

			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true, false, status);
			status_flags.rc_calibration_valid = false;

		} else {
			// The calibration is fine, but only set the overall health state to true if the signal is not currently lost
			status_flags.rc_calibration_valid = true;
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true,
					 !status.rc_signal_lost, status);
		}
	}

	/* ---- SYSTEM POWER ---- */
	if (checkPower) {
		if (!powerCheck(mavlink_log_pub, status, (reportFailures && !failed), prearm)) {
			failed = true;
		}
	}

	/* ---- Navigation EKF ---- */
	// only check EKF2 data if EKF2 is selected as the estimator and GNSS checking is enabled
	int32_t estimator_type = -1;

	if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol) {
		param_get(param_find("SYS_MC_EST_GROUP"), &estimator_type);

	} else {
		// EKF2 is currently the only supported option for FW & VTOL
		estimator_type = 2;
	}

	if (estimator_type == 2) {
		// don't report ekf failures for the first 10 seconds to allow time for the filter to start
		bool report_ekf_fail = (time_since_boot > 10_s);

		if (!ekf2Check(mavlink_log_pub, status, false, reportFailures && report_ekf_fail && !failed, checkGNSS)) {
			failed = true;
		}
	}

	/* ---- Failure Detector ---- */
	if (checkFailureDetector) {
		if (!failureDetectorCheck(mavlink_log_pub, status, (reportFailures && !failed), prearm)) {
			failed = true;
		}
	}

	status.preflight_checks_result = !failed;

	/* Report status */
	return !failed;
}

}
