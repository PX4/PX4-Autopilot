/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file PreFlightCheck.cpp
 */

#include "PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>

using namespace time_literals;

static constexpr unsigned max_mandatory_mag_count = 1;
static constexpr unsigned max_mandatory_gyro_count = 1;
static constexpr unsigned max_mandatory_accel_count = 1;
static constexpr unsigned max_mandatory_baro_count = 1;

bool PreFlightCheck::preflightCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
				    vehicle_status_flags_s &status_flags, const vehicle_control_mode_s &control_mode,
				    bool report_failures,
				    const bool safety_button_available, const bool safety_off,
				    const bool is_arm_attempt)
{
	report_failures = (report_failures && !status_flags.calibration_enabled);

	bool failed = false;

	failed = failed || !airframeCheck(mavlink_log_pub, status);
	failed = failed || !sdcardCheck(mavlink_log_pub, status_flags.sd_card_detected_once, report_failures);

	/* ---- MAG ---- */
	{
		int32_t sys_has_mag = 1;
		param_get(param_find("SYS_HAS_MAG"), &sys_has_mag);

		if (sys_has_mag == 1) {
			failed |= !sensorAvailabilityCheck(report_failures, max_mandatory_mag_count,
							   mavlink_log_pub, status, magnetometerCheck);

			/* mag consistency checks (need to be performed after the individual checks) */
			if (!magConsistencyCheck(mavlink_log_pub, status, report_failures)) {
				failed = true;
			}
		}
	}

	/* ---- ACCEL ---- */
	{
		failed |= !sensorAvailabilityCheck(report_failures, max_mandatory_accel_count,
						   mavlink_log_pub, status, accelerometerCheck);

		// TODO: highest priority (from params)
	}

	/* ---- GYRO ---- */
	{
		failed |= !sensorAvailabilityCheck(report_failures, max_mandatory_gyro_count,
						   mavlink_log_pub, status, gyroCheck);

		// TODO: highest priority (from params)
	}

	/* ---- BARO ---- */
	{
		int32_t sys_has_baro = 1;
		param_get(param_find("SYS_HAS_BARO"), &sys_has_baro);

		if (sys_has_baro == 1) {
			static_cast<void>(sensorAvailabilityCheck(report_failures, max_mandatory_baro_count,
					  mavlink_log_pub, status, baroCheck));
		}
	}

	/* ---- IMU CONSISTENCY ---- */
	// To be performed after the individual sensor checks have completed
	{
		if (!imuConsistencyCheck(mavlink_log_pub, status, report_failures)) {
			failed = true;
		}
	}

	/* ---- Distance Sensor ---- */
	{
		int32_t sys_has_num_dist_sens = 0;
		param_get(param_find("SYS_HAS_NUM_DIST"), &sys_has_num_dist_sens);

		if (sys_has_num_dist_sens > 0) {
			static_cast<void>(sensorAvailabilityCheck(report_failures, sys_has_num_dist_sens,
					  mavlink_log_pub, status, distSensCheck));
		}

	}

	/* ---- AIRSPEED ---- */
	/* Perform airspeed check only if circuit breaker is not engaged and it's not a rotary wing */
	if (!status_flags.circuit_breaker_engaged_airspd_check &&
	    (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || status.is_vtol)) {

		int32_t airspeed_mode = 0;
		param_get(param_find("FW_ARSP_MODE"), &airspeed_mode);
		const bool optional = (airspeed_mode == 1);

		int32_t max_airspeed_check_en = 0;
		param_get(param_find("COM_ARM_ARSP_EN"), &max_airspeed_check_en);

		float airspeed_trim = 10.0f;
		param_get(param_find("FW_AIRSPD_TRIM"), &airspeed_trim);

		const float arming_max_airspeed_allowed = airspeed_trim / 2.0f; // set to half of trim airspeed

		if (!airspeedCheck(mavlink_log_pub, status, optional, report_failures, is_arm_attempt, (bool)max_airspeed_check_en,
				   arming_max_airspeed_allowed)
		    && !(bool)optional) {
			failed = true;
		}
	}

	/* ---- RC CALIBRATION ---- */
	int32_t com_rc_in_mode{0};
	param_get(param_find("COM_RC_IN_MODE"), &com_rc_in_mode);

	if (com_rc_in_mode == 0) {
		if (rcCalibrationCheck(mavlink_log_pub, report_failures) != OK) {
			if (report_failures) {
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
	if (status_flags.power_input_valid && !status_flags.circuit_breaker_engaged_power_check) {
		if (!powerCheck(mavlink_log_pub, status, report_failures)) {
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
		const bool ekf_healthy = ekf2Check(mavlink_log_pub, status, report_failures) &&
					 ekf2CheckSensorBias(mavlink_log_pub, report_failures);

		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_AHRS, true, true, ekf_healthy, status);

		if (control_mode.flag_control_attitude_enabled
		    || control_mode.flag_control_velocity_enabled
		    || control_mode.flag_control_position_enabled) {
			// healthy estimator only required for dependent control modes
			failed |= !ekf_healthy;
		}
	}

	/* ---- Failure Detector ---- */
	if (!failureDetectorCheck(mavlink_log_pub, status, report_failures)) {
		failed = true;
	}

	failed = failed || !manualControlCheck(mavlink_log_pub, report_failures);
	failed = failed || !modeCheck(mavlink_log_pub, report_failures, status);
	failed = failed || !cpuResourceCheck(mavlink_log_pub, report_failures);
	failed = failed || !parachuteCheck(mavlink_log_pub, report_failures, status_flags);
	failed = failed || !preArmCheck(mavlink_log_pub, status_flags, control_mode,
					safety_button_available, safety_off, status, report_failures, is_arm_attempt);

	/* Report status */
	return !failed;
}

bool PreFlightCheck::sensorAvailabilityCheck(const bool report_failure,
		const uint8_t nb_mandatory_instances, orb_advert_t *mavlink_log_pub,
		vehicle_status_s &status, sens_check_func_t sens_check)
{
	bool pass_check = true;
	bool report_fail = report_failure;

	/* check all sensors, but fail only for mandatory ones */
	for (uint8_t i = 0u; i < ORB_MULTI_MAX_INSTANCES; i++) {
		const bool is_mandatory = i < nb_mandatory_instances;

		if (!sens_check(mavlink_log_pub, status, i, is_mandatory, report_fail)) {
			pass_check = false;
		}
	}

	return pass_check;
}
