/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include "estimatorCheck.hpp"

using namespace time_literals;

EstimatorChecks::EstimatorChecks()
{
	// initially set to failed
	_last_lpos_fail_time_us = hrt_absolute_time();
	_last_lpos_relaxed_fail_time_us = _last_lpos_fail_time_us;
	_last_gpos_fail_time_us = _last_lpos_fail_time_us;
	_last_lvel_fail_time_us = _last_lpos_fail_time_us;
}

void EstimatorChecks::checkAndReport(const Context &context, Report &reporter)
{
	sensor_gps_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.copy(&vehicle_gps_position)) {
		checkGps(context, reporter, vehicle_gps_position);

	} else {
		vehicle_gps_position = {};
	}

	vehicle_local_position_s lpos;

	if (!_vehicle_local_position_sub.copy(&lpos)) {
		lpos = {};
	}

	bool pre_flt_fail_innov_heading = false;
	bool pre_flt_fail_innov_vel_horiz = false;
	bool pre_flt_fail_innov_pos_horiz = false;
	bool missing_data = false;
	const NavModes required_groups = (NavModes)reporter.failsafeFlags().mode_req_attitude;

	// Change topics to primary estimator instance
	if (_param_sens_imu_mode.get() == 0) { // multi-ekf
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			bool instance_changed = _estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance)
						&& _estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance)
						&& _estimator_status_flags_sub.ChangeInstance(estimator_selector_status.primary_instance);

			if (!instance_changed) {
				missing_data = true;
			}

		} else {
			missing_data = true;
		}
	}

	if (!missing_data) {
		estimator_status_s estimator_status;

		if (_estimator_status_sub.copy(&estimator_status)) {
			pre_flt_fail_innov_heading = estimator_status.pre_flt_fail_innov_heading;
			pre_flt_fail_innov_vel_horiz = estimator_status.pre_flt_fail_innov_vel_horiz;
			pre_flt_fail_innov_pos_horiz = estimator_status.pre_flt_fail_innov_pos_horiz;

			checkEstimatorStatus(context, reporter, estimator_status, required_groups);
			checkEstimatorStatusFlags(context, reporter, estimator_status, lpos);

		} else {
			missing_data = true;
		}
	}

	param_t param_ekf2_en_handle = param_find_no_notification("EKF2_EN");
	int32_t param_ekf2_en = 0;

	if (param_ekf2_en_handle != PARAM_INVALID) {
		param_get(param_ekf2_en_handle, &param_ekf2_en);
	}

	if (missing_data && (param_ekf2_en == 1)) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_missing_data"),
					    events::Log::Info, "Waiting for estimator to initialize");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: ekf2 missing data");
		}

	} else {
		reporter.setIsPresent(health_component_t::local_position_estimate);
		checkSensorBias(context, reporter, required_groups);
	}

	// set mode requirements
	setModeRequirementFlags(context, pre_flt_fail_innov_heading, pre_flt_fail_innov_vel_horiz, pre_flt_fail_innov_pos_horiz,
				lpos, vehicle_gps_position,
				reporter.failsafeFlags(), reporter);


	lowPositionAccuracy(context, reporter, lpos);
}

void EstimatorChecks::checkEstimatorStatus(const Context &context, Report &reporter,
		const estimator_status_s &estimator_status, NavModes required_groups)
{
	if (!context.isArmed() && estimator_status.pre_flt_fail_innov_heading) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_heading_not_stable"),
					    events::Log::Error, "Heading estimate not stable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: heading estimate not stable");
		}

	} else if (!context.isArmed() && estimator_status.pre_flt_fail_innov_vel_horiz) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_hor_vel_not_stable"),
					    events::Log::Error, "Horizontal velocity unstable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: horizontal velocity unstable");
		}

	} else if (!context.isArmed() && estimator_status.pre_flt_fail_innov_vel_vert) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_vert_vel_not_stable"),
					    events::Log::Error, "Vertical velocity unstable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: vertical velocity unstable");
		}

	} else if (!context.isArmed() && estimator_status.pre_flt_fail_innov_pos_horiz) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_hor_pos_not_stable"),
					    events::Log::Error, "Horizontal position unstable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: horizontal position unstable");
		}

	} else if (!context.isArmed() && estimator_status.pre_flt_fail_innov_height) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_hgt_not_stable"),
					    events::Log::Error, "Height estimate not stable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: height estimate not stable");
		}
	}


	if ((_param_com_arm_mag_str.get() >= 1)
	    && (!context.isArmed() && estimator_status.pre_flt_fail_mag_field_disturbed)) {

		NavModes required_groups_mag = required_groups;

		if (_param_com_arm_mag_str.get() != 1) {
			required_groups_mag = NavModes::None; // optional
		}

		/* EVENT
		 * @description
		 * <profile name="dev">
		 * Measured strength: {1:.3}, expected: {2:.3} ± <param>EKF2_MAG_CHK_STR</param>
		 * Measured inclination: {3:.3}, expected: {4:.3} ± <param>EKF2_MAG_CHK_INC</param>
		 * This check can be configured via <param>COM_ARM_MAG_STR</param> and <param>EKF2_MAG_CHECK</param> parameters.
		 * </profile>
		 */
		reporter.armingCheckFailure<float, float, float, float>(required_groups_mag,
				health_component_t::local_position_estimate,
				events::ID("check_estimator_mag_interference"),
				events::Log::Warning, "Strong magnetic interference",
				estimator_status.mag_strength_gs, estimator_status.mag_strength_ref_gs,
				estimator_status.mag_inclination_deg, estimator_status.mag_inclination_ref_deg);

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Strong magnetic interference");
		}
	}

	// If GPS aiding is required, declare fault condition if the required GPS quality checks are failing
	if (_param_sys_has_gps.get()) {
		const bool ekf_gps_fusion = estimator_status.control_mode_flags & (1 << estimator_status_s::CS_GPS);
		const bool ekf_gps_check_fail = estimator_status.gps_check_fail_flags > 0;

		if (ekf_gps_fusion) {
			reporter.setIsPresent(health_component_t::gps); // should be based on the sensor data directly
		}

		if (context.isArmed()) {

			if (_gps_was_fused && !ekf_gps_fusion) {
				if (reporter.mavlink_log_pub()) {
					mavlink_log_warning(reporter.mavlink_log_pub(), "GNSS data fusion stopped\t");
				}

				// only report this failure as critical if not already in a local position invalid state
				events::Log log_level = reporter.failsafeFlags().local_position_invalid ? events::Log::Info : events::Log::Error;
				events::send(events::ID("check_estimator_gnss_fusion_stopped"), {log_level, events::LogInternal::Info},
					     "GNSS data fusion stopped");

			} else if (!_gps_was_fused && ekf_gps_fusion) {

				if (reporter.mavlink_log_pub()) {
					mavlink_log_info(reporter.mavlink_log_pub(), "GNSS data fusion started\t");
				}

				events::send(events::ID("check_estimator_gnss_fusion_started"), {events::Log::Info, events::LogInternal::Info},
					     "GNSS data fusion started");
			}
		}

		_gps_was_fused = ekf_gps_fusion;

		if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_SPOOFED)) {
			if (!_gnss_spoofed) {
				_gnss_spoofed = true;

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "GNSS signal spoofed\t");
				}

				events::send(events::ID("check_estimator_gnss_warning_spoofing"), {events::Log::Alert, events::LogInternal::Info},
					     "GNSS signal spoofed");
			}

		} else {
			_gnss_spoofed = false;
		}

		if (!context.isArmed() && ekf_gps_check_fail) {
			NavModes required_groups_gps;
			events::Log log_level;

			switch (static_cast<GnssArmingCheck>(_param_com_arm_wo_gps.get())) {
			default:

			/* FALLTHROUGH */
			case GnssArmingCheck::DenyArming:
				required_groups_gps = required_groups;
				log_level = events::Log::Error;
				break;

			case GnssArmingCheck::WarningOnly:
				required_groups_gps = NavModes::None; // optional
				log_level = events::Log::Warning;
				break;

			case GnssArmingCheck::Disabled:
				required_groups_gps = NavModes::None;
				log_level = events::Log::Disabled;
				break;
			}

			// Only report the first failure to avoid spamming
			const char *message = nullptr;

			if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_GPS_FIX)) {
				message = "Preflight%s: GPS fix too low";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_fix_too_low"),
							    log_level, "GPS fix too low");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MIN_SAT_COUNT)) {
				message = "Preflight%s: not enough GPS Satellites";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_num_sats_too_low"),
							    log_level, "Not enough GPS Satellites");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_PDOP)) {
				message = "Preflight%s: GPS PDOP too high";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_pdop_too_high"),
							    log_level, "GPS PDOP too high");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR)) {
				message = "Preflight%s: GPS Horizontal Pos Error too high";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_hor_pos_err_too_high"),
							    log_level, "GPS Horizontal Position Error too high");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR)) {
				message = "Preflight%s: GPS Vertical Pos Error too high";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_vert_pos_err_too_high"),
							    log_level, "GPS Vertical Position Error too high");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR)) {
				message = "Preflight%s: GPS Speed Accuracy too low";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_speed_acc_too_low"),
							    log_level, "GPS Speed Accuracy too low");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_DRIFT)) {
				message = "Preflight%s: GPS Horizontal Pos Drift too high";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_hor_pos_drift_too_high"),
							    log_level, "GPS Horizontal Position Drift too high");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_DRIFT)) {
				message = "Preflight%s: GPS Vertical Pos Drift too high";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_vert_pos_drift_too_high"),
							    log_level, "GPS Vertical Position Drift too high");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR)) {
				message = "Preflight%s: GPS Hor Speed Drift too high";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_hor_speed_drift_too_high"),
							    log_level, "GPS Horizontal Speed Drift too high");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR)) {
				message = "Preflight%s: GPS Vert Speed Drift too high";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_vert_speed_drift_too_high"),
							    log_level, "GPS Vertical Speed Drift too high");

			} else if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_SPOOFED)) {
				message = "Preflight%s: GPS signal spoofed";
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_spoofed"),
							    log_level, "GPS signal spoofed");

			} else {
				if (!ekf_gps_fusion) {
					// Likely cause unknown
					message = "Preflight%s: Estimator not using GPS";
					/* EVENT
					 */
					reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
								    events::ID("check_estimator_gps_not_fusing"),
								    log_level, "Estimator not using GPS");

				} else {
					// if we land here there was a new flag added and the code not updated. Show a generic message.
					message = "Preflight%s: Poor GPS Quality";
					/* EVENT
					 */
					reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
								    events::ID("check_estimator_gps_generic"),
								    log_level, "Poor GPS Quality");
				}
			}

			if (message && reporter.mavlink_log_pub()) {
				switch (static_cast<GnssArmingCheck>(_param_com_arm_wo_gps.get())) {
				default:

				/* FALLTHROUGH */
				case GnssArmingCheck::DenyArming:
					mavlink_log_critical(reporter.mavlink_log_pub(), message, " Fail");
					break;

				case GnssArmingCheck::WarningOnly:
					mavlink_log_warning(reporter.mavlink_log_pub(), message, "");
					break;

				case GnssArmingCheck::Disabled:
					break;
				}
			}
		}
	}

}

void EstimatorChecks::checkSensorBias(const Context &context, Report &reporter, NavModes required_groups)
{
	// _estimator_sensor_bias_sub instance got changed above already
	estimator_sensor_bias_s bias;

	if (_estimator_sensor_bias_sub.copy(&bias) && hrt_elapsed_time(&bias.timestamp) < 30_s) {

		// check accelerometer bias estimates
		if (bias.accel_bias_valid) {
			const float ekf_ab_test_limit = 0.75f * bias.accel_bias_limit;

			for (int axis_index = 0; axis_index < 3; axis_index++) {
				// allow for higher uncertainty in estimates for axes that are less observable to prevent false positives
				// adjust test threshold by 3-sigma
				const float test_uncertainty = 3.0f * sqrtf(fmaxf(bias.accel_bias_variance[axis_index], 0.0f));

				if (fabsf(bias.accel_bias[axis_index]) > ekf_ab_test_limit + test_uncertainty) {
					/* EVENT
					 * @description
					 * An accelerometer recalibration might help.
					 *
					 * <profile name="dev">
					 * Axis {1}: |{2:.8}| \> {3:.8} + {4:.8}
					 *
					 * This check can be configured via <param>EKF2_ABL_LIM</param> parameter.
					 * </profile>
					 */
					reporter.armingCheckFailure<uint8_t, float, float, float>(required_groups, health_component_t::local_position_estimate,
							events::ID("check_estimator_high_accel_bias"),
							events::Log::Error, "High Accelerometer Bias", axis_index,
							bias.accel_bias[axis_index], ekf_ab_test_limit, test_uncertainty);

					if (reporter.mavlink_log_pub()) {
						mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: High Accelerometer Bias");
					}

					return; // avoid showing more than one error
				}
			}
		}

		// check gyro bias estimates
		if (bias.gyro_bias_valid) {
			const float ekf_gb_test_limit = 0.75f * bias.gyro_bias_limit;

			for (int axis_index = 0; axis_index < 3; axis_index++) {
				// allow for higher uncertainty in estimates for axes that are less observable to prevent false positives
				// adjust test threshold by 3-sigma
				const float test_uncertainty = 3.0f * sqrtf(fmaxf(bias.gyro_bias_variance[axis_index], 0.0f));

				if (fabsf(bias.gyro_bias[axis_index]) > ekf_gb_test_limit + test_uncertainty) {
					/* EVENT
					 * @description
					 * A Gyro recalibration might help.
					 *
					 * <profile name="dev">
					 * Axis {1}: |{2:.8}| \> {3:.8} + {4:.8}
					 *
					 * This check can be configured via <param>EKF2_ABL_GYRLIM</param> parameter.
					 * </profile>
					 */
					reporter.armingCheckFailure<uint8_t, float, float, float>(required_groups, health_component_t::local_position_estimate,
							events::ID("check_estimator_high_gyro_bias"),
							events::Log::Error, "High Gyro Bias", axis_index,
							bias.gyro_bias[axis_index], ekf_gb_test_limit, test_uncertainty);

					if (reporter.mavlink_log_pub()) {
						mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: High Gyro Bias");
					}

					return; // avoid showing more than one error
				}
			}
		}
	}
}

void EstimatorChecks::checkEstimatorStatusFlags(const Context &context, Report &reporter,
		const estimator_status_s &estimator_status, const vehicle_local_position_s &lpos)
{
	estimator_status_flags_s estimator_status_flags;

	if (_estimator_status_flags_sub.copy(&estimator_status_flags)) {
		// Check for a magnetometer fault and notify the user
		if (estimator_status_flags.cs_mag_fault) {
			/* EVENT
			 * @description
			 * Land and calibrate the compass.
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::local_position_estimate,
						    events::ID("check_estimator_mag_fault"),
						    events::Log::Critical, "Stopping compass use");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Compass needs calibration - Land now!\t");
			}
		}

		if (estimator_status_flags.cs_gnss_yaw_fault) {
			/* EVENT
			 * @description
			 * Land now
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::local_position_estimate,
						    events::ID("check_estimator_gnss_fault"),
						    events::Log::Critical, "GNSS heading not reliable");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "GNSS heading not reliable - Land now!\t");
			}
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	/* Check estimator status for signs of bad yaw induced post takeoff navigation failure
	* for a short time interval after takeoff.
	* Most of the time, the drone can recover from a bad initial yaw using GPS-inertial
	* heading estimation (yaw emergency estimator) or GPS heading (fixed wings only), but
	* if this does not fix the issue we need to stop using a position controlled
	* mode to prevent flyaway crashes.
	*/

	if (context.status().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		if (!context.isArmed()) {
			_nav_test_failed = false;
			_nav_test_passed = false;

		} else {
			if (!_nav_test_passed) {
				// Both test ratios need to pass/fail together to change the nav test status
				const bool innovation_pass = (estimator_status.vel_test_ratio < 1.f) && (estimator_status.pos_test_ratio < 1.f)
							     && (estimator_status.vel_test_ratio > FLT_EPSILON) && (estimator_status.pos_test_ratio > FLT_EPSILON);

				const bool innovation_fail = (estimator_status.vel_test_ratio >= 1.f) && (estimator_status.pos_test_ratio >= 1.f);

				if (innovation_pass) {
					_time_last_innov_pass = now;

					// if nav status is unconfirmed, confirm yaw angle as passed after 30 seconds or achieving 5 m/s of speed
					const bool sufficient_time = (context.status().takeoff_time != 0) && (now > context.status().takeoff_time + 30_s);
					const bool sufficient_speed = matrix::Vector2f(lpos.vx, lpos.vy).longerThan(5.f);

					// Even if the test already failed, allow it to pass if it did not fail during the last 10 seconds
					if ((now > _time_last_innov_fail + 10_s) && (sufficient_time || sufficient_speed)) {
						_nav_test_passed = true;
						_nav_test_failed = false;
					}

				} else if (innovation_fail) {
					_time_last_innov_fail = now;

					if (now > _time_last_innov_pass + 2_s) {
						// if the innovation test has failed continuously, declare the nav as failed
						_nav_test_failed = true;
						/* EVENT
						 * @description
						 * Land and recalibrate the sensors.
						 */
						reporter.healthFailure(NavModes::All, health_component_t::local_position_estimate,
								       events::ID("check_estimator_nav_failure"),
								       events::Log::Emergency, "Navigation failure");

						if (reporter.mavlink_log_pub()) {
							mavlink_log_critical(reporter.mavlink_log_pub(), "Navigation failure! Land and recalibrate sensors\t");
						}
					}
				}
			}
		}
	}
}

void EstimatorChecks::checkGps(const Context &context, Report &reporter, const sensor_gps_s &vehicle_gps_position) const
{
	if (vehicle_gps_position.jamming_state == sensor_gps_s::JAMMING_STATE_CRITICAL) {
		/* EVENT
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::gps,
					    events::ID("check_estimator_gps_jamming_critical"),
					    events::Log::Critical, "GPS reports critical jamming state");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "GPS reports critical jamming state\t");
		}
	}

	if (vehicle_gps_position.spoofing_state == sensor_gps_s::SPOOFING_STATE_INDICATED) {
		/* EVENT
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::gps,
					    events::ID("check_estimator_gps_spoofing_indicated"),
					    events::Log::Critical, "GPS reports spoofing indicated");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "GPS reports spoofing indicated\t");
		}

	} else if (vehicle_gps_position.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE) {
		/* EVENT
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::gps,
					    events::ID("check_estimator_gps_multiple_spoofing_indicated"),
					    events::Log::Critical, "GPS reports multiple spoofing indicated");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "GPS reports multiple spoofing indicated\t");
		}
	}
}

void EstimatorChecks::lowPositionAccuracy(const Context &context, Report &reporter,
		const vehicle_local_position_s &lpos) const
{
	const bool local_position_valid_but_low_accuracy = !reporter.failsafeFlags().local_position_invalid
			&& (_param_com_low_eph.get() > FLT_EPSILON && lpos.eph > _param_com_low_eph.get());

	if (!reporter.failsafeFlags().local_position_accuracy_low && local_position_valid_but_low_accuracy
	    && _param_com_pos_low_act.get()) {

		// only report if armed
		if (context.isArmed()) {
			/* EVENT
			 * @description Local position estimate valid but has low accuracy. Warn user.
			 *
			 * <profile name="dev">
			 * This check can be configured via <param>COM_POS_LOW_EPH</param> and <param>COM_POS_LOW_ACT</param> parameters.
			 * </profile>
			 */
			events::send(events::ID("check_estimator_low_position_accuracy"), {events::Log::Error, events::LogInternal::Info},
				     "Local position estimate has low accuracy");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_warning(reporter.mavlink_log_pub(), "Local position estimate has low accuracy\t");
			}
		}
	}

	reporter.failsafeFlags().local_position_accuracy_low = local_position_valid_but_low_accuracy;
}

void EstimatorChecks::setModeRequirementFlags(const Context &context, bool pre_flt_fail_innov_heading,
		bool pre_flt_fail_innov_vel_horiz, bool pre_flt_fail_innov_pos_horiz,
		const vehicle_local_position_s &lpos, const sensor_gps_s &vehicle_gps_position, failsafe_flags_s &failsafe_flags,
		Report &reporter)
{
	// The following flags correspond to mode requirements, and are reported in the corresponding mode checks
	vehicle_global_position_s gpos;

	if (!_vehicle_global_position_sub.copy(&gpos)) {
		gpos = {};
	}

	const hrt_abstime now = hrt_absolute_time();

	// run position and velocity accuracy checks
	// Check if quality checking of position accuracy and consistency is to be performed
	const float lpos_eph_threshold = (_param_com_pos_fs_eph.get() < 0) ? INFINITY : _param_com_pos_fs_eph.get();

	bool xy_valid = lpos.xy_valid && !_nav_test_failed;
	bool v_xy_valid = lpos.v_xy_valid && !_nav_test_failed;

	if (!context.isArmed()) {
		if (pre_flt_fail_innov_heading || pre_flt_fail_innov_pos_horiz) {
			xy_valid = false;
		}

		if (pre_flt_fail_innov_vel_horiz) {
			v_xy_valid = false;
		}
	}

	const bool global_pos_valid = gpos.lat_lon_valid && gpos.alt_valid;

	failsafe_flags.global_position_invalid =
		!checkPosVelValidity(now, global_pos_valid, gpos.eph, lpos_eph_threshold, gpos.timestamp,
				     _last_gpos_fail_time_us, !failsafe_flags.global_position_invalid);

	// Additional warning if the system is about to enter position-loss failsafe after dead-reckoning period
	const float eph_critical = 2.5f * lpos_eph_threshold; // threshold used to trigger the navigation failsafe
	const float gpos_critical_warning_thrld = math::max(0.9f * eph_critical, math::max(eph_critical - 10.f, 0.f));

	estimator_status_flags_s estimator_status_flags;

	if (_estimator_status_flags_sub.copy(&estimator_status_flags)) {

		// only do the following if the estimator status flags are recent (less than 5 seconds old)
		if (now - estimator_status_flags.timestamp < 5_s) {
			const bool dead_reckoning = estimator_status_flags.cs_inertial_dead_reckoning
						    || estimator_status_flags.cs_wind_dead_reckoning;

			if (!failsafe_flags.global_position_invalid
			    && !_nav_failure_imminent_warned
			    && gpos.eph > gpos_critical_warning_thrld
			    && dead_reckoning) {
				/* EVENT
				* @description
				* Switch to manual mode recommended.
				*
				* <profile name="dev">
				* This warning is triggered when the position error estimate is 90% of (or only 10m below) <param>COM_POS_FS_EPH</param> parameter.
				* </profile>
				*/
				events::send(events::ID("check_estimator_position_failure_imminent"), {events::Log::Error, events::LogInternal::Info},
					     "Estimated position error is approaching the failsafe threshold");

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(),
							     "Estimated position error is approaching the failsafe threshold\t");
				}

				_nav_failure_imminent_warned = true;

			} else if (!dead_reckoning) {
				_nav_failure_imminent_warned = false;
			}
		}
	}

	failsafe_flags.local_position_invalid =
		!checkPosVelValidity(now, xy_valid, lpos.eph, lpos_eph_threshold, lpos.timestamp,
				     _last_lpos_fail_time_us, !failsafe_flags.local_position_invalid);


	// In some modes we assume that the operator will compensate for the drift so we do not need to check the position error
	const float lpos_eph_threshold_relaxed = INFINITY;

	failsafe_flags.local_position_invalid_relaxed =
		!checkPosVelValidity(now, xy_valid, lpos.eph, lpos_eph_threshold_relaxed, lpos.timestamp,
				     _last_lpos_relaxed_fail_time_us, !failsafe_flags.local_position_invalid_relaxed);

	failsafe_flags.local_velocity_invalid =
		!checkPosVelValidity(now, v_xy_valid, lpos.evh, _param_com_vel_fs_evh.get(), lpos.timestamp,
				     _last_lvel_fail_time_us, !failsafe_flags.local_velocity_invalid);


	// altitude
	failsafe_flags.local_altitude_invalid = !lpos.z_valid || (now > lpos.timestamp + (_param_com_pos_fs_delay.get() * 1_s));


	// attitude
	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.copy(&attitude)) {
		const matrix::Quatf q{attitude.q};
		const float eps = 1e-5f;
		const bool no_element_larger_than_one = (fabsf(q(0)) <= 1.f + eps)
							&& (fabsf(q(1)) <= 1.f + eps)
							&& (fabsf(q(2)) <= 1.f + eps)
							&& (fabsf(q(3)) <= 1.f + eps);
		const bool norm_in_tolerance = fabsf(1.f - q.norm()) <= eps;

		failsafe_flags.attitude_invalid = (now > attitude.timestamp + 1_s) || !norm_in_tolerance
						  || !no_element_larger_than_one;

	} else {
		failsafe_flags.attitude_invalid = true;
	}

	// angular velocity
	vehicle_angular_velocity_s angular_velocity{};
	_vehicle_angular_velocity_sub.copy(&angular_velocity);
	const bool condition_angular_velocity_time_valid = angular_velocity.timestamp != 0
			&& (now < angular_velocity.timestamp + 1_s);
	const bool condition_angular_velocity_finite = matrix::Vector3f(angular_velocity.xyz).isAllFinite();
	const bool angular_velocity_invalid = !condition_angular_velocity_time_valid
					      || !condition_angular_velocity_finite;

	if (!failsafe_flags.angular_velocity_invalid && angular_velocity_invalid) {
		const char err_str[] {"angular velocity no longer valid"};

		if (!condition_angular_velocity_time_valid && angular_velocity.timestamp != 0) {
			PX4_ERR("%s (timeout)", err_str);

		} else if (!condition_angular_velocity_finite) {
			PX4_ERR("%s (non-finite values)", err_str);
		}
	}

	failsafe_flags.angular_velocity_invalid = angular_velocity_invalid;
}

bool EstimatorChecks::checkPosVelValidity(const hrt_abstime &now, const bool data_valid, const float data_accuracy,
		const float required_accuracy,
		const hrt_abstime &data_timestamp_us, hrt_abstime &last_fail_time_us,
		const bool was_valid) const
{
	bool valid = was_valid;
	const bool data_stale = (now > data_timestamp_us + _param_com_pos_fs_delay.get() * 1_s) || (data_timestamp_us == 0);
	const float req_accuracy = (was_valid ? required_accuracy * 2.5f : required_accuracy);
	const bool level_check_pass = data_valid && !data_stale && (data_accuracy < req_accuracy);

	// Check accuracy with hysteresis in both test level and time
	if (level_check_pass) {
		if (!was_valid) {
			// check if probation period has elapsed
			if (now > last_fail_time_us + 1_s) {
				valid = true;
			}
		}

	} else {
		// level check failed
		if (was_valid) {
			// FAILURE! no longer valid
			valid = false;
		}

		last_fail_time_us = now;
	}

	return valid;
}
