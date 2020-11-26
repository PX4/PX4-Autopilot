/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "Ekf2Checks.hpp"

bool Ekf2Checks::ekf2Enabled(const Context &context) const
{
	if (context.status().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !context.status().is_vtol) {
		return _param_sys_mc_est_group.get() == 2;
	}

	// EKF2 is currently the only supported option for FW & VTOL
	return true;
}

void Ekf2Checks::checkAndReport(const Context &context, Report &reporter)
{
	if (!ekf2Enabled(context)) {
		return;
	}

	estimator_status_s status;

	if (!_status_sub.copy(&status)) {
		/* EVENT
		 * @description
		 *  Wait for the estimator to initialize. This should only take a few seconds.
		 *  <profile name="dev">
		 *  If this persists, check your software configuration and make sure ekf2 is running with all required sensors.
		 *  </profile>
		 */
		reporter.armingCheckFailure(ModeCategory::All, HealthComponentIndex::attitude_estimate,
					    events::ID("health_ekf2_status_missing"),
					    "Estimator is initializing", events::Log::Info);
		return;
	}

	// mark the estimator as present (even if we don't have a valid estimate yet)
	reporter.setIsPresent(HealthComponentIndex::attitude_estimate);
	reporter.setIsPresent(HealthComponentIndex::local_position_estimate);
	reporter.setIsPresent(HealthComponentIndex::global_position_estimate);

	// Check if preflight check performed by estimator has failed
	if (status.pre_flt_fail_innov_heading) {
		/* EVENT
		 * @description
		 *  TODO
		 */
		reporter.armingCheckFailure(ModeCategory::All, HealthComponentIndex::attitude_estimate,
					    events::ID("health_ekf2_heading_est_unstable"),
					    "Heading estimate not stable", events::Log::Error);

	} else if (status.pre_flt_fail_innov_vel_horiz) {
		/* EVENT
		 * @description
		 *  TODO
		 */
		reporter.armingCheckFailure(ModeCategory::All, HealthComponentIndex::local_position_estimate,
					    events::ID("health_ekf2_hor_vel_est_unstable"),
					    "Horizontal velocity estimate not stable", events::Log::Error);

	} else if (status.pre_flt_fail_innov_vel_vert) {
		/* EVENT
		 * @description
		 *  TODO
		 */
		reporter.armingCheckFailure(ModeCategory::All, HealthComponentIndex::local_position_estimate,
					    events::ID("health_ekf2_vert_vel_est_unstable"),
					    "Vertical velocity estimate not stable", events::Log::Error);

	} else if (status.pre_flt_fail_innov_height) {
		/* EVENT
		 * @description
		 *  TODO
		 */
		reporter.armingCheckFailure(ModeCategory::All, HealthComponentIndex::local_position_estimate,
					    events::ID("health_ekf2_height_est_unstable"),
					    "Height estimate not stable", events::Log::Error);
	}

	if (_param_mag_strength_check_enabled.get() && status.pre_flt_fail_mag_field_disturbed) {
		/* EVENT
		 * @description
		 *  TODO
		 *
		 *  <profile name="dev">
		 *  This check can be disabled with the parameter <param>COM_ARM_MAG_STR</param>.
		 *  </profile>
		 */
		reporter.armingCheckFailure(ModeCategory::All, HealthComponentIndex::local_position_estimate,
					    events::ID("health_ekf2_mag_interference"),
					    "Strong magnetic interference detected", events::Log::Error);
	}

	// check vertical position innovation test ratio
	if (status.hgt_test_ratio > _param_hgt_test_ratio_limit.get()) {
		/* EVENT
		 * @description
		 *  TODO
		 *
		 *  <profile name="dev">
		 *  The threshold can be configured with the parameter <param>COM_ARM_EKF_HGT</param>.
		 *  Current value: {1:.2}
		 *  </profile>
		 * @arg1: hgt_test_ratio
		 */
		reporter.armingCheckFailure<float>(ModeCategory::All, HealthComponentIndex::local_position_estimate,
						   events::ID("health_ekf2_height_est_error"),
						   "Height estimate error", events::Log::Error, status.hgt_test_ratio);
	}

	// check velocity innovation test ratio
	if (status.vel_test_ratio > _param_vel_test_ratio_limit.get()) {
//		if (report_fail) {
//			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Velocity estimate error");
//		}
//
//		success = false;
//		goto out;
	}

	// check horizontal position innovation test ratio
	if (status.pos_test_ratio > _param_pos_test_ratio_limit.get()) {
//		if (report_fail) {
//			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Position estimate error");
//		}
//
//		success = false;
//		goto out;
	}

	// check magnetometer innovation test ratio
	if (status.mag_test_ratio > _param_mag_test_ratio_limit.get()) {
//		if (report_fail) {
//			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Yaw estimate error");
//		}
//
//		success = false;
//		goto out;
	}

	// GPS checks
	ModeCategory required_modes = ModeCategory::None;

	if (!_param_arm_without_gps.get()) {
		required_modes = ModeCategory::All;
	}

	const bool ekf_gps_fusion = status.control_mode_flags & (1 << estimator_status_s::CS_GPS);
	const bool ekf_gps_check_fail = status.gps_check_fail_flags > 0;

	if (ekf_gps_check_fail) {
		// Only report the first failure to avoid spamming
		if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_GPS_FIX)) {
			/* EVENT
			 * @description
			 *  TODO
			 *
			 */
			reporter.armingCheckFailure(required_modes, HealthComponentIndex::sensor_gps, events::ID("health_ekf2_gps_fix_too_low"),
						    "GPS fix too low", events::Log::Error);

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MIN_SAT_COUNT)) {
			/* EVENT
			 * @description
			 *  TODO
			 *
			 *  <profile name="dev">
			 *  This check can be configured with the parameters <param>EKF2_GPS_CHECK</param> and
			 *  <param>EKF2_REQ_NSATS</param>.
			 *  </profile>
			 */
			reporter.armingCheckFailure(required_modes, HealthComponentIndex::sensor_gps,
						    events::ID("health_ekf2_gps_sats_too_low"),
						    "Not enough GPS Satellites", events::Log::Warning);

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MIN_PDOP)) {
//			message = "Preflight%s: GPS PDOP too low";

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR)) {
//			message = "Preflight%s: GPS Horizontal Pos Error too high";

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR)) {
//			message = "Preflight%s: GPS Vertical Pos Error too high";

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR)) {
//			message = "Preflight%s: GPS Speed Accuracy too low";

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_DRIFT)) {
//			message = "Preflight%s: GPS Horizontal Pos Drift too high";

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_DRIFT)) {
//			message = "Preflight%s: GPS Vertical Pos Drift too high";

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR)) {
//			message = "Preflight%s: GPS Hor Speed Drift too high";

		} else if (status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR)) {
//			message = "Preflight%s: GPS Vert Speed Drift too high";

		} else {
			if (!ekf_gps_fusion) {
				// Likely cause unknown
//				message = "Preflight%s: Estimator not using GPS";
//				gps_present = false;

			} else {
				// if we land here there was a new flag added and the code not updated. Show a generic message.
//				message = "Preflight%s: Poor GPS Quality";
			}
		}
	}

	stateChecks(context, reporter);
}

void Ekf2Checks::stateChecks(const Context &context, Report &reporter)
{
	// Get estimator states data if available and exit with a fail recorded if not
	estimator_states_s states;

	if (!_states_sub.copy(&states)) {
		return;
	}

	// check accelerometer delta velocity bias estimates
	for (uint8_t index = 13; index < 16; index++) {
		// allow for higher uncertainty in estimates for axes that are less observable to prevent false positives
		// adjust test threshold by 3-sigma
		float test_uncertainty = 3.0f * sqrtf(fmaxf(states.covariances[index], 0.0f));

		if (fabsf(states.states[index]) > _param_ekf_ab_test_limit.get() + test_uncertainty) {
			/* EVENT
			 * @description
			 *  TODO
			 *
			 *  <profile name="dev">
			 *  The threshold can be configured with the parameter <param>COM_ARM_EKF_AB</param>.
			 *  State {1}: |{2:.6}| \> {3:.6} + {4:.6}
			 *  </profile>
			 * @arg1: state_index
			 * @arg2: accel_bias
			 * @arg3: test_limit
			 * @arg4: test_uncertainty
			 */
			reporter.armingCheckFailure<uint8_t, float, float, float>(ModeCategory::All, HealthComponentIndex::attitude_estimate,
					events::ID("health_ekf2_high_accel_bias"),
					"High accelerometer bias", events::Log::Error, index, states.states[index], _param_ekf_ab_test_limit.get(),
					test_uncertainty);
			// only report the first
			break;
		}
	}

	// check gyro delta angle bias estimates
	float max_gyro_bias = fmaxf(fmaxf(fabsf(states.states[10]), fabsf(states.states[11])), fabsf(states.states[12]));

	if (max_gyro_bias > _param_ekf_gb_test_limit.get()) {

		/* EVENT
		 * @description
		 *  TODO
		 *
		 *  <profile name="dev">
		 *  The threshold can be configured with the parameter <param>COM_ARM_EKF_GB</param>.
		 *  Current value: {1:.2}
		 *  </profile>
		 * @arg1: max_gyro_bias
		 */
		reporter.armingCheckFailure<float>(ModeCategory::All, HealthComponentIndex::attitude_estimate,
						   events::ID("health_ekf2_high_gyro_bias"),
						   "High gyro bias", events::Log::Error, max_gyro_bias);
	}
}

