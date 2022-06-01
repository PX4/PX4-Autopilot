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

#include "estimatorCheck.hpp"

using namespace time_literals;

void EstimatorChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_sys_mc_est_group.get() != 2) {
		return;
	}

	const NavModes required_groups = (NavModes)reporter.failsafeFlags().mode_req_attitude;
	bool missing_data = false;

	if (_param_sens_imu_mode.get() == 0) { // multi-ekf
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			bool instance_changed = _estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance)
						&& _estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);

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

			checkEstimatorStatus(context, reporter, estimator_status, required_groups);

		} else {
			missing_data = true;
		}
	}

	if (missing_data) {
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
}

void EstimatorChecks::checkEstimatorStatus(const Context &context, Report &reporter,
		const estimator_status_s &estimator_status, NavModes required_groups)
{
	if (estimator_status.pre_flt_fail_innov_heading) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_heading_not_stable"),
					    events::Log::Error, "Heading estimate not stable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: heading estimate not stable");
		}

	} else if (estimator_status.pre_flt_fail_innov_vel_horiz) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_hor_vel_not_stable"),
					    events::Log::Error, "Horizontal velocity estimate not stable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: horizontal velocity estimate not stable");
		}

	} else if (estimator_status.pre_flt_fail_innov_vel_vert) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_vert_vel_not_stable"),
					    events::Log::Error, "Vertical velocity estimate not stable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: vertical velocity estimate not stable");
		}

	} else if (estimator_status.pre_flt_fail_innov_height) {
		/* EVENT
		 */
		reporter.armingCheckFailure(required_groups, health_component_t::local_position_estimate,
					    events::ID("check_estimator_hgt_not_stable"),
					    events::Log::Error, "Height estimate not stable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: height estimate not stable");
		}
	}


	if ((_param_com_arm_mag_str.get() >= 1) && estimator_status.pre_flt_fail_mag_field_disturbed) {
		NavModes required_groups_mag = required_groups;

		if (_param_com_arm_mag_str.get() != 1) {
			required_groups_mag = NavModes::None; // optional
		}

		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_MAG_STR</param> and <param>EKF2_MAG_CHECK</param> parameters.
		 * </profile>
		 */
		reporter.armingCheckFailure(required_groups_mag, health_component_t::local_position_estimate,
					    events::ID("check_estimator_mag_interference"),
					    events::Log::Warning, "Strong magnetic interference detected");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Strong magnetic interference detected");
		}
	}

	// check vertical position innovation test ratio
	if (estimator_status.hgt_test_ratio > _param_com_arm_ekf_hgt.get()) {
		/* EVENT
		 * <profile name="dev">
		 * Test ratio: {1:.3}, limit: {2:.3}.
		 *
		 * This check can be configured via <param>COM_ARM_EKF_HGT</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure<float, float>(required_groups, health_component_t::local_position_estimate,
				events::ID("check_estimator_hgt_est_err"),
				events::Log::Error, "Height estimate error", estimator_status.hgt_test_ratio, _param_com_arm_ekf_hgt.get());

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: height estimate error");
		}
	}

	// check velocity innovation test ratio
	if (estimator_status.vel_test_ratio > _param_com_arm_ekf_vel.get()) {
		/* EVENT
		 * <profile name="dev">
		 * Test ratio: {1:.3}, limit: {2:.3}.
		 *
		 * This check can be configured via <param>COM_ARM_EKF_VEL</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure<float, float>(required_groups, health_component_t::local_position_estimate,
				events::ID("check_estimator_vel_est_err"),
				events::Log::Error, "Velocity estimate error", estimator_status.vel_test_ratio, _param_com_arm_ekf_vel.get());

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: velocity estimate error");
		}
	}

	// check horizontal position innovation test ratio
	if (estimator_status.pos_test_ratio > _param_com_arm_ekf_pos.get()) {
		/* EVENT
		 * <profile name="dev">
		 * Test ratio: {1:.3}, limit: {2:.3}.
		 *
		 * This check can be configured via <param>COM_ARM_EKF_POS</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure<float, float>(required_groups, health_component_t::local_position_estimate,
				events::ID("check_estimator_pos_est_err"),
				events::Log::Error, "Position estimate error", estimator_status.pos_test_ratio, _param_com_arm_ekf_pos.get());

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: position estimate error");
		}
	}

	// check magnetometer innovation test ratio
	if (estimator_status.mag_test_ratio > _param_com_arm_ekf_yaw.get()) {
		/* EVENT
		 * <profile name="dev">
		 * Test ratio: {1:.3}, limit: {2:.3}.
		 *
		 * This check can be configured via <param>COM_ARM_EKF_YAW</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure<float, float>(required_groups, health_component_t::local_position_estimate,
				events::ID("check_estimator_yaw_est_err"),
				events::Log::Error, "Yaw estimate error", estimator_status.mag_test_ratio, _param_com_arm_ekf_yaw.get());

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Yaw estimate error");
		}
	}

	// If GPS aiding is required, declare fault condition if the required GPS quality checks are failing
	if (_param_sys_has_gps.get()) {
		const bool ekf_gps_fusion = estimator_status.control_mode_flags & (1 << estimator_status_s::CS_GPS);
		const bool ekf_gps_check_fail = estimator_status.gps_check_fail_flags > 0;

		if (ekf_gps_fusion) {
			reporter.setIsPresent(health_component_t::gps); // should be based on the sensor data directly
		}

		if (ekf_gps_check_fail) {
			NavModes required_groups_gps = required_groups;
			events::Log log_level = events::Log::Error;

			if (_param_com_arm_wo_gps.get()) {
				required_groups_gps = NavModes::None; // optional
				log_level = events::Log::Warning;
			}

			// Only report the first failure to avoid spamming
			const char *message = nullptr;

			if (estimator_status.gps_check_fail_flags & (1 << estimator_status_s::GPS_CHECK_FAIL_GPS_FIX)) {
				message = "Preflight%s: GPS fix too low";
				/* EVENT
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
				 * <profile name="dev">
				 * This check can be configured via <param>EKF2_GPS_CHECK</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure(required_groups_gps, health_component_t::gps,
							    events::ID("check_estimator_gps_vert_speed_drift_too_high"),
							    log_level, "GPS Vertical Speed Drift too high");

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
				if (!_param_com_arm_wo_gps.get()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), message, " Fail");

				} else {
					mavlink_log_critical(reporter.mavlink_log_pub(), message, "");
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
					 * An accelerometer recalibration might help.
					 *
					 * <profile name="dev">
					 * Axis {1}: |{2:.8}| > {3:.8} + {4:.8}
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
					 * A Gyro recalibration might help.
					 *
					 * <profile name="dev">
					 * Axis {1}: |{2:.8}| > {3:.8} + {4:.8}
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
