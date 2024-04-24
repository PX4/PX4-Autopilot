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

#include "imuConsistencyCheck.hpp"

void ImuConsistencyChecks::checkAndReport(const Context &context, Report &reporter)
{
	sensors_status_imu_s imu;

	if (!_sensors_status_imu_sub.copy(&imu) || context.isArmed()) {
		return;
	}

	// Use the difference between IMU's to detect a bad calibration.
	// If a single IMU is fitted, the value being checked will be zero so this check will always pass.
	for (unsigned i = 0; i < (sizeof(imu.accel_inconsistency_m_s_s) / sizeof(imu.accel_inconsistency_m_s_s[0])); i++) {
		if (imu.accel_device_ids[i] != 0) {

			const float accel_inconsistency_m_s_s = imu.accel_inconsistency_m_s_s[i];

			if (accel_inconsistency_m_s_s > _param_com_arm_imu_acc.get()) {
				/* EVENT
				 * @description
				 * Check the calibration.
				 *
				 * Inconsistency value: {2}.
				 * Configured Threshold: {3}.
				 *
				 * <profile name="dev">
				 * This check can be configured via <param>COM_ARM_IMU_ACC</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure<uint8_t, float, float>(NavModes::All, health_component_t::accel,
						events::ID("check_imu_accel_inconsistent"),
						events::Log::Warning, "Accel {1} inconsistent", i, accel_inconsistency_m_s_s, _param_com_arm_imu_acc.get());

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Accel %u inconsistent - check cal", i);
				}

				break;
			}
		}
	}

	// Fail if gyro difference greater than 5 deg/sec and notify if greater than 2.5 deg/sec
	for (unsigned i = 0; i < (sizeof(imu.gyro_inconsistency_rad_s) / sizeof(imu.gyro_inconsistency_rad_s[0])); i++) {
		if (imu.gyro_device_ids[i] != 0) {

			const float gyro_inconsistency_rad_s = imu.gyro_inconsistency_rad_s[i];

			if (gyro_inconsistency_rad_s > _param_com_arm_imu_gyr.get()) {
				/* EVENT
				 * @description
				 * Check the calibration.
				 *
				 * Inconsistency value: {2}.
				 * Configured Threshold: {3}.
				 *
				 * <profile name="dev">
				 * This check can be configured via <param>COM_ARM_IMU_GYR</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure<uint8_t, float, float>(NavModes::All, health_component_t::gyro,
						events::ID("check_imu_gyro_inconsistent"),
						events::Log::Warning, "Gyro {1} inconsistent", i, gyro_inconsistency_rad_s, _param_com_arm_imu_gyr.get());

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Gyro %u inconsistent - check cal", i);
				}

				break;
			}
		}
	}
}
