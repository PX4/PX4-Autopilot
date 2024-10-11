/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;
	_state_reset_count_prev = _state_reset_status.reset_count;

	if (_system_flag_buffer) {
		systemFlagUpdate system_flags_delayed;

		if (_system_flag_buffer->pop_first_older_than(imu_delayed.time_us, &system_flags_delayed)) {

			set_vehicle_at_rest(system_flags_delayed.at_rest);
			set_in_air_status(system_flags_delayed.in_air);

			set_is_fixed_wing(system_flags_delayed.is_fixed_wing);

			if (system_flags_delayed.gnd_effect) {
				set_gnd_effect();
			}

			set_constant_pos(system_flags_delayed.constant_pos);
		}
	}

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		// Once the tilt variances have reduced to equivalent of 3 deg uncertainty
		// and declare the tilt alignment complete
		if (getTiltVariance() < sq(math::radians(3.f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = "unknown";

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";
			}

			ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
				 (unsigned long long)imu_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);

			ECL_DEBUG("tilt aligned, roll: %.3f, pitch %.3f, yaw: %.3f",
				  (double)matrix::Eulerf(_state.quat_nominal).phi(),
				  (double)matrix::Eulerf(_state.quat_nominal).theta(),
				  (double)matrix::Eulerf(_state.quat_nominal).psi()
				 );
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// control use of observations for aiding
	controlMagFusion(imu_delayed);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	controlOpticalFlowFusion(imu_delayed);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
	controlGpsFusion(imu_delayed);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.update(*this, imu_delayed);
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

#if defined(CONFIG_EKF2_AIRSPEED)
	controlAirDataFusion(imu_delayed);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	controlBetaFusion(imu_delayed);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	controlDragFusion(imu_delayed);
#endif // CONFIG_EKF2_DRAG_FUSION

	controlHeightFusion(imu_delayed);

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	controlGravityFusion(imu_delayed);
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// Additional data odometry data from an external estimator can be fused.
	controlExternalVisionFusion(imu_delayed);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion(imu_delayed);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_TERRAIN)
	controlTerrainFakeFusion();
	updateTerrainValidity();
#endif // CONFIG_EKF2_TERRAIN

	controlZeroInnovationHeadingUpdate();

	_zero_velocity_update.update(*this, imu_delayed);

	if (_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias)) {
		_zero_gyro_update.update(*this, imu_delayed);
	}

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();
	controlFakeHgtFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	updateDeadReckoningStatus();
}
