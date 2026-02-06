/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file ranging_beacon_control.cpp
 * Control functions for EKF ranging beacon fusion
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_range_beacon_innov_var_and_h.h>
#include <mathlib/mathlib.h>

void Ekf::controlRangingBeaconFusion(const imuSample &imu_delayed)
{
	if (!_ranging_beacon_buffer || (_params.ekf2_rngbc_ctrl == 0)) {
		stopRangingBeaconFusion();
		return;
	}

	rangingBeaconSample ranging_beacon_sample_delayed{};
	const bool ranging_beacon_data_ready = _ranging_beacon_buffer->pop_first_older_than(
			imu_delayed.time_us, &ranging_beacon_sample_delayed);

	if (ranging_beacon_data_ready) {

		const rangingBeaconSample &sample =  ranging_beacon_sample_delayed;

		// Basic validity check
		const bool measurement_valid = PX4_ISFINITE(sample.range_m)
					       && (sample.range_m > 0.f)
					       && PX4_ISFINITE(sample.range_var)
					       && PX4_ISFINITE(sample.beacon_lat)
					       && PX4_ISFINITE(sample.beacon_lon)
					       && PX4_ISFINITE(sample.beacon_alt);

		const bool common_conditions_passing = measurement_valid
						       && _local_origin_lat_lon.isInitialized();

		if (common_conditions_passing) {
			printf("fusing\n");
			fuseRangingBeacon(sample);

		} else {
			stopRangingBeaconFusion();
		}

	} else {
		stopRangingBeaconFusion();
	}
}

void Ekf::fuseRangingBeacon(const rangingBeaconSample &sample)
{
	const matrix::Vector3d vehicle_ecef = _gpos.toEcef();

	const LatLonAlt beacon_lla(sample.beacon_lat, sample.beacon_lon, sample.beacon_alt);
	const matrix::Vector3d beacon_ecef = beacon_lla.toEcef();

	const matrix::Vector3d delta_ecef = beacon_ecef - vehicle_ecef;
	const double predicted_range = delta_ecef.norm();

	const float innovation = static_cast<float>(predicted_range) - sample.range_m;
	const float R = fmaxf(sample.range_var, sq(_params.ekf2_rngbc_noise));

	// Compute beacon position relative to the vehicle in local NED.
	// _state.pos(0:1) is always 0 in the global-position EKF, so we must pass
	// the relative position directly so the symforce function gets the correct
	// vehicle-to-beacon direction for the H matrix.
	const Vector2f beacon_pos_ne = _local_origin_lat_lon.project(sample.beacon_lat, sample.beacon_lon);
	const Vector2f vehicle_pos_ne = getLocalHorizontalPosition();
	const float beacon_D = -sample.beacon_alt + _local_origin_alt;
	const Vector3f beacon_pos_rel(beacon_pos_ne(0) - vehicle_pos_ne(0),
				      beacon_pos_ne(1) - vehicle_pos_ne(1),
				      beacon_D);

	float innov_var;
	VectorState H;

	sym::ComputeRangeBeaconInnovVarAndH(_state.vector(), P, beacon_pos_rel, R, FLT_EPSILON, &innov_var, &H);

	updateAidSourceStatus(_aid_src_ranging_beacon,
			      sample.time_us,
			      sample.range_m,
			      R,
			      innovation,
			      innov_var,
			      _params.ekf2_rngbc_gate);

	_aid_src_ranging_beacon.device_id = sample.beacon_id;

	if (_aid_src_ranging_beacon.innovation_rejected) {

		printf("innovation rejected\n");
		return;
	}

	VectorState K = P * H / innov_var;
	clearInhibitedStateKalmanGains(K);
	measurementUpdate(K, H, R, innovation);

	_aid_src_ranging_beacon.fused = true;
	_aid_src_ranging_beacon.time_last_fuse = _time_delayed_us;
	_time_last_hor_pos_fuse = _time_delayed_us;

	if (!_control_status.flags.rngbcn_fusion) {
		ECL_INFO("starting ranging beacon fusion");
		_control_status.flags.rngbcn_fusion = true;
		printf("lets go\n");
	}

}

void Ekf::stopRangingBeaconFusion()
{
	if (_control_status.flags.rngbcn_fusion) {
		ECL_INFO("stopping ranging beacon fusion");
		_control_status.flags.rngbcn_fusion = false;
	}
}
