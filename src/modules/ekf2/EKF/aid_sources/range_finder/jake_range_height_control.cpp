/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_h.h"
#include "ekf_derivation/generated/compute_hagl_innov_var.h"

void Ekf::controlRangeHaglFusion(const imuSample &imu_sample)
{
	// Check if rangefinder is available/enabled
	if (!_range_buffer) {
		return;
	}

	bool sample_valid = false;
	bool kinematically_consistent = false;

	// Pop rangefinder measurement from buffer of samples into active sample
	sensor::rangeSample sample = {};
	bool sample_ready = _range_buffer->pop_first_older_than(imu_sample.time_us, &sample);
	if (sample_ready) {
		// Set all of the parameters
		_range_sensor.setPitchOffset(_params.ekf2_rng_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_rng_consistency_check.setGate(_params.range_kin_consistency_gate);

		// Update sensor to earth rotation
		_range_sensor.updateSensorToEarthRotation(_R_to_earth);

		// Gate sample consumption on these checks
		// - Quality is OK
		bool quality_ok = sample.quality > 0;
		// - Tilt is OK
		bool tilt_ok = _range_sensor.isTiltOk();
		// - Value is IN range
		bool range_ok = sample.rng <= _range_sensor.getValidMaxVal() && sample.rng >= _range_sensor.getValidMinVal();
		// - Not stuck value
		// - Not fog detected

		if (quality_ok && tilt_ok && range_ok) {

			// Correct the range data for position offset relative to the IMU
			const Vector3f rng_pos_body = { _params.ekf2_rng_pos_x, _params.ekf2_rng_pos_y, _params.ekf2_rng_pos_z };
			const Vector3f imu_pos_body = _params.imu_pos_body;
			const Vector3f pos_offset_body = rng_pos_body - imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			sample.rng = sample.rng + pos_offset_earth(2) / _range_sensor.getCosTilt();

			// Provide sample from buffer to object
			_range_sensor.setSample(sample);

			// Check kinematic consistency of rangefinder measurement w.r.t Altitude Estimate
			_rng_consistency_check.current_posD_reset_count = get_posD_reset_count();

			const float z = _gpos.altitude();
			const float vz = _state.vel(2);
			const float dist = _range_sensor.getDistBottom(); // NOTE: applies rotation into world frame
			const float dist_var = 0.05;
			const float z_var = P(State::pos.idx + 2, State::pos.idx + 2);
			const float vz_var = P(State::vel.idx + 2, State::vel.idx + 2);

			// Run the kinematic consistency check
			_rng_consistency_check.run(z, z_var, vz, vz_var, dist, dist_var, imu_sample.time_us);

			kinematically_consistent = _rng_consistency_check.isKinematicallyConsistent();

			if (kinematically_consistent) {
				sample_valid = true;
			}

			// Publish EstimatorAidSource1d (observation, variance, rejected, fused)
			updateRangeHagl(_aid_src_rng_hgt);
		}
	}

	// Check if rangefinder has timed out
	if (_range_sensor.timedOut(imu_sample.time_us)) {
		// TODO: timeout value?
		// TODO: handle timeout
	}

	// If the sample is valid conditionally fuse into Altitude and Terrain estimates
	if (sample_valid) {

	} else {

		// Sample is invalid
		// - Timeout
		// - Quality
		// - Tilt
		// - Range
		// - KinematicConsistency
		// - RangeAidChecks...
		//   - Horizontal Velocity
		//   - aid_src_rng_hgt.test_ratio

		// If sample is invalid
		// - Don't fuse
		// -

		return;
	}


	// Conditionally enable/disable rangefinder fusion for Altitude Estimate and Terrain Estimate
}

void Ekf::updateRangeHagl(estimator_aid_source1d_s &aid_src)
{
	const float measurement = math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance);
	const float measurement_variance = getRngVar();

	float innovation_variance;
	sym::ComputeHaglInnovVar(P, measurement_variance, &innovation_variance);

	const float innov_gate = math::max(_params.range_innov_gate, 1.f);
	updateAidSourceStatus(aid_src,
			      _range_sensor.sample()->time_us, // sample timestamp
			      measurement,                               // observation
			      measurement_variance,                      // observation variance
			      getHagl() - measurement,                   // innovation
			      innovation_variance,                       // innovation variance
			      innov_gate);                               // innovation gate

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

float Ekf::getRngVar() const
{
	const float dist_dependant_var = sq(_params.ekf2_rng_sfe * _range_sensor.getDistBottom());
	const float dist_var = sq(_params.range_noise) + dist_dependant_var;
	return dist_var;
}
