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

	// Pop rangefinder measurement from buffer of samples into active sample
	sensor::rangeSample sample = {};
	if (!_range_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {
		if (_range_sensor.timedOut(imu_sample.time_us)) {
			// Disable fusion if it's currently enabled
			if (_control_status.flags.rng_hgt || _control_status.flags.rng_terrain) {
				ECL_INFO("stopping RNG fusion, sensor timed out");
				stopRangeAltitudeFusion();
				stopRangeTerrainFusion();
			}
		}
		return;
	}

	// TODO: move setting params to init function
	// Set all of the parameters
	_range_sensor.setPitchOffset(_params.ekf2_rng_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_rng_consistency_check.setGate(_params.range_kin_consistency_gate);

	// Update sensor to earth rotation
	_range_sensor.updateSensorToEarthRotation(_R_to_earth);

	// Gate sample consumption on these checks
	bool quality_ok = sample.quality > 0;
	bool tilt_ok = _range_sensor.isTiltOk();
	bool range_ok = sample.rng <= _range_sensor.getValidMaxVal() && sample.rng >= _range_sensor.getValidMinVal();
	// - Not stuck value
	// - Not fog detected

	// If quality, tilt, or value are outside of bounds -- throw away measurement
	if (!quality_ok || !tilt_ok || !range_ok) {
		if (_range_sensor.timedOut(imu_sample.time_us)) {
			// Disable fusion if it's currently enabled
			if (_control_status.flags.rng_hgt || _control_status.flags.rng_terrain) {
				ECL_INFO("stopping RNG fusion, sensor data invalid");
				stopRangeAltitudeFusion();
				stopRangeTerrainFusion();
			}
		}
		return;
	}

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

	// Track kinematic consistency
	// _control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();
	_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	// Publish EstimatorAidSource1d (observation, variance, rejected, fused)
	updateRangeHagl(_aid_src_rng_hgt);

	if (!PX4_ISFINITE(_aid_src_rng_hgt.observation) || !PX4_ISFINITE(_aid_src_rng_hgt.observation_variance)) {
		ECL_INFO("INFINIFY OBSERVATION INVALID");
	}

	if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::RANGE)) {
		fuseRangeAsHeightSource();
	} else {
		fuseRangeAsHeightAiding();
	}
}

void Ekf::fuseRangeAsHeightSource()
{
	// When primary height source is RANGE, we always fuse it
	// I don't think conditional range aid mode makes sense in the context of RANGE = primary

	// Fusion init logic
	if (_height_sensor_ref != HeightSensor::RANGE) {

		_height_sensor_ref = HeightSensor::RANGE;
		_information_events.flags.reset_hgt_to_rng = true;

		// Reset aid source innovation
		resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);

		// Reset altitude to RANGE
		resetAltitudeTo(_aid_src_rng_hgt.observation, _aid_src_rng_hgt.observation_variance);
		_control_status.flags.rng_hgt = true;

		// Cannot have terrain estimate fusion while RANGE is primary
		stopRangeTerrainFusion();
		_state.terrain = 0.f;

		// TODO: needed? It's set above in --> resetAidSourceStatusZeroInnovation()
		// _aid_src_rng_hgt.time_last_fuse = imu_sample.time_us;
	}


}

void Ekf::fuseRangeAsHeightAiding()
{
	bool range_aid_conditional = _params.rng_ctrl == RngCtrl::CONDITIONAL;
	bool range_aid_enabled = _params.rng_ctrl == RngCtrl::ENABLED;

	bool range_aid_conditions_passed = rangeAidConditionsPassed();
	bool kinematically_consistent = _control_status.flags.rng_kin_consistent;

	bool do_range_aid = kinematically_consistent &&
						(range_aid_enabled || (range_aid_conditional && range_aid_conditions_passed));

	bool fuse_measurement = false;


	// Variables to use below
	bool innovation_rejected = _aid_src_rng_hgt.innovation_rejected;
	bool optical_flow_for_terrain = _control_status.flags.opt_flow_terrain;

	// Fuse Range into Altitude if:
	// - passes range_aid_conditionalchecks
	// - kinematically consistent
	if (do_range_aid) {

		// Start fusion
		if (!_control_status.flags.rng_hgt) {
			// Fusion init logic
			_control_status.flags.rng_hgt = true;

			// TODO: review for correctness
			if (innovation_rejected && kinematically_consistent) {
				// Reset aid source
				ECL_INFO("range alt fusion, resetting aid source");
				resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);
			}
		}

		// Fuse
		fuse_measurement = true;

	} else {
		// Stop fusion
		ECL_INFO("stopping RNG Altitude fusion:");
		if (!range_aid_conditions_passed) {
			ECL_INFO("range aid conditions failed");
		}
		if (!kinematically_consistent) {
			ECL_INFO("kinematically inconsistent");
		}
		stopRangeAltitudeFusion();
	}

	// Fuse Range into Terrain if:
	// - kinematically consistent
	if (kinematically_consistent) {

		// Start fusion
		if (!_control_status.flags.rng_terrain) {
			// Fusion init logic
			_control_status.flags.rng_terrain = true;

			if (!optical_flow_for_terrain && innovation_rejected
				&& kinematically_consistent) {
				// Reset aid source and then reset terrain estimate
				ECL_INFO("range terrain fusion, resetting terrain to range");
				resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);
				resetTerrainToRng(_aid_src_rng_hgt);
			}
		}

		// Fuse
		fuse_measurement = true;

	} else {
		// Stop fusion
		ECL_INFO("stopping RNG Terrain fusion, kinematically inconsistent");
		stopRangeTerrainFusion();
	}

	if (fuse_measurement) {
		fuseHaglRng(_aid_src_rng_hgt, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
	}
}

bool Ekf::rangeAidConditionsPassed()
{
	// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
	// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
	float range_hagl_max = _params.max_hagl_for_range_aid;
	float max_vel_xy = _params.max_vel_for_range_aid;

	const float hagl_test_ratio = _aid_src_rng_hgt.test_ratio;

	bool is_hagl_stable = (hagl_test_ratio < 1.f);

	if (!_control_status.flags.rng_hgt) {
		range_hagl_max = 0.7f * _params.max_hagl_for_range_aid;
		max_vel_xy = 0.7f * _params.max_vel_for_range_aid;
		is_hagl_stable = (hagl_test_ratio < 0.01f);
	}

	const bool is_in_range = (getHagl() < range_hagl_max);

	bool is_below_max_speed = true;

	if (isHorizontalAidingActive()) {
		is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
	}

	return is_in_range && is_hagl_stable && is_below_max_speed;
}

void Ekf::resetTerrainToRng(estimator_aid_source1d_s &aid_src)
{
	// Since the distance is not a direct observation of the terrain state but is based
	// on the height state, a reset should consider the height uncertainty. This can be
	// done by manipulating the Kalman gain to inject all the innovation in the terrain state
	// and create the correct correlation with the terrain state with a covariance update.
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 0.f);

	const float old_terrain = _state.terrain;

	VectorState H;
	sym::ComputeHaglH(&H);

	VectorState K;
	K(State::terrain.idx) = 1.f; // innovation is forced into the terrain state to create a "reset"

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	// record the state change
	const float delta_terrain = _state.terrain - old_terrain;

	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_terrain;
	}

	_state_reset_status.reset_count.hagl++;

	aid_src.time_last_fuse = _time_delayed_us;
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

void Ekf::stopRangeAltitudeFusion()
{
	_control_status.flags.rng_hgt = false;
	if (_height_sensor_ref == HeightSensor::RANGE) {
		_height_sensor_ref = HeightSensor::UNKNOWN;
	}
}

void Ekf::stopRangeTerrainFusion()
{
	_control_status.flags.rng_terrain = false;
}
