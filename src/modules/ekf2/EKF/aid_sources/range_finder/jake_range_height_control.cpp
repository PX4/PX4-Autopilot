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
		// ECL_INFO("no buff");
		return;
	}

	// Pop rangefinder measurement from buffer of samples into active sample
	sensor::rangeSample sample = {};
	if (!_range_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {
		if (_range_sensor.timedOut(imu_sample.time_us)) {
			// Disable fusion if it's currently enabled
			if (_control_status.flags.rng_hgt || _control_status.flags.rng_terrain) {
				stopRangeAltitudeFusion("sensor timed out");
				stopRangeTerrainFusion("sensor timed out");
			}
			// ECL_INFO("timed out1");
		}
		return;
	}

	// Set the raw sample
	_range_sensor.setSample(sample);

	// TODO: move setting params to init function
	// Set all of the parameters
	_range_sensor.setPitchOffset(_params.ekf2_rng_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_rng_consistency_check.setGate(_params.range_kin_consistency_gate);

	// Update sensor to earth rotation
	_range_sensor.updateSensorToEarthRotation(_R_to_earth);


	// TODO: refactor into validity_checks()
	// Gate sample consumption on these checks
	bool quality_ok = sample.quality > 0; // TODO: what about unknown? (-1)
	bool tilt_ok = _range_sensor.isTiltOk();
	bool range_ok = sample.rng <= _range_sensor.getValidMaxVal() && sample.rng >= _range_sensor.getValidMinVal();
	// - Not stuck value
	// - Not fog detected

	// If quality, tilt, or value are outside of bounds -- throw away measurement
	if (!quality_ok || !tilt_ok || !range_ok) {
		if (_range_sensor.timedOut(imu_sample.time_us)) {
			// Disable fusion if it's currently enabled
			if (_control_status.flags.rng_hgt || _control_status.flags.rng_terrain) {
				const char* reason = "sensor data invalid";
				stopRangeAltitudeFusion(reason);
				stopRangeTerrainFusion(reason);
			}
			ECL_INFO("timed out2");
		}

		if (!quality_ok) {
			ECL_INFO("!quality_ok");
		}
		if (!tilt_ok) {
			ECL_INFO("!tilt_ok");
		}
		if (!range_ok) {
			ECL_INFO("!range_ok");
		} // commander takeoff
		return;
	}


	// TODO: refactor into apply_body_offset()
	// Correct the range data for position offset relative to the IMU
	const Vector3f rng_pos_body = { _params.ekf2_rng_pos_x, _params.ekf2_rng_pos_y, _params.ekf2_rng_pos_z };
	const Vector3f imu_pos_body = _params.imu_pos_body;
	const Vector3f pos_offset_body = rng_pos_body - imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
	sample.rng = sample.rng + pos_offset_earth(2) / _range_sensor.getCosTilt();

	// Provide sample from buffer to object
	_range_sensor.setSample(sample);


	// TODO: refactor into consintency_filter_update() that runs consistency status
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
	_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	// Publish EstimatorAidSource1d (observation, variance, rejected, fused)
	updateRangeHagl(_aid_src_rng_hgt);

	if (!PX4_ISFINITE(_aid_src_rng_hgt.observation) || !PX4_ISFINITE(_aid_src_rng_hgt.observation_variance)) {
		ECL_INFO("INFINIFY OBSERVATION INVALID");
	}

	// Fuse Range data as Primary height source
	// NOTE: terrain is not estimated in this mode
	if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::RANGE)) {
		fuseRangeAsHeightSource();
	} else {
		// Fuse Range data as aiding height source (Primary GPS or Baro)
		fuseRangeAsHeightAiding();
	}
}

void Ekf::fuseRangeAsHeightAiding()
{
	// ISSUES: 6/25/2025
	// - optical flow terrain fucks everything up, I've hardcoded disabled it
	// - when rng fusion starts again, EKF Z state is corrupted (oscillation)
	// -

	const char* kNotKinematicallyConsistentText = "not kinematically consistent";
	const char* kConditionsFailingText = "conditions failing";

	// Stop fusion if rangefinder kinematic consistency fails
	if (!_control_status.flags.rng_kin_consistent) {
		stopRangeTerrainFusion(kNotKinematicallyConsistentText);
		stopRangeAltitudeFusion(kNotKinematicallyConsistentText);
		return;
	}


	//// TERRAIN FUSION ////

	// Fuse Range into Terrain if:
	// - kinematically consistent (hagl_rate < 1)

	// Start fusion
	if (!_control_status.flags.rng_terrain) {
		ECL_INFO("START RNG Terrain fusion");
		_control_status.flags.rng_terrain = true;

		// We must reset terrain to range before we start fusing again, otherwise there
		// can be a delta between RNG terrain estimate and previous, which will cause a
		// discontinuity in the lpos.z state
		ECL_INFO("resetting terrain to range");
		resetTerrainToRng(_aid_src_rng_hgt);
		resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);
	}


	//// ALTITUDE FUSION ////

	bool range_aid_conditional = _params.rng_ctrl == RngCtrl::CONDITIONAL;
	bool range_aid_enabled = _params.rng_ctrl == RngCtrl::ENABLED;

	bool range_aid_conditions_passed = rangeAidConditionsPassed();

	bool do_range_aid = range_aid_enabled || (range_aid_conditional && range_aid_conditions_passed);

	// Fuse Range into Altitude if:
	// - passes range_aid_conditionalchecks
	// - kinematically consistent
	if (do_range_aid) {

		if (!_control_status.flags.rng_hgt) {
			ECL_INFO("START RNG Altitude fusion");
			_control_status.flags.rng_hgt = true;

			// Reset altitude to rangefinder if on ground
			if (!_control_status.flags.in_air) {
				ECL_INFO("JAKE GND resetting altitude to range");
				resetAltitudeTo(_aid_src_rng_hgt.observation - _state.terrain);
			}

			// TODO: review for correctness
			if (_aid_src_rng_hgt.innovation_rejected) {
				// Reset aid source
				ECL_INFO("resetting aid source");
				resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);
			}
		}

	} else {
		stopRangeAltitudeFusion(kConditionsFailingText);
	}

	// If we make it here, fuse
	fuseHaglRng(_aid_src_rng_hgt, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
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
		stopRangeTerrainFusion("Cannot have terrain estimate fusion while RANGE is primary");
		ECL_INFO("initializing range as primary");
		_state.terrain = 0.f;

		// TODO: needed? It's set above in --> resetAidSourceStatusZeroInnovation()
		// _aid_src_rng_hgt.time_last_fuse = imu_sample.time_us;
	}

	// TODO:
	// When RNG is primary height source
	if (isHeightResetRequired() && _control_status.flags.rng_hgt && (_height_sensor_ref == HeightSensor::RANGE)) {
		ECL_INFO("RNG height fusion reset required, all height sources failing");

		uint64_t timestamp = _aid_src_rng_hgt.timestamp;
		_information_events.flags.reset_hgt_to_rng = true;
		resetAltitudeTo(_aid_src_rng_hgt.observation - _state.terrain);
		resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);

		// reset vertical velocity if no valid sources available
		if (!isVerticalVelocityAidingActive()) {
			ECL_INFO("resetting vertical velocity");
			resetVerticalVelocityToZero();
		}

		_aid_src_rng_hgt.time_last_fuse = timestamp;
	}
}

bool Ekf::rangeAidConditionsPassed()
{
	bool is_in_range = getHagl() < _params.ekf2_rng_a_hmax;
	// bool is_hagl_stable = _aid_src_rng_hgt.test_ratio < 1.f;
	// bool is_horizontal_aiding_active = isHorizontalAidingActive();
	// bool is_below_max_speed = is_horizontal_aiding_active && !_state.vel.xy().longerThan(_params.ekf2_rng_a_vmax);
	bool is_below_max_speed = !_state.vel.xy().longerThan(_params.ekf2_rng_a_vmax);

	// Require conditions passing for 1_s (same as kinematic consistency check)
	bool conditions_passing = false;
	// bool conditions_met = is_in_range && is_hagl_stable && is_below_max_speed;
	bool conditions_met = is_in_range && is_below_max_speed;

	if (conditions_met) {
		if (!_rng_aid_conditions_valid) {
			// Conditions just became valid
			ECL_INFO("RNG AID conditions valid");
			_rng_aid_conditions_valid = true;
			_time_rng_aid_conditions_valid = _time_latest_us;
		}

		if (_time_latest_us > _time_rng_aid_conditions_valid + 1'000'000) {
			// Conditions have been valid for at least 1s
			conditions_passing = true;
		}
	} else {

		if (_rng_aid_conditions_valid) {
			_rng_aid_conditions_valid = false;

			// if (!is_hagl_stable) {
			// 	ECL_INFO("!is_hagl_stable");
			// }
			// if (is_horizontal_aiding_active && !is_below_max_speed) {
			if (!is_below_max_speed) {
				ECL_INFO("!is_below_max_speed");
			}
			if (!is_in_range) {
				ECL_INFO("!is_in_range");
			}
		}
	}

	return conditions_passing;
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
	const float measurement = math::max(_range_sensor.getDistBottom(), _params.ekf2_min_rng);
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

void Ekf::stopRangeAltitudeFusion(const char* reason)
{
	if (_control_status.flags.rng_hgt) {
		ECL_INFO("STOP RNG Altitude fusion: %s", reason);
		_control_status.flags.rng_hgt = false;
		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}
	}
}

void Ekf::stopRangeTerrainFusion(const char* reason)
{
	if (_control_status.flags.rng_terrain) {
		ECL_INFO("STOP RNG Terrain fusion: %s", reason);
		_control_status.flags.rng_terrain = false;
	}
}
