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
	if (!_range_buffer) {
		return;
	}

	// TODO: why isn't this being done anywhere?
	_aid_src_rng_hgt.fused = false;

	// Pop rangefinder measurement from buffer of samples into active sample
	rangeSample sample = {};

	if (!_range_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {
		// no new sample available, check if we've timed out
		bool had_sample = _range_time_last_good_sample > 0;
		bool timed_out = imu_sample.time_us > _range_time_last_good_sample + 200'000;

		if (had_sample && timed_out) {
			stopRangeAltitudeFusion("sensor timed out");
			stopRangeTerrainFusion("sensor timed out");
		}

		return;
	}

	// Quality checks
	if (sample.quality == 0) {
		if (_control_status.flags.in_air) {
			// Disable fusion after 1s of bad quality
			uint64_t elapsed = sample.time_us - _range_time_last_good_sample;

			if (elapsed > 1'000'000) {
				stopRangeAltitudeFusion("sensor quality bad");
				stopRangeTerrainFusion("sensor quality bad");
			}

			return;

		} else {
			// While on ground fake a measurement at ground level if the signal quality is bad
			sample.quality = 100;
			sample.range = sample.min_distance;
			_range_time_last_good_sample = sample.time_us;
		}

	} else {
		_range_time_last_good_sample = sample.time_us;
	}

	float cos_theta = _R_to_earth(2, 2);
	bool tilt_ok = cos_theta > _params.range_cos_max_tilt;
	bool range_ok = sample.range <= sample.max_distance && sample.range >= sample.min_distance;

	// If tilt or value are outside of bounds -- throw away measurement
	if (!tilt_ok || !range_ok) {
		stopRangeAltitudeFusion("pre checks failed");
		stopRangeTerrainFusion("pre checks failed");
		return;
	}

	// TODO: refactor into apply_body_offset()
	// Correct the range data for position offset relative to the IMU
	const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;

	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	sample.range = sample.range + pos_offset_earth(2) / cos_theta; // rotate into earth frame

	// TODO: refactor into consintency_filter_update() that runs consistency status
	// Check kinematic consistency of rangefinder measurement w.r.t Altitude Estimate
	_rng_consistency_check.current_posD_reset_count = get_posD_reset_count();

	// rotate into world frame
	float sample_corrected = sample.range * cos_theta;

	const float z = _gpos.altitude();
	const float vz = _state.vel(2);
	const float dist = sample_corrected;

	const float dist_var = 0.05;
	const float z_var = P(State::pos.idx + 2, State::pos.idx + 2);
	const float vz_var = P(State::vel.idx + 2, State::vel.idx + 2);

	// Run the kinematic consistency check
	_rng_consistency_check.run(z, z_var, vz, vz_var, dist, dist_var, imu_sample.time_us);

	// Track kinematic consistency
	_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	// Publish EstimatorAidSource1d (observation, variance, rejected, fused)
	updateRangeHagl(_aid_src_rng_hgt, sample_corrected, sample.time_us);

	if (!PX4_ISFINITE(_aid_src_rng_hgt.observation) || !PX4_ISFINITE(_aid_src_rng_hgt.observation_variance)) {
		printf("INFINIFY OBSERVATION INVALID\n");
	}

	// Fuse Range data as Primary height source
	// NOTE: terrain is not estimated in this mode
	if (_params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::RANGE)) {
		fuseRangeAsHeightSource();

	} else {
		// Fuse Range data as aiding height source (Primary GPS or Baro)
		fuseRangeAsHeightAiding();
	}
}

void Ekf::fuseRangeAsHeightAiding()
{
	const char *kNotKinematicallyConsistentText = "not kinematically consistent";
	const char *kConditionsFailingText = "conditions failing";

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
		printf("START RNG Terrain fusion\n");
		_control_status.flags.rng_terrain = true;

		// We must reset terrain to range before we start fusing again, otherwise there
		// can be a delta between RNG terrain estimate and previous, which will cause a
		// discontinuity in the lpos.z state
		printf("resetting terrain to range\n");
		resetTerrainToRng(_aid_src_rng_hgt);
		resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);
	}


	//// ALTITUDE FUSION ////

	bool range_aid_conditional = (RngCtrl)_params.ekf2_rng_ctrl == RngCtrl::CONDITIONAL;
	bool range_aid_enabled = (RngCtrl)_params.ekf2_rng_ctrl == RngCtrl::ENABLED;

	bool range_aid_conditions_passed = rangeAidConditionsPassed();

	bool do_range_aid = range_aid_enabled || (range_aid_conditional && range_aid_conditions_passed);

	// Fuse Range into Altitude if:
	// - passes range_aid_conditionalchecks
	// - kinematically consistent
	if (do_range_aid) {

		if (!_control_status.flags.rng_hgt) {
			printf("START RNG Altitude fusion\n");
			_control_status.flags.rng_hgt = true;

			// Reset altitude to rangefinder if on ground
			if (!_control_status.flags.in_air) {
				printf("GND resetting altitude to range\n");
				resetAltitudeTo(_aid_src_rng_hgt.observation - _state.terrain);
			}

			// TODO: review for correctness
			if (_aid_src_rng_hgt.innovation_rejected) {
				// Reset aid source
				printf("resetting aid source\n");
				resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);
			}
		}

	} else {
		stopRangeAltitudeFusion(kConditionsFailingText);
	}

	// If we make it here, fuse
	fuseHaglRng(_aid_src_rng_hgt, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
}

// TODO: remove this mode entirely
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
		printf("initializing range as primary\n");
		_state.terrain = 0.f;

		// TODO: needed? It's set above in --> resetAidSourceStatusZeroInnovation()
		// _aid_src_rng_hgt.time_last_fuse = imu_sample.time_us;
	}

	// TODO:
	// When RNG is primary height source
	if (isHeightResetRequired() && _control_status.flags.rng_hgt && (_height_sensor_ref == HeightSensor::RANGE)) {
		printf("RNG height fusion reset required, all height sources failing\n");

		uint64_t timestamp = _aid_src_rng_hgt.timestamp;
		_information_events.flags.reset_hgt_to_rng = true;
		resetAltitudeTo(_aid_src_rng_hgt.observation - _state.terrain);
		resetAidSourceStatusZeroInnovation(_aid_src_rng_hgt);

		// reset vertical velocity if no valid sources available
		if (!isVerticalVelocityAidingActive()) {
			printf("resetting vertical velocity\n");
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
			printf("RNG AID conditions valid\n");
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
			//  printf("!is_hagl_stable\n");
			// }
			// if (is_horizontal_aiding_active && !is_below_max_speed) {
			if (!is_below_max_speed) {
				printf("!is_below_max_speed\n");
			}

			if (!is_in_range) {
				printf("!is_in_range\n");
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

void Ekf::updateRangeHagl(estimator_aid_source1d_s &aid_src, float measurement, uint64_t time_us)
{
	measurement = math::max(measurement, _params.ekf2_min_rng);
	const float measurement_variance = sq(_params.ekf2_rng_noise) + sq(_params.ekf2_rng_sfe * measurement);

	float innovation_variance;
	sym::ComputeHaglInnovVar(P, measurement_variance, &innovation_variance);

	const float innov_gate = math::max(_params.ekf2_rng_gate, 1.f);
	updateAidSourceStatus(aid_src,
			      time_us,                                   // sample timestamp
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

void Ekf::stopRangeAltitudeFusion(const char *reason)
{
	if (_control_status.flags.rng_hgt) {
		printf("STOP RNG Altitude fusion: %s\n", reason);
		_control_status.flags.rng_hgt = false;

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}
	}
}

void Ekf::stopRangeTerrainFusion(const char *reason)
{
	if (_control_status.flags.rng_terrain) {
		printf("STOP RNG Terrain fusion: %s\n", reason);
		_control_status.flags.rng_terrain = false;
	}
}
