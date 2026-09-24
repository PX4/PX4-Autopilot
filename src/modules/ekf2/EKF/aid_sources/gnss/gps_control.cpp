/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * @file gps_control.cpp
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void GnssAiding::update(Ekf &ekf, const imuSample &imu_delayed)
{
	bool any_buffer = false;
	bool any_intended = false;

	for (uint8_t slot = 0; slot < MAX_GNSS_INSTANCES; slot++) {
		ekf._fc.gps[slot].available = (_sources[slot].params.ctrl != 0);
		any_buffer |= (_sources[slot]._buffer != nullptr);
		any_intended |= intended(ekf, slot) && (_sources[slot]._buffer != nullptr);
	}

	if (!any_buffer) {
		ekf.stopGnssFusion();
		return;
	}

	if (!ekf.gyro_bias_inhibited()) {
		ekf._yawEstimator.setGyroBias(ekf.getGyroBias(), ekf._control_status.flags.vehicle_at_rest);
	}

	ekf._yawEstimator.predict(imu_delayed.delta_ang, imu_delayed.delta_ang_dt,
				  imu_delayed.delta_vel, imu_delayed.delta_vel_dt,
				  (ekf._control_status.flags.in_air && !ekf._control_status.flags.vehicle_at_rest));

	if (!any_intended) {
		ekf.stopGnssFusion();
		return;
	}

	// GNSS height, dual antenna yaw and the yaw estimator use a single receiver,
	// restart their fusion when the receiver changes
	const int8_t hgt_slot = selectSlot(ekf, GnssCtrl::VPOS, true);
	const int8_t yaw_slot = selectSlot(ekf, GnssCtrl::YAW, false);
	const int8_t gsf_slot = selectGsfSlot(ekf);

	if (hgt_slot != _hgt_slot) {
		ekf.stopGpsHgtFusion();
		_hgt_slot = hgt_slot;
	}

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (yaw_slot != _yaw_slot) {
		ekf.stopGnssYawFusion();
	}

#endif // CONFIG_EKF2_GNSS_YAW

	_yaw_slot = yaw_slot;

	for (uint8_t slot = 0; slot < MAX_GNSS_INSTANCES; slot++) {
		GnssSource &src = _sources[slot];

		src._hgt_source = (slot == hgt_slot);
		src._yaw_source = (slot == yaw_slot);
		src._gsf_source = (slot == gsf_slot);

		if (!src._buffer || !intended(ekf, slot)) {
			src.stopVel();
			src.stopPos();
			src._data_ready = false;
			continue;
		}

		bool other_slot_vel_fusing = false;
		bool other_slot_pos_fusing = false;

		for (uint8_t other = 0; other < MAX_GNSS_INSTANCES; other++) {
			if (other != slot) {
				other_slot_vel_fusing |= _sources[other].isVelFusing(ekf);
				other_slot_pos_fusing |= _sources[other].isPosFusing(ekf);
			}
		}

		src.update(ekf, imu_delayed, other_slot_vel_fusing, other_slot_pos_fusing);

		updateStatusFlags(ekf);
	}

	updateStatusFlags(ekf);
}

bool GnssAiding::intended(const Ekf &ekf, const uint8_t slot) const
{
	return ekf._fc.gps[slot].intended();
}

int8_t GnssAiding::selectSlot(const Ekf &ekf, const GnssCtrl bit, const bool fallback_to_any) const
{
	for (uint8_t slot = 0; slot < MAX_GNSS_INSTANCES; slot++) {
		if (_sources[slot]._buffer && intended(ekf, slot) && _sources[slot].ctrl(bit)) {
			return slot;
		}
	}

	if (fallback_to_any) {
		// e.g. altitude initialisation when GNSS is the height reference but its height is not fused
		for (uint8_t slot = 0; slot < MAX_GNSS_INSTANCES; slot++) {
			if (_sources[slot]._buffer && intended(ekf, slot)) {
				return slot;
			}
		}
	}

	return -1;
}

int8_t GnssAiding::selectGsfSlot(const Ekf &ekf) const
{
	// prefer a receiver passing the quality checks
	for (uint8_t slot = 0; slot < MAX_GNSS_INSTANCES; slot++) {
		if (_sources[slot]._buffer && intended(ekf, slot) && _sources[slot]._checks.passed()) {
			return slot;
		}
	}

	return selectSlot(ekf, GnssCtrl::VEL, true);
}

void GnssAiding::updateStatusFlags(Ekf &ekf) const
{
	bool any_vel = false;
	bool any_pos = false;
	bool any_intended = false;
	bool all_faulty = true;

	for (uint8_t slot = 0; slot < MAX_GNSS_INSTANCES; slot++) {
		any_vel |= _sources[slot]._vel_active;
		any_pos |= _sources[slot]._pos_active;

		if (intended(ekf, slot)) {
			any_intended = true;
			all_faulty &= _sources[slot]._fault;
		}
	}

	ekf._control_status.flags.gnss_vel = any_vel;
	ekf._control_status.flags.gnss_pos = any_pos;
	ekf._control_status.flags.gnss_fault = any_intended && all_faulty;
}

uint8_t GnssAiding::primarySlot() const
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_sources[i]._vel_active || _sources[i]._pos_active) {
			return i;
		}
	}

	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_sources[i]._sample_delayed.time_us != 0) {
			return i;
		}
	}

	return 0;
}

void GnssAiding::stop(Ekf &ekf)
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		GnssSource &src = _sources[i];

		if (src._vel_active || src._pos_active) {
			src._checks.reset();
		}

		src.stopVel();
		src.stopPos();
		src._data_ready = false;
	}

	_hgt_slot = -1;
	_yaw_slot = -1;

	updateStatusFlags(ekf);
}

void GnssAiding::reset()
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		GnssSource &src = _sources[i];
		src._checks.resetHard();
		src._vel_active = false;
		src._pos_active = false;
		src._fault = false;
		src._data_ready = false;
	}

	_hgt_slot = -1;
	_yaw_slot = -1;
}

float GnssAiding::maxActiveVelTestRatioXY() const
{
	float test_ratio = -1.f;

	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_sources[i]._vel_active) {
			for (int k = 0; k < 2; k++) {
				test_ratio = math::max(test_ratio, fabsf(_sources[i]._aid_src_vel.test_ratio_filtered[k]));
			}
		}
	}

	return test_ratio;
}

float GnssAiding::maxActiveVelTestRatioZ() const
{
	float test_ratio = -1.f;

	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_sources[i]._vel_active) {
			test_ratio = math::max(test_ratio, fabsf(_sources[i]._aid_src_vel.test_ratio_filtered[2]));
		}
	}

	return test_ratio;
}

float GnssAiding::maxActivePosTestRatio() const
{
	float test_ratio = -1.f;

	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_sources[i]._pos_active) {
			for (const auto &test_ratio_filtered : _sources[i]._aid_src_pos.test_ratio_filtered) {
				test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
			}
		}
	}

	return test_ratio;
}

float GnssAiding::maxActiveVelInnovNormXY() const
{
	float innov_norm = 0.f;

	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_sources[i]._vel_active) {
			innov_norm = math::max(innov_norm, Vector2f(_sources[i]._aid_src_vel.innovation).norm());
		}
	}

	return innov_norm;
}

float GnssAiding::maxActivePosInnovNorm() const
{
	float innov_norm = 0.f;

	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_sources[i]._pos_active) {
			innov_norm = math::max(innov_norm, Vector2f(_sources[i]._aid_src_pos.innovation).norm());
		}
	}

	return innov_norm;
}

bool GnssAiding::anyInnovationBad() const
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		const GnssSource &src = _sources[i];

		if ((src._vel_active || src._pos_active)
		    && ((Vector3f(src._aid_src_vel.test_ratio).max() > 1.f) || (Vector2f(src._aid_src_pos.test_ratio).max() > 1.f))) {
			return true;
		}
	}

	return false;
}

void GnssSource::setData(const gnssSample &sample, const uint8_t buffer_length, const uint64_t min_obs_interval_us,
			 const float dt_ekf_avg, const uint64_t time_latest_us)
{
	if (params.ctrl == 0) {
		return;
	}

	// allocated on the first sample of an enabled slot, so that a slot can be enabled at runtime
	if (_buffer == nullptr) {
		_buffer = new TimestampedRingBuffer<gnssSample>(buffer_length);

		if (_buffer == nullptr || !_buffer->valid()) {
			delete _buffer;
			_buffer = nullptr;
			ECL_ERR("GNSS %d buffer allocation failed", _slot);
			return;
		}
	}

	const int64_t time_us = sample.time_us
				- static_cast<int64_t>(dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	if (time_us >= static_cast<int64_t>(_buffer->get_newest().time_us + min_obs_interval_us)) {

		gnssSample sample_new(sample);
		sample_new.time_us = time_us;

		_buffer->push(sample_new);
		_time_last_buffer_push = time_latest_us;

		if (PX4_ISFINITE(sample.yaw)) {
			_time_last_yaw_buffer_push = time_latest_us;
		}

	} else {
		ECL_WARN("GNSS %d data too fast %" PRIi64 " < %" PRIu64 " + %" PRIu64, _slot, time_us,
			 _buffer->get_newest().time_us, min_obs_interval_us);
	}
}

bool GnssSource::isVelFusing(const Ekf &ekf) const
{
	return _vel_active && !ekf.isTimedOut(_aid_src_vel.time_last_fuse, ekf._params.reset_timeout_max);
}

bool GnssSource::isPosFusing(const Ekf &ekf) const
{
	return _pos_active && !ekf.isTimedOut(_aid_src_pos.time_last_fuse, ekf._params.reset_timeout_max);
}

void GnssSource::update(Ekf &ekf, const imuSample &imu_delayed, const bool other_slot_vel_fusing,
			const bool other_slot_pos_fusing)
{
	_intermittent = !ekf.isNewestSampleRecent(_time_last_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_data_ready = _buffer->pop_first_older_than(imu_delayed.time_us, &_sample_delayed);

	if (_data_ready) {
		const gnssSample &gnss_sample = _sample_delayed;

		const bool initial_checks_passed_prev = _checks.initialChecksPassed();

		if (_checks.run(gnss_sample, ekf._time_delayed_us)) {
			if (_checks.initialChecksPassed() && !initial_checks_passed_prev) {
				// First time checks are passing, latching.
				ekf._information_events.flags.gps_checks_passed = true;
			}

		} else {
			// Skip this sample
			_data_ready = false;

			const bool using_gnss = _vel_active || _pos_active;
			const bool gnss_checks_pass_timeout = ekf.isTimedOut(_checks.getLastPassUs(), ekf._params.reset_timeout_max);

			if (using_gnss && gnss_checks_pass_timeout) {
				stop(ekf);
				ECL_WARN("GNSS %d quality poor - stopping use", _slot);
			}
		}

		ekf.updateGnssPos(gnss_sample, _aid_src_pos);
		ekf.updateGnssVel(imu_delayed, gnss_sample, _aid_src_vel);

	} else if (_vel_active || _pos_active) {
		if (!ekf.isNewestSampleRecent(_time_last_buffer_push, ekf._params.reset_timeout_max)) {
			stop(ekf);
			ECL_WARN("GNSS %d data stopped", _slot);
		}
	}

	if (_data_ready) {
#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_yaw_source) {
			ekf.controlGnssYawFusion(*this);
		}

#endif // CONFIG_EKF2_GNSS_YAW

		if (_gsf_source) {
			ekf.controlGnssYawEstimator(_aid_src_vel, params.ctrl);
		}

		bool do_vel_pos_reset = false;

		// while another receiver still constrains the drift, this receiver is the inconsistent one, not the yaw
		const bool other_slot_fusing = other_slot_vel_fusing || other_slot_pos_fusing;

		if (!_fault && !other_slot_fusing && ekf._control_status.flags.in_air && ekf.isYawFailure()) {
			const bool velocity_fusion_failure =  _aid_src_vel.innovation_rejected
							      && ekf.isTimedOut(ekf._time_last_hor_vel_fuse, ekf._params.EKFGSF_reset_delay)
							      && (ekf._time_last_hor_vel_fuse > ekf._time_last_on_ground_us);

			const bool position_fusion_failure =  _aid_src_pos.innovation_rejected
							      && ekf.isTimedOut(ekf._time_last_hor_pos_fuse, ekf._params.EKFGSF_reset_delay)
							      && (ekf._time_last_hor_pos_fuse > ekf._time_last_on_ground_us);

			if ((_vel_active && velocity_fusion_failure)
			    || (_pos_active && position_fusion_failure)) {
				do_vel_pos_reset = ekf.tryYawEmergencyReset();
			}
		}

		controlVelFusion(ekf, do_vel_pos_reset, other_slot_vel_fusing);
		controlPosFusion(ekf, do_vel_pos_reset, other_slot_pos_fusing);
	}
}

void GnssSource::controlVelFusion(Ekf &ekf, const bool force_reset, const bool other_slot_fusing)
{
	const auto &cs = ekf._control_status.flags;

	const bool continuing_conditions_passing = ctrl(GnssCtrl::VEL)
			&& cs.tilt_align
			&& cs.yaw_align
			&& !_fault
			&& !cs.gnss_hgt_fault;
	const bool starting_conditions_passing = continuing_conditions_passing && _checks.passed();

	estimator_aid_source3d_s &aid_src = _aid_src_vel;

	if (_vel_active) {
		if (continuing_conditions_passing) {
			ekf.fuseVelocity(aid_src);

			const bool fusion_timeout = ekf.isTimedOut(aid_src.time_last_fuse, ekf._params.reset_timeout_max);

			if (fusion_timeout || force_reset) {
				if (isVelResetAllowed(ekf, other_slot_fusing) || force_reset) {
					ECL_WARN("GNSS %d fusion timeout, resetting", _slot);
					ekf.resetVelocityToGnss(aid_src);

				} else {
					stopVel();
				}
			}

		} else {
			stopVel();
		}

	} else {
		if (starting_conditions_passing) {
			bool fused = false;

			const bool do_reset = force_reset || !ekf._control_status_prev.flags.yaw_align;

			// Start fusing the data without reset if possible to avoid disturbing the filter
			if (!do_reset && aid_src.test_ratio[0] < 1.f && aid_src.test_ratio[1] < 1.f) {
				fused = ekf.fuseVelocity(aid_src);
			}

			bool reset = false;

			if (!fused && (isVelResetAllowed(ekf, other_slot_fusing) || force_reset)) {
				ekf.resetVelocityToGnss(aid_src);
				reset = true;
			}

			if (fused || reset) {
				ECL_INFO("starting GNSS %d velocity fusion", _slot);
				ekf._information_events.flags.starting_gps_fusion = true;
				_vel_active = true;
			}
		}
	}
}

void GnssSource::controlPosFusion(Ekf &ekf, const bool force_reset, const bool other_slot_fusing)
{
	const auto &cs = ekf._control_status.flags;

	const bool gnss_pos_enabled = ctrl(GnssCtrl::HPOS);

	const bool continuing_conditions_passing = gnss_pos_enabled
			&& cs.tilt_align
			&& cs.yaw_align
			&& !cs.gnss_hgt_fault;
	const bool starting_conditions_passing = continuing_conditions_passing && _checks.passed();
	const bool gpos_init_conditions_passing = gnss_pos_enabled && _checks.passed();

	estimator_aid_source2d_s &aid_src = _aid_src_pos;

	if (_pos_active) {
		if (continuing_conditions_passing) {
			ekf.fuseHorizontalPosition(aid_src);

			const bool fusion_timeout = ekf.isTimedOut(aid_src.time_last_fuse, ekf._params.reset_timeout_max);

			if (fusion_timeout || force_reset) {
				if (isPosResetAllowed(ekf, other_slot_fusing)) {
					ECL_WARN("GNSS %d fusion timeout, resetting", _slot);
					ekf.resetHorizontalPositionToGnss(aid_src);

				} else {
					stopPos();
					_fault = true;
				}
			}

		} else {
			stopPos();
		}

	} else {
		if (starting_conditions_passing) {
			bool fused = false;

			const bool do_reset = force_reset || !ekf._control_status_prev.flags.yaw_align;

			// Start fusing the data without reset if possible to avoid disturbing the filter
			if (ekf._local_origin_lat_lon.isInitialized()
			    && !do_reset
			    && aid_src.test_ratio[0] < 1.f && aid_src.test_ratio[1] < 1.f) {
				fused = ekf.fuseHorizontalPosition(aid_src);
			}

			bool reset = false;

			if ((!fused && isPosResetAllowed(ekf, other_slot_fusing))
			    || (gpos_init_conditions_passing && !ekf._local_origin_lat_lon.isInitialized())) {
				ekf.resetHorizontalPositionToGnss(aid_src);
				reset = true;
			}

			if (fused || reset) {
				ECL_INFO("starting GNSS %d position fusion", _slot);
				ekf._information_events.flags.starting_gps_fusion = true;
				_pos_active = true;
				_fault = false;
			}

		} else if (gpos_init_conditions_passing && !ekf._local_origin_lat_lon.isInitialized()) {
			ekf.resetHorizontalPositionToGnss(aid_src);
		}
	}
}

bool GnssSource::isVelResetAllowed(const Ekf &ekf, const bool other_slot_fusing) const
{
	// a receiver never resets the state while another one is fused
	if (_fault || other_slot_fusing) {
		return false;
	}

	bool allowed = true;

	switch (static_cast<GnssMode>(ekf._params.ekf2_gps_mode)) {
	case GnssMode::kAuto:
		if (ekf.isOtherSourceOfHorizontalVelocityAidingThan(ekf._control_status.flags.gnss_vel)
		    && !ekf._control_status.flags.wind_dead_reckoning) {
			allowed = false;
		}

		break;

	case GnssMode::kDeadReckoning:
		if (ekf.isOtherSourceOfHorizontalAidingThan(ekf._control_status.flags.gnss_vel)) {
			allowed = false;
		}

		break;
	}

	return allowed;
}

bool GnssSource::isPosResetAllowed(const Ekf &ekf, const bool other_slot_fusing) const
{
	// a receiver never resets the state while another one is fused
	if (_fault || other_slot_fusing) {
		return false;
	}

	bool allowed = true;

	switch (static_cast<GnssMode>(ekf._params.ekf2_gps_mode)) {
	case GnssMode::kAuto:
		if (ekf.isOtherSourceOfHorizontalPositionAidingThan(ekf._control_status.flags.gnss_pos)) {
			allowed = false;
		}

		break;

	case GnssMode::kDeadReckoning:
		if (ekf.isOtherSourceOfHorizontalAidingThan(ekf._control_status.flags.gnss_pos)) {
			allowed = false;
		}

		break;
	}

	return allowed;
}

void GnssSource::stop(Ekf &ekf)
{
	if (_vel_active || _pos_active) {
		_checks.reset();
	}

	stopVel();
	stopPos();

	if (_hgt_source) {
		ekf.stopGpsHgtFusion();
	}

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_yaw_source) {
		ekf.stopGnssYawFusion();
	}

#endif // CONFIG_EKF2_GNSS_YAW

	if (_gsf_source) {
		ekf._yawEstimator.reset();
		ekf._time_yaw_estimator_activated_us = 0;
	}
}

void GnssSource::stopVel()
{
	if (_vel_active) {
		ECL_INFO("stopping GNSS %d velocity fusion", _slot);
		_vel_active = false;

		//TODO: what if gnss yaw or height is used?
		if (!_pos_active) {
			_checks.reset();
		}
	}
}

void GnssSource::stopPos()
{
	if (_pos_active) {
		ECL_INFO("stopping GNSS %d position fusion", _slot);
		_pos_active = false;

		//TODO: what if gnss yaw or height is used?
		if (!_vel_active) {
			_checks.reset();
		}
	}
}

void Ekf::stopGnssFusion()
{
	_gnss_aiding.stop(*this);

	stopGpsHgtFusion();
#if defined(CONFIG_EKF2_GNSS_YAW)
	stopGnssYawFusion();
#endif // CONFIG_EKF2_GNSS_YAW

	_yawEstimator.reset();
	_time_yaw_estimator_activated_us = 0;
}

void Ekf::updateGnssVel(const imuSample &imu_sample, const gnssSample &gnss_sample, estimator_aid_source3d_s &aid_src)
{
	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = gnss_sample.pos_body - _params.imu_pos_body;

	const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias;
	const Vector3f vel_offset_body = angular_velocity % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
	const Vector3f velocity = gnss_sample.vel - vel_offset_earth;

	const float vel_var = sq(math::max(gnss_sample.sacc, _params.ekf2_gps_v_noise, 0.01f));
	const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));

	const float innovation_gate = math::max(_params.ekf2_gps_v_gate, 1.f);

	aid_src.device_id = gnss_sample.device_id;

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                  // sample timestamp
			      velocity,                             // observation
			      vel_obs_var,                          // observation variance
			      _state.vel - velocity,                // innovation
			      getVelocityVariance() + vel_obs_var,  // innovation variance
			      innovation_gate);                     // innovation gate

	// vz special case if there is bad vertical acceleration data, then don't reject measurement if GNSS reports velocity accuracy is acceptable,
	// but limit innovation to prevent spikes that could destabilise the filter
	bool bad_acc_vz_rejected = _fault_status.flags.bad_acc_vertical
				   && (aid_src.test_ratio[2] > 1.f)                                   // vz rejected
				   && (aid_src.test_ratio[0] < 1.f) && (aid_src.test_ratio[1] < 1.f); // vx & vy accepted

	if (bad_acc_vz_rejected
	    && (gnss_sample.sacc < _params.ekf2_req_sacc)
	   ) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

void Ekf::updateGnssPos(const gnssSample &gnss_sample, estimator_aid_source2d_s &aid_src)
{
	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_body = gnss_sample.pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = Vector3f(_R_to_earth * pos_offset_body);
	const LatLonAlt measurement(gnss_sample.lat, gnss_sample.lon, gnss_sample.alt);
	const LatLonAlt measurement_corrected = measurement + (-pos_offset_earth);
	const Vector2f innovation = (_gpos - measurement_corrected).xy();

	// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
	float pos_noise = math::max(gnss_sample.hacc, _params.ekf2_gps_p_noise);

	if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gnss_pos)) {
		// if we are not using another source of aiding, then we are reliant on the GNSS
		// observations to constrain attitude errors and must limit the observation noise value.
		if (pos_noise > _params.ekf2_noaid_noise) {
			pos_noise = _params.ekf2_noaid_noise;
		}
	}

	const float pos_var = math::max(sq(pos_noise), sq(0.01f));
	const Vector2f pos_obs_var(pos_var, pos_var);
	const matrix::Vector2d observation(measurement_corrected.latitude_deg(), measurement_corrected.longitude_deg());

	aid_src.device_id = gnss_sample.device_id;

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                                    // sample timestamp
			      observation,                                            // observation
			      pos_obs_var,                                            // observation variance
			      innovation,                                             // innovation
			      Vector2f(getStateVariance<State::pos>()) + pos_obs_var, // innovation variance
			      math::max(_params.ekf2_gps_p_gate, 1.f));            // innovation gate
}

void Ekf::controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel, const int32_t gnss_ctrl)
{
	// update yaw estimator velocity (basic sanity check on GNSS velocity data)
	const float vel_var = aid_src_vel.observation_variance[0];
	const float vel_accuracy = sqrtf(vel_var);
	const Vector2f vel_xy(aid_src_vel.observation);

	if ((vel_var > 0.f)
	    && (vel_accuracy < _params.ekf2_req_sacc)
	    && vel_xy.isAllFinite()) {

		_yawEstimator.fuseVelocity(vel_xy, vel_accuracy, _control_status.flags.in_air);

		if (_yawEstimator.isActive()) {
			if (_time_yaw_estimator_activated_us == 0) {
				_time_yaw_estimator_activated_us = _time_delayed_us;
				_yaw_estimator_restarted_in_air = _control_status.flags.in_air
								  && _yaw_estimator_was_active_in_air;
			}

			if (_control_status.flags.in_air) {
				_yaw_estimator_was_active_in_air = true;
			}

		} else {
			_time_yaw_estimator_activated_us = 0;
		}

		if (!_control_status.flags.in_air) {
			_yaw_estimator_was_active_in_air = false;
			_yaw_estimator_restarted_in_air = false;
		}

		// Try to align yaw using estimate if available
		if (((gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
		     || (gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)))
		    && !_control_status.flags.yaw_align
		    && _control_status.flags.tilt_align) {
			if (resetYawToEKFGSF()) {
				ECL_INFO("GPS yaw aligned using IMU");
			}
		}
	}
}

bool Ekf::tryYawEmergencyReset()
{
	bool success = false;

	/* A rapid reset to the yaw emergency estimate is performed if horizontal velocity innovation checks continuously
	 * fails while the difference between the yaw emergency estimator and the yaw estimate is large.
	 * This enables recovery from a bad yaw estimate. A reset is not performed if the fault condition was
	 * present before flight to prevent triggering due to GPS glitches or other sensor errors.
	 */
	if (resetYawToEKFGSF()) {
		ECL_WARN("GPS emergency yaw reset");

		// in-flight yaw rescue is a signal that gyro_bias_z could be wrong
		// bump its variance so new observations will correct it faster
		resetGyroBiasZCov();

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
			// stop using the magnetometer in the main EKF otherwise its fusion could drag the yaw around
			// and cause another navigation failure
			_control_status.flags.mag_fault = true;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gnss_yaw) {
			_control_status.flags.gnss_yaw_fault = true;
		}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_yaw) {
			_control_status.flags.ev_yaw_fault = true;
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		success = true;
	}

	return success;
}

void Ekf::resetVelocityToGnss(estimator_aid_source3d_s &aid_src)
{
	_information_events.flags.reset_vel_to_gps = true;
	resetVelocityTo(Vector3f(aid_src.observation), Vector3f(aid_src.observation_variance));

	resetAidSourceStatusZeroInnovation(aid_src);
}

void Ekf::resetHorizontalPositionToGnss(estimator_aid_source2d_s &aid_src)
{
	_information_events.flags.reset_pos_to_gps = true;
	resetLatLonTo(aid_src.observation[0], aid_src.observation[1],
		      aid_src.observation_variance[0] +
		      aid_src.observation_variance[1]);

	resetAidSourceStatusZeroInnovation(aid_src);
}

bool Ekf::isYawEmergencyEstimateAvailable() const
{
	// don't allow reet using the EKF-GSF estimate until the filter has started fusing velocity
	// data and the yaw estimate has converged
	if (!_yawEstimator.isActive()) {
		return false;
	}

	const float yaw_var = _yawEstimator.getYawVar();

	return (yaw_var > 0.f)
	       && (yaw_var < sq(_params.EKFGSF_yaw_err_max))
	       && PX4_ISFINITE(yaw_var);
}

bool Ekf::isYawFailure() const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	if (_yaw_estimator_restarted_in_air
	    && ((_time_yaw_estimator_activated_us == 0)
		|| !isTimedOut(_time_yaw_estimator_activated_us, _params.EKFGSF_min_active_time))) {
		return false;
	}

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw());

	return fabsf(yaw_error) > math::radians(25.f);
}

bool Ekf::resetYawToEKFGSF()
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	ECL_INFO("yaw estimator reset heading %.3f -> %.3f rad",
		 (double)getEulerYaw(_R_to_earth), (double)_yawEstimator.getYaw());

	resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

	_control_status.flags.yaw_align = true;
	_information_events.flags.yaw_aligned_to_imu_gps = true;

	return true;
}

bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight);
}
