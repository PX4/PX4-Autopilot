/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
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

#include "ekf.h"

void Ekf::updateVerticalPositionAidStatus(estimator_aid_source1d_s &aid_src, const uint64_t &time_us,
		const float observation, const float observation_variance, const float innovation_gate) const
{
	float innovation = -_gpos.altitude() - observation;
	float innovation_variance = getStateVariance<State::pos>()(2) + observation_variance;

	updateAidSourceStatus(aid_src, time_us,
			      observation, observation_variance,
			      innovation, innovation_variance,
			      innovation_gate);

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

bool Ekf::fuseHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// x & y
	if (!aid_src.innovation_rejected) {
		for (unsigned i = 0; i < 2; i++) {
			fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::pos.idx + i);
		}

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_pos_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

bool Ekf::fuseVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// z
	if (!aid_src.innovation_rejected) {
		fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance,
					   State::pos.idx + 2);

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hgt_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

void Ekf::resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude,
				    const Vector2f &new_horz_pos_var)
{
	const Vector2f delta_horz_pos = computeDeltaHorizontalPosition(new_latitude, new_longitude);

	updateHorizontalPositionResetStatus(delta_horz_pos);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - delta_horz_pos);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	//_gps_pos_b_est.setBias(_gps_pos_b_est.getBias() + _state_reset_status.posNE_change);

	_gpos.setLatLonDeg(new_latitude, new_longitude);
	_output_predictor.resetLatLonTo(new_latitude, new_longitude);

	if (PX4_ISFINITE(new_horz_pos_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, math::max(sq(0.01f), new_horz_pos_var(0)));
	}

	if (PX4_ISFINITE(new_horz_pos_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, math::max(sq(0.01f), new_horz_pos_var(1)));
	}

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;
}

Vector2f Ekf::computeDeltaHorizontalPosition(const double &new_latitude, const double &new_longitude) const
{
	Vector2f pos;
	Vector2f pos_new;

	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pos(0), pos(1));
		_local_origin_lat_lon.project(new_latitude, new_longitude, pos_new(0), pos_new(1));

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pos(0), pos(1));
		zero_ref.project(new_latitude, new_longitude, pos_new(0), pos_new(1));
	}

	return pos_new - pos;
}

Vector2f Ekf::getLocalHorizontalPosition() const
{
	if (_local_origin_lat_lon.isInitialized()) {
		return _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		return zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg());
	}
}

void Ekf::updateHorizontalPositionResetStatus(const Vector2f &delta)
{
	if (_state_reset_status.reset_count.posNE == _state_reset_count_prev.posNE) {
		_state_reset_status.posNE_change = delta;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posNE_change += delta;
	}

	_state_reset_status.reset_count.posNE++;
}

void Ekf::resetHorizontalPositionTo(const Vector2f &new_pos,
				    const Vector2f &new_horz_pos_var)
{
	double new_latitude;
	double new_longitude;

	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.reproject(new_pos(0), new_pos(1), new_latitude, new_longitude);

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.reproject(new_pos(0), new_pos(1), new_latitude, new_longitude);
	}

	resetHorizontalPositionTo(new_latitude, new_longitude, new_horz_pos_var);
}

void Ekf::resetHeightTo(const float new_altitude, float new_vert_pos_var)
{
	const float old_altitude = _gpos.altitude();
	_gpos.setAltitude(new_altitude);

	if (PX4_ISFINITE(new_vert_pos_var)) {
		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, math::max(sq(0.01f), new_vert_pos_var));
	}

	const float delta_z = -(new_altitude - old_altitude);

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_predictor.resetAltitudeTo(new_altitude, delta_z);

	updateVerticalPositionResetStatus(delta_z);

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_b_est.setBias(_baro_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - delta_z);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_GNSS)
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_TERRAIN)
	updateTerrainResetStatus(delta_z);
	_state.terrain += delta_z;
#endif // CONFIG_EKF2_TERRAIN

	// Reset the timout timer
	_time_last_hgt_fuse = _time_delayed_us;
}

void Ekf::updateVerticalPositionResetStatus(const float delta_z)
{
	if (_state_reset_status.reset_count.posD == _state_reset_count_prev.posD) {
		_state_reset_status.posD_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posD_change += delta_z;
	}

	_state_reset_status.reset_count.posD++;
}

void Ekf::updateTerrainResetStatus(const float delta_z)
{
	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_z;
	}

	_state_reset_status.reset_count.hagl++;
}

void Ekf::resetHorizontalPositionToLastKnown()
{
	ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_gpos.latitude_deg(),
		 (double)_last_known_gpos.longitude_deg());
	_information_events.flags.reset_pos_to_last_known = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalPositionTo(_last_known_gpos.latitude_deg(), _last_known_gpos.longitude_deg(),
				  sq(_params.pos_noaid_noise));
}
