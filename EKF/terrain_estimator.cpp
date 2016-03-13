/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file terrain_estimator.cpp
 * Function for fusing rangefinder measurements to estimate terrain vertical position/
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include "mathlib.h"

bool Ekf::initHagl()
{
	// get most recent range measurement from buffer
	rangeSample latest_measurement = _range_buffer.get_newest();

	if ((_time_last_imu - latest_measurement.time_us) < 2e5) {
		// if we have a fresh measurement, use it to initialise the terrain estimator
		_terrain_vpos = _state.pos(2) + latest_measurement.rng;
		// initialise state variance to variance of measurement
		_terrain_var = sq(_params.range_noise);
		// success
		return true;

	} else if (!_in_air) {
		// if on ground we assume a ground clearance
		_terrain_vpos = _state.pos(2) + _params.rng_gnd_clearance;
		// Use the ground clearance value as our uncertainty
		_terrain_var = sq(_params.rng_gnd_clearance);
		// ths is a guess
		return false;

	} else {
		// no information - cannot initialise
		return false;
	}
}

void Ekf::predictHagl()
{
	// predict the state variance growth
	// the state is the vertical position of the terrain underneath the vehicle

	// process noise due to errors in vehicle height estimate
	_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_p_noise);

	// process noise due to terrain gradient
	_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_gradient) * (sq(_state.vel(0)) + sq(_state.vel(
				1)));

	// limit the variance to prevent it becoming badly conditioned
	_terrain_var = math::constrain(_terrain_var, 0.0f, 1e4f);
}

void Ekf::fuseHagl()
{
	// If the vehicle is excessively tilted, do not try to fuse range finder observations
	if (_R_prev(2, 2) > 0.7071f) {
		// get a height above ground measurement from the range finder assuming a flat earth
		float meas_hagl = _range_sample_delayed.rng * _R_prev(2, 2);

		// predict the hagl from the vehicle position and terrain height
		float pred_hagl = _terrain_vpos - _state.pos(2);

		// calculate the innovation
		_hagl_innov = pred_hagl - meas_hagl;

		// calculate the observation variance adding the variance of the vehicles own height uncertainty and factoring in the effect of tilt on measurement error
		float obs_variance = fmaxf(P[8][8], 0.0f) + sq(_params.range_noise / _R_prev(2, 2));

		// calculate the innovation variance - limiting it to prevent a badly conditioned fusion
		_hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

		// perform an innovation consistency check and only fuse data if it passes
		float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
		float test_ratio = sq(_hagl_innov) / (sq(gate_size) * _hagl_innov_var);

		if (test_ratio <= 1.0f) {
			// calculate the Kalman gain
			float gain = _terrain_var / _hagl_innov_var;
			// correct the state
			_terrain_vpos -= gain * _hagl_innov;
			// correct the variance
			_terrain_var = fmaxf(_terrain_var * (1.0f - gain), 0.0f);
			// record last successful fusion time
			_time_last_hagl_fuse = _time_last_imu;
		}


	} else {
		return;
	}
}

// return true if the estimate is fresh
// return the estimated vertical position of the terrain relative to the NED origin
bool Ekf::get_terrain_vert_pos(float *ret)
{
	memcpy(ret, &_terrain_vpos, sizeof(float));

	// The height is useful if the uncertainty in terrain height is significantly smaller than than the estimated height above terrain
	bool accuracy_useful = (sqrtf(_terrain_var) < 0.2f * fmaxf((_terrain_vpos - _state.pos(2)), _params.rng_gnd_clearance));

	if (_time_last_imu - _time_last_hagl_fuse < 1e6 || accuracy_useful) {
		return true;

	} else {
		return false;
	}
}

void Ekf::get_hagl_innov(float *hagl_innov)
{
	memcpy(hagl_innov, &_hagl_innov, sizeof(_hagl_innov));
}


void Ekf::get_hagl_innov_var(float *hagl_innov_var)
{
	memcpy(hagl_innov_var, &_hagl_innov_var, sizeof(_hagl_innov_var));
}
