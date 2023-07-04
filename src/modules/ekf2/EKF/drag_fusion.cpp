/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file drag_fusion.cpp
 * Body frame drag fusion methods used for multi-rotor wind estimation.
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_drag_x_innov_var_and_k.h"
#include "python/ekf_derivation/generated/compute_drag_y_innov_var_and_k.h"

#include <mathlib/mathlib.h>

void Ekf::controlDragFusion()
{
	if ((_params.drag_ctrl > 0) && _drag_buffer &&
	    !_control_status.flags.fake_pos && _control_status.flags.in_air) {

		if (!_control_status.flags.wind) {
			// reset the wind states and covariances when starting drag accel fusion
			_control_status.flags.wind = true;
			resetWindToZero();
		}

		dragSample drag_sample;

		if (_drag_buffer->pop_first_older_than(_time_delayed_us, &drag_sample)) {
			fuseDrag(drag_sample);
		}
	}
}

void Ekf::fuseDrag(const dragSample &drag_sample)
{
	const float R_ACC = fmaxf(_params.drag_noise, 0.5f); // observation noise variance in specific force drag (m/sec**2)**2
	const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

	// correct rotor momentum drag for increase in required rotor mass flow with altitude
	// obtained from momentum disc theory
	const float mcoef_corrrected = fmaxf(_params.mcoef * sqrtf(rho / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C), 0.f);

	// drag model parameters
	const bool using_bcoef_x = _params.bcoef_x > 1.0f;
	const bool using_bcoef_y = _params.bcoef_y > 1.0f;
	const bool using_mcoef   = _params.mcoef   > 0.001f;

	if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
		return;
	}

	// calculate relative wind velocity in earth frame and rotate into body frame
	const Vector3f rel_wind_earth(_state.vel(0) - _state.wind_vel(0),
				      _state.vel(1) - _state.wind_vel(1),
				      _state.vel(2));
	const Vector3f rel_wind_body = _state.quat_nominal.rotateVectorInverse(rel_wind_earth);
	const float rel_wind_speed = rel_wind_body.norm();
	const Vector24f state_vector_prev = getStateAtFusionHorizonAsVector();

	Vector2f bcoef_inv;

	if (using_bcoef_x) {
		bcoef_inv(0) = 1.0f / _params.bcoef_x;
	}

	if (using_bcoef_y) {
		bcoef_inv(1) = 1.0f / _params.bcoef_y;
	}

	if (using_bcoef_x && using_bcoef_y) {

		// Interpolate between the X and Y bluff body drag coefficients using current relative velocity
		// This creates an elliptic drag distribution around the XY plane
		bcoef_inv(0) = Vector2f(bcoef_inv.emult(rel_wind_body.xy()) / rel_wind_body.xy().norm()).norm();
		bcoef_inv(1) = bcoef_inv(0);
	}

	Vector24f Kfusion;

	// perform sequential fusion of XY specific forces
	for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
		// measured drag acceleration corrected for sensor bias
		const float mea_acc = drag_sample.accelXY(axis_index)  - _state.delta_vel_bias(axis_index) / _dt_ekf_avg;

		// Drag is modelled as an arbitrary combination of bluff body drag that proportional to
		// equivalent airspeed squared, and rotor momentum drag that is proportional to true airspeed
		// parallel to the rotor disc and mass flow through the rotor disc.

		if (axis_index == 0) {
			if (!using_bcoef_x && !using_mcoef) {
				continue;
			}

			sym::ComputeDragXInnovVarAndK(state_vector_prev, P, rho, bcoef_inv(axis_index), mcoef_corrrected, R_ACC, FLT_EPSILON, &_drag_innov_var(axis_index), &Kfusion);

		} else if (axis_index == 1) {
			if (!using_bcoef_y && !using_mcoef) {
				continue;
			}

			sym::ComputeDragYInnovVarAndK(state_vector_prev, P, rho, bcoef_inv(axis_index), mcoef_corrrected, R_ACC, FLT_EPSILON, &_drag_innov_var(axis_index), &Kfusion);
		}

		if (_drag_innov_var(axis_index) < R_ACC) {
			// calculation is badly conditioned
			return;
		}

		const float pred_acc = -0.5f * bcoef_inv(axis_index) * rho * rel_wind_body(axis_index) * rel_wind_speed - rel_wind_body(axis_index) * mcoef_corrrected;

		// Apply an innovation consistency check with a 5 Sigma threshold
		_drag_innov(axis_index) = pred_acc - mea_acc;
		_drag_test_ratio(axis_index) = sq(_drag_innov(axis_index)) / (sq(5.0f) * _drag_innov_var(axis_index));

		// if the innovation consistency check fails then don't fuse the sample
		if (_drag_test_ratio(axis_index) <= 1.0f) {
			measurementUpdate(Kfusion, _drag_innov_var(axis_index), _drag_innov(axis_index));
		}
	}
}
