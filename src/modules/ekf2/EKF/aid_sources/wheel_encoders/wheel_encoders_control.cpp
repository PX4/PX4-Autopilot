/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the disclaimer following this.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the disclaimer in the
 *    accompanying documentation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, LOSS OF USE, DATA, OR PROFITS, OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE) OR OTHERWISE, ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE OR ANY REPRODUCTION OF the SOFTWARE.
 *
 ****************************************************************************/

/**
 * @file wheel_encoders_control.cpp
 * Control functions for EKF wheel encoder fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>
#include <ekf_derivation/generated/compute_wheel_vel_innov_var_and_h.h>

void Ekf::controlWheelEncoderFusion(const imuSample &imu_delayed)
{
	_fc.wheel_encoders.available = (_params.ekf2_wheel_ctrl != 0);

	if (!_wheel_encoder_buffer || !_fc.wheel_encoders.intended()) {
		stopWheelEncoderFusion();
		return;
	}

	wheelEncoderSample sample_delayed{};
	const bool data_ready = _wheel_encoder_buffer->pop_first_older_than(
					imu_delayed.time_us, &sample_delayed);

	if (data_ready) {
		fuseWheelEncoders(sample_delayed);
	}

	if (_control_status.flags.wheel_encoders_fusion
	    && isTimedOut(_aid_src_wheel_encoders.time_last_fuse, 2 * WHEEL_MAX_INTERVAL)) {
		stopWheelEncoderFusion();
	}
}

void Ekf::fuseWheelEncoders(const wheelEncoderSample &sample)
{
	// Differential drive velocity: v = (delta_sr + delta_sl) / (2 * dt)
	const float v_body_x = (sample.delta_sr + sample.delta_sl) / (2.0f * sample.dt);
	const float v_body_y = 0.0f; // Differential drive assumes no lateral slip
	const Vector2f meas_vel(v_body_x, v_body_y);

	// Prediction: v_body = R_to_earth.transpose() * v_earth
	const Vector3f v_earth = _state.vel;
	const Vector3f v_body_pred = _R_to_earth.transpose() * v_earth;
	const Vector2f pred_vel(v_body_pred(0), v_body_pred(1));

	const Vector2f innovation = meas_vel - pred_vel;
	const float R_val = fmaxf(_params.ekf2_wheel_noise, 1e-3f);
	const Vector2f R(R_val, R_val);

	float innov_var[2];
	VectorState Hx, Hy;

	sym::ComputeWheelVelInnovVarAndH(_state.vector(), P, R, innov_var, &Hx, &Hy);

	updateAidSourceStatus(_aid_src_wheel_encoders,
			      sample.time_us,
			      meas_vel,
			      R,
			      innovation,
			      Vector2f(innov_var[0], innov_var[1]),
			      _params.ekf2_wheel_gate);

	if (_aid_src_wheel_encoders.innovation_rejected) {
		return;
	}

	// Fuse Vx
	VectorState Kx = P * Hx / innov_var[0];
	measurementUpdate(Kx, Hx, R(0), innovation(0));

	// Fuse Vy
	VectorState Ky = P * Hy / innov_var[1];
	measurementUpdate(Ky, Hy, R(1), innovation(1));

	_aid_src_wheel_encoders.fused = true;
	_aid_src_wheel_encoders.time_last_fuse = _time_delayed_us;
	_time_last_hor_vel_fuse = _time_delayed_us;

	if (!_control_status.flags.wheel_encoders_fusion) {
		ECL_INFO("starting wheel encoder fusion");
		_control_status.flags.wheel_encoders_fusion = true;
	}
}
void Ekf::stopWheelEncoderFusion()
{
	_control_status.flags.wheel_encoders_fusion = false;
}
