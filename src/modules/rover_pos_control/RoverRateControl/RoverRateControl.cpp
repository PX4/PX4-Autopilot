/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file RoverRateControl.cpp
 */

#include <RoverRateControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void RoverRateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RoverRateControl::setSaturationStatus(const MultirotorMixer::saturation_status &status)
{
	_mixer_saturation_positive[0] = status.flags.roll_pos;
	_mixer_saturation_positive[1] = status.flags.pitch_pos;
	_mixer_saturation_positive[2] = status.flags.yaw_pos;
	_mixer_saturation_negative[0] = status.flags.roll_neg;
	_mixer_saturation_negative[1] = status.flags.pitch_neg;
	_mixer_saturation_negative[2] = status.flags.yaw_neg;
}

Vector3f RoverRateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
				  const float dt, const bool landed)
{
	// /* Do not calculate control signal with bad inputs */
	// if (!(PX4_ISFINITE(ctl_data.body_z_rate))) {

	// 	return math::constrain(_last_output, -1.0f, 1.0f);
	// }

	// /* get the usual dt estimate */
	// uint64_t dt_micros = hrt_elapsed_time(&_last_run);
	// _last_run = hrt_absolute_time();
	// float dt = (float)dt_micros * 1e-6f;

	// /* lock integral for long intervals */
	// bool lock_integrator = ctl_data.lock_integrator;

	// if (dt_micros > 500000) {
	// 	lock_integrator = true;
	// }

	// //TODO: Handle differential rover and ackerman differently
	// // A. Integrator should not integrate if there is no groundspeed for ackerman
	// // B. speed scaling should not be done in differential rovers

	// //Only using vehicle kinematiccs, without considering tire slip

	// /* Calculate body angular rate error */
	// _rate_error = _bodyrate_setpoint - ctl_data.body_z_rate; // body angular rate error

	// float scaler = math::max(ctl_data.groundspeed, 0.1f);

	// if (!lock_integrator && _k_i > 0.0f) {

	// 	float id = _rate_error * dt;

	// 	/*
	// 	 * anti-windup: do not allow integrator to increase if actuator is at limit
	// 	 */
	// 	if (_last_output < -1.0f) {
	// 		/* only allow motion to center: increase value */
	// 		id = math::max(id, 0.0f);

	// 	} else if (_last_output > 1.0f) {
	// 		/* only allow motion to center: decrease value */
	// 		id = math::min(id, 0.0f);
	// 	}

	// 	/* add and constrain */
	// 	_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	// }

	// /* Apply PI rate controller and store non-limited output */
	// _last_output = (_bodyrate_setpoint * _k_ff + _rate_error * _k_p) / scaler + _integrator;
	// _last_output = math::constrain(_last_output, -1.0f, 1.0f);
	// return _last_output;

	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

void RoverRateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_mixer_saturation_positive[i]) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_mixer_saturation_negative[i]) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RoverRateControl::getRoverRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
