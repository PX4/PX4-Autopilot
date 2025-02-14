/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include "StickYaw.hpp"

#include <px4_platform_common/defines.h>

using matrix::wrap_pi;

StickYaw::StickYaw(ModuleParams *parent) :
	ModuleParams(parent)
{}

void StickYaw::reset(const float yaw, const float unaided_yaw)
{
	if (PX4_ISFINITE(unaided_yaw)) {
		_yaw_error_lpf.reset(wrap_pi(yaw - unaided_yaw));
	}
}

void StickYaw::ekfResetHandler(const float delta_yaw)
{
	_yaw_error_lpf.reset(wrap_pi(_yaw_error_lpf.getState() + delta_yaw));
	_yaw_error_ref = wrap_pi(_yaw_error_ref + delta_yaw);
}

void StickYaw::generateYawSetpoint(float &yawspeed_setpoint, float &yaw_setpoint, const float stick_yaw,
				   const float yaw, const float deltatime, const float unaided_yaw)
{
	const float yaw_correction_prev = _yaw_correction;
	const bool reset_setpoint = updateYawCorrection(yaw, unaided_yaw, deltatime);

	if (reset_setpoint) {
		yaw_setpoint = NAN;
	}

	_yawspeed_filter.setParameters(deltatime, _param_mpc_man_y_tau.get());
	const float yawspeed_scale = math::min(math::radians(_param_mpc_man_y_max.get()), _yawspeed_constraint);
	yawspeed_setpoint = _yawspeed_filter.update(stick_yaw * yawspeed_scale);
	yaw_setpoint = updateYawLock(yaw, yawspeed_setpoint, yaw_setpoint, yaw_correction_prev);
}

bool StickYaw::updateYawCorrection(const float yaw, const float unaided_yaw, const float deltatime)
{
	if (!PX4_ISFINITE(unaided_yaw)) {
		_yaw_correction = 0.f;
		return false;
	}

	// Detect the convergence phase of the yaw estimate by monitoring its relative
	// distance from an unaided yaw source.
	const float yaw_error = wrap_pi(yaw - unaided_yaw);

	// Run it through a high-pass filter to detect transients
	const float yaw_error_hpf = wrap_pi(yaw_error - _yaw_error_lpf.getState());
	_yaw_error_lpf.update(yaw_error, deltatime);

	const bool was_converging = _yaw_estimate_converging;
	_yaw_estimate_converging = fabsf(yaw_error_hpf) > _kYawErrorChangeThreshold;

	bool reset_setpoint = false;

	if (!_yaw_estimate_converging) {
		_yaw_error_ref = yaw_error;

		if (was_converging) {
			// Force a reset of the locking mechanism
			reset_setpoint = true;
		}
	}

	_yaw_correction = wrap_pi(yaw_error - _yaw_error_ref);

	return reset_setpoint;
}

float StickYaw::updateYawLock(const float yaw, const float yawspeed_setpoint, const float yaw_setpoint,
			      const float yaw_correction_prev) const
{
	// Yaw-lock depends on desired yawspeed input. If not locked, yaw_sp is set to NAN.
	if (fabsf(yawspeed_setpoint) > FLT_EPSILON) {
		// no fixed heading when rotating around yaw by stick
		return NAN;

	} else {
		// break down and hold the current heading when no more rotation commanded
		if (!PX4_ISFINITE(yaw_setpoint)) {
			return yaw;

		} else {
			return wrap_pi(yaw_setpoint - yaw_correction_prev + _yaw_correction);
		}
	}
}
