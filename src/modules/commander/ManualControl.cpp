/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ManualControl.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;

enum OverrideBits {
	OVERRIDE_AUTO_MODE_BIT = (1 << 0),
	OVERRIDE_OFFBOARD_MODE_BIT = (1 << 1),
	OVERRIDE_IGNORE_THROTTLE_BIT = (1 << 2)
};

bool ManualControl::update()
{
	bool updated = false;

	if (_manual_control_setpoint_sub.updated()) {
		_last_manual_control_setpoint = _manual_control_setpoint;
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

		updated = true;
	}

	_rc_available = _rc_allowed
			&& _manual_control_setpoint.timestamp != 0
			&& (hrt_elapsed_time(&_manual_control_setpoint.timestamp) < (_param_com_rc_loss_t.get() * 1_s));

	return updated && _rc_available;
}

bool ManualControl::wantsOverride(const vehicle_control_mode_s &vehicle_control_mode)
{
	const bool override_auto_mode = (_param_rc_override.get() & OverrideBits::OVERRIDE_AUTO_MODE_BIT)
					&& vehicle_control_mode.flag_control_auto_enabled;

	const bool override_offboard_mode = (_param_rc_override.get() & OverrideBits::OVERRIDE_OFFBOARD_MODE_BIT)
					    && vehicle_control_mode.flag_control_offboard_enabled;

	if (_rc_available && (override_auto_mode || override_offboard_mode)) {
		const float minimum_stick_change = .01f * _param_com_rc_stick_ov.get();

		const bool rpy_moved = (fabsf(_manual_control_setpoint.x - _last_manual_control_setpoint.x) > minimum_stick_change)
				       || (fabsf(_manual_control_setpoint.y - _last_manual_control_setpoint.y) > minimum_stick_change)
				       || (fabsf(_manual_control_setpoint.r - _last_manual_control_setpoint.r) > minimum_stick_change);
		// Throttle change value doubled to achieve the same scaling even though the range is [0,1] instead of [-1,1]
		const bool throttle_moved =
			(fabsf(_manual_control_setpoint.z - _last_manual_control_setpoint.z) * 2.f > minimum_stick_change);
		const bool use_throttle = !(_param_rc_override.get() & OverrideBits::OVERRIDE_IGNORE_THROTTLE_BIT);

		if (rpy_moved || (use_throttle && throttle_moved)) {
			return true;
		}
	}

	return false;
}

bool ManualControl::wantsDisarm(const vehicle_control_mode_s &vehicle_control_mode,
				const vehicle_status_s &vehicle_status, const bool landed)
{
	const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool stick_in_lower_left = isThrottleLow() && (_manual_control_setpoint.r < -.9f);

	const bool mc_manual_thrust_mode = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					   && vehicle_control_mode.flag_control_manual_enabled
					   && !vehicle_control_mode.flag_control_climb_rate_enabled;

	if (armed && (landed || mc_manual_thrust_mode) && stick_in_lower_left) {

		const bool last_disarm_hysteresis = _stick_disarm_hysteresis.get_state();
		_stick_disarm_hysteresis.set_state_and_update(true, hrt_absolute_time());

		if (!last_disarm_hysteresis && _stick_disarm_hysteresis.get_state() && !_stick_arm_hysteresis.get_state()) {
			return true;
		}
	}

	return false;
}

bool ManualControl::wantsArm(const vehicle_control_mode_s &vehicle_control_mode, const vehicle_status_s &vehicle_status,
			     const bool landed)
{
	const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool stick_in_lower_right = isThrottleLow() && (_manual_control_setpoint.r > .9f);

	if (!armed && stick_in_lower_right) {
		const bool last_arm_hysteresis = _stick_arm_hysteresis.get_state();
		_stick_arm_hysteresis.set_state_and_update(true, hrt_absolute_time());

		if (!last_arm_hysteresis && _stick_arm_hysteresis.get_state() && !_stick_disarm_hysteresis.get_state()) {
			return true;
		}
	}

	return false;
}

void ManualControl::updateParams()
{
	ModuleParams::updateParams();
	_stick_disarm_hysteresis.set_hysteresis_time_from(false, _param_rc_arm_hyst.get() * 1_ms);
	_stick_arm_hysteresis.set_hysteresis_time_from(false, _param_rc_arm_hyst.get() * 1_ms);
}
