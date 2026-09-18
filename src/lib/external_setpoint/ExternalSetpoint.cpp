/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ExternalSetpoint.hpp"

#include <mathlib/mathlib.h>

ExternalSetpoint::ExternalSetpoint(ModuleParams *parent) :
	ModuleParams(parent)
{
}

void ExternalSetpoint::modifySetpoint(trajectory_setpoint_s &setpoint, uint8_t nav_state)
{
	_active = false;

	// Feature must be explicitly enabled.
	if (!_param_ext_sp_en.get()) {
		return;
	}

	// Only fuse in Position and Mission mode. In every other mode (Offboard, RTL,
	// Takeoff, Land, ...) we leave the setpoint untouched.
	const bool mode_ok = (nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL)
			     || (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);

	if (!mode_ok) {
		return;
	}

	external_setpoint_s ext;

	if (!_external_setpoint_sub.copy(&ext) || !ext.valid) {
		return;
	}

	// Stale stream -> stop deviating so the underlying mission/position setpoint resumes.
	const hrt_abstime timeout = (hrt_abstime)(_param_ext_sp_timeout.get() * 1e6f);

	if ((hrt_absolute_time() - ext.timestamp) > timeout) {
		return;
	}

	// Fuse: override velocity/acceleration on every commanded (finite) axis. For each
	// axis we override in velocity we also NaN the position setpoint, otherwise the
	// position controller's P-loop would fight the deviation and pull the vehicle back
	// toward the (blocked) mission waypoint. This is the key Mission-mode subtlety:
	// in Position mode the position setpoint is already NaN while moving, so this is a
	// no-op there.
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(ext.velocity[i])) {
			setpoint.velocity[i] = ext.velocity[i];
			setpoint.position[i] = NAN;
		}

		if (PX4_ISFINITE(ext.acceleration[i])) {
			setpoint.acceleration[i] = ext.acceleration[i];
		}
	}

	if (PX4_ISFINITE(ext.yaw)) {
		setpoint.yaw = ext.yaw;
	}

	if (PX4_ISFINITE(ext.yawspeed)) {
		setpoint.yawspeed = ext.yawspeed;
	}

	_active = true;
}
