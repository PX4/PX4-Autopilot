/***************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file prec_takeoff.cpp
 *
 * Precision takeoff helper, see prec_takeoff.h.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "prec_takeoff.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>

using namespace time_literals;

static constexpr hrt_abstime kTargetTimeoutUs = 1_s;

PrecTakeoff::PrecTakeoff(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool PrecTakeoff::run(const vehicle_local_position_s &local_pos, position_setpoint_s &current_sp,
		      bool takeoff_reached, hrt_abstime now)
{
	_active = true;
	_reached = takeoff_reached;

	if (takeoff_reached) {
		return false;
	}

	return update_setpoint(local_pos, current_sp, now);
}

bool PrecTakeoff::update_setpoint(const vehicle_local_position_s &local_pos, position_setpoint_s &current_sp,
				  hrt_abstime now)
{
	landing_target_pose_s target_pose;

	if (!_target_pose_sub.update(&target_pose)) {
		return false;
	}

	const bool target_fresh = target_pose.timestamp <= now && now - target_pose.timestamp < kTargetTimeoutUs;
	const bool target_valid = target_pose.abs_pos_valid && PX4_ISFINITE(target_pose.x_abs)
				  && PX4_ISFINITE(target_pose.y_abs);

	if (!target_fresh || !target_valid || !local_pos.xy_global) {
		return false;
	}

	// Follow local origin resets
	if (!_map_ref.isInitialized() || _map_ref.getProjectionReferenceTimestamp() != local_pos.ref_timestamp) {
		_map_ref.initReference(local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp);
	}

	_map_ref.reproject(target_pose.x_abs, target_pose.y_abs, current_sp.lat, current_sp.lon);
	return true;
}

void PrecTakeoff::publish_status()
{
	uint8_t state = prec_takeoff_status_s::PREC_TAKEOFF_STATE_STOPPED;

	if (_active) {
		state = _reached ? prec_takeoff_status_s::PREC_TAKEOFF_STATE_DONE
			: prec_takeoff_status_s::PREC_TAKEOFF_STATE_ONGOING;
	}

	_active = false;

	if (state == _state) {
		return;
	}

	_state = state;

	prec_takeoff_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = state;
	_status_pub.publish(status);
}
