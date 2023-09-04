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

/**
 * @file PrecLandTask.cpp
 * @brief Implements the precision-landing VTE task.
 *
 * This task owns the precland-specific subscriptions and per-task cache used by
 * VisionTargetEst. It tracks when precision landing is active, resolves the
 * landing waypoint from Navigator, and seeds the position estimator with that
 * mission landing point whenever the estimator starts or restarts inside the
 * same precision-landing run.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "PrecLandTask.h"

#include <math.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include "../../navigator/navigation.h"
#include "../Position/VTEPosition.h"

namespace vision_target_estimator
{

void PrecLandTask::pollStatus()
{
	prec_land_status_s prec_land_status;

	if (_prec_land_status_sub.update(&prec_land_status)) {
		_is_in_prec_land = prec_land_status.state != prec_land_status_s::PREC_LAND_STATE_STOPPED
				   && prec_land_status.state != prec_land_status_s::PREC_LAND_STATE_DONE;
	}

	if (_is_in_prec_land) {
		updateMissionSetpoint();
	}
}

bool PrecLandTask::isComplete()
{
	vehicle_land_detected_s vehicle_land_detected;

	// Stop computations once the drone has landed.
	if (_vehicle_land_detected_sub.update(&vehicle_land_detected) && vehicle_land_detected.landed) {
		PX4_DEBUG("Land detected, precision landing task completed.");
		_is_in_prec_land = false;
		return true;
	}

	// Stop computations once precision landing is over.
	if (!_is_in_prec_land) {
		PX4_DEBUG("Precision landing task completed.");
		return true;
	}

	return false;
}

void PrecLandTask::onActivate()
{
	resetMissionSetpoint();
}

void PrecLandTask::onPosEstStart(VTEPosition &pos)
{
	if (updateMissionSetpoint()) {
		pos.setMissionPosition(_cached_mission_setpoint.lat, _cached_mission_setpoint.lon,
				       _cached_mission_setpoint.alt);

	} else {
		PX4_WARN("VTE for precision landing, land position cannot be used.");
		pos.setMissionPosition(0.0, 0.0, NAN);
	}
}

const position_setpoint_s *PrecLandTask::findLandSetpoint()
{
	(void)_pos_sp_triplet_sub.update(&_pos_sp_triplet_buffer);

	if (_pos_sp_triplet_buffer.next.valid && _pos_sp_triplet_buffer.next.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		PX4_DEBUG("VTE for precision landing, next sp is land.");
		return &_pos_sp_triplet_buffer.next;
	}

	if (_pos_sp_triplet_buffer.current.valid
	    && _pos_sp_triplet_buffer.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		PX4_DEBUG("VTE for precision landing, current sp is land.");
		return &_pos_sp_triplet_buffer.current;
	}

	return nullptr;
}

bool PrecLandTask::updateMissionSetpoint()
{
	if (_cached_mission_setpoint.valid) {
		return true;
	}

	navigator_mission_item_s navigator_mission_item{};

	// The live triplet is rewritten by precland once the task starts, while navigator_mission_item
	// keeps publishing the logical land item selected by mission or RTL. Prefer that stable source
	// and keep the triplet as a fallback for paths that do not publish navigator mission items.
	if (_navigator_mission_item_sub.copy(&navigator_mission_item) && cacheMissionSetpoint(navigator_mission_item)) {
		return true;
	}

	if (const position_setpoint_s *land_setpoint = findLandSetpoint()) {
		_cached_mission_setpoint = *land_setpoint;
		return true;
	}

	return false;
}

bool PrecLandTask::cacheMissionSetpoint(const navigator_mission_item_s &mission_item)
{
	if (mission_item.nav_cmd != NAV_CMD_LAND && mission_item.nav_cmd != NAV_CMD_VTOL_LAND) {
		return false;
	}

	if (!PX4_ISFINITE(mission_item.latitude) || !PX4_ISFINITE(mission_item.longitude) || !PX4_ISFINITE(mission_item.altitude)) {
		return false;
	}

	float altitude_amsl = mission_item.altitude;

	if (mission_item.altitude_is_relative) {
		home_position_s home_position{};

		if (!_home_position_sub.copy(&home_position) || !PX4_ISFINITE(home_position.alt)) {
			return false;
		}

		altitude_amsl += home_position.alt;
	}

	_cached_mission_setpoint = {};
	_cached_mission_setpoint.valid = true;
	_cached_mission_setpoint.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	_cached_mission_setpoint.lat = static_cast<double>(mission_item.latitude);
	_cached_mission_setpoint.lon = static_cast<double>(mission_item.longitude);
	_cached_mission_setpoint.alt = altitude_amsl;
	return true;
}

} // namespace vision_target_estimator
