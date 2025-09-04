/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskVtolTakeoffLoiterLand.cpp
 *
 * Flight task for VTOL takeoff, loiter, and land sequence.
 *
 */

#include "FlightTaskVtolTakeoffLoiterLand.hpp"
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

using namespace time_literals;

bool FlightTaskVtolTakeoffLoiterLand::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	// Initialize state
	_state = VtolTakeoffLoiterLandState::TAKEOFF;
	_state_start_time = hrt_absolute_time();
	_loiter_start_time = 0;

	// Store takeoff position
	_takeoff_position = _position;
	_loiter_center = _position;

	// Update parameters
	_loiter_radius = _param_vt_loiter_rad.get();
	_loiter_duration = _param_vt_loiter_time.get();

	// Check if this is a VTOL vehicle
	_vehicle_status_sub.update();
	_is_vtol = _vehicle_status_sub.get().is_vtol;

	if (!_is_vtol) {
		PX4_ERR("VTOL_TAKEOFF_LOITER_LAND mode requires a VTOL vehicle");
		return false;
	}

	PX4_INFO("VTOL_TAKEOFF_LOITER_LAND activated");
	return ret;
}

bool FlightTaskVtolTakeoffLoiterLand::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_vehicle_status_sub.update();
	_position_setpoint_triplet_sub.update();

	// Update VTOL status
	_is_vtol = _vehicle_status_sub.get().is_vtol;
	_in_transition_mode = _vehicle_status_sub.get().in_transition_mode;

	return ret;
}

bool FlightTaskVtolTakeoffLoiterLand::update()
{
	bool ret = FlightTask::update();

	_updateStateMachine();
	_updateSetpoints();

	return ret;
}

void FlightTaskVtolTakeoffLoiterLand::reActivate()
{
	// Reset to takeoff state
	_state = VtolTakeoffLoiterLandState::TAKEOFF;
	_state_start_time = hrt_absolute_time();
	_loiter_start_time = 0;

	FlightTask::reActivate();
}

void FlightTaskVtolTakeoffLoiterLand::_updateStateMachine()
{
	const hrt_abstime now = hrt_absolute_time();
	const float state_duration = (now - _state_start_time) / 1e6f;

	switch (_state) {
	case VtolTakeoffLoiterLandState::TAKEOFF:
		// Check if we've reached takeoff altitude
		if (_position(2) >= _takeoff_position(2) + _param_vto_loiter_alt.get()) {
			_state = VtolTakeoffLoiterLandState::TRANSITION_TO_FW;
			_state_start_time = now;
			PX4_INFO("VTOL: Takeoff complete, transitioning to FW");
		}
		break;

	case VtolTakeoffLoiterLandState::TRANSITION_TO_FW:
		// Check if transition is complete
		if (!_in_transition_mode && state_duration > _param_vt_trans_min_tm.get()) {
			_state = VtolTakeoffLoiterLandState::LOITER_FW;
			_state_start_time = now;
			_loiter_start_time = now;
			PX4_INFO("VTOL: Transition to FW complete, starting FW loiter");
		}
		break;

	case VtolTakeoffLoiterLandState::LOITER_FW:
		// Check if loiter time is complete
		if (state_duration >= _loiter_duration) {
			_state = VtolTakeoffLoiterLandState::TRANSITION_TO_MC;
			_state_start_time = now;
			PX4_INFO("VTOL: FW loiter complete, transitioning to MC");
		}
		break;

	case VtolTakeoffLoiterLandState::TRANSITION_TO_MC:
		// Check if transition is complete
		if (!_in_transition_mode && state_duration > _param_vt_trans_min_tm.get()) {
			_state = VtolTakeoffLoiterLandState::LOITER_MC;
			_state_start_time = now;
			_loiter_start_time = now;
			PX4_INFO("VTOL: Transition to MC complete, starting MC loiter");
		}
		break;

	case VtolTakeoffLoiterLandState::LOITER_MC:
		// Check if loiter time is complete
		if (state_duration >= _loiter_duration) {
			_state = VtolTakeoffLoiterLandState::LAND;
			_state_start_time = now;
			PX4_INFO("VTOL: MC loiter complete, starting landing");
		}
		break;

	case VtolTakeoffLoiterLandState::LAND:
		// Check if we've reached landing altitude
		if (_position(2) <= _takeoff_position(2) + _param_vt_land_alt.get()) {
			_state = VtolTakeoffLoiterLandState::COMPLETE;
			_state_start_time = now;
			PX4_INFO("VTOL: Landing complete, sequence finished");
		}
		break;

	case VtolTakeoffLoiterLandState::COMPLETE:
		// Stay in complete state
		break;
	}
}

void FlightTaskVtolTakeoffLoiterLand::_updateSetpoints()
{
	switch (_state) {
	case VtolTakeoffLoiterLandState::TAKEOFF:
		_setTakeoffSetpoints();
		break;

	case VtolTakeoffLoiterLandState::TRANSITION_TO_FW:
	case VtolTakeoffLoiterLandState::TRANSITION_TO_MC:
		// During transitions, let the transition controller handle setpoints
		// Just maintain current position
		_position_setpoint = _position;
		_velocity_setpoint.setNaN();
		_acceleration_setpoint.setNaN();
		break;

	case VtolTakeoffLoiterLandState::LOITER_FW:
	case VtolTakeoffLoiterLandState::LOITER_MC:
		_setLoiterSetpoints();
		break;

	case VtolTakeoffLoiterLandState::LAND:
		_setLandSetpoints();
		break;

	case VtolTakeoffLoiterLandState::COMPLETE:
		// Hold position
		_position_setpoint = _position;
		_velocity_setpoint.setNaN();
		_acceleration_setpoint.setNaN();
		break;
	}
}

void FlightTaskVtolTakeoffLoiterLand::_setTakeoffSetpoints()
{
	// Takeoff: climb vertically to takeoff altitude
	const float target_altitude = _takeoff_position(2) + _param_vto_loiter_alt.get();

	_position_setpoint = matrix::Vector3f(_takeoff_position(0), _takeoff_position(1), target_altitude);
	_velocity_setpoint.setNaN();
	_acceleration_setpoint.setNaN();

	// Set yaw to current heading
	_yaw_setpoint = _yaw;
}

void FlightTaskVtolTakeoffLoiterLand::_setLoiterSetpoints()
{
	const hrt_abstime now = hrt_absolute_time();
	const float loiter_time = (now - _loiter_start_time) / 1e6f;

	// Calculate loiter position based on time
	const float angle = (loiter_time * 2.0f * M_PI_F) / 30.0f; // Complete circle every 30 seconds
	const float target_altitude = _takeoff_position(2) + _param_vto_loiter_alt.get();

	matrix::Vector3f loiter_position;
	loiter_position(0) = _loiter_center(0) + _loiter_radius * cosf(angle);
	loiter_position(1) = _loiter_center(1) + _loiter_radius * sinf(angle);
	loiter_position(2) = target_altitude;

	_position_setpoint = loiter_position;
	_velocity_setpoint.setNaN();
	_acceleration_setpoint.setNaN();

	// Set yaw to point in direction of travel
	_yaw_setpoint = atan2f(loiter_position(1) - _loiter_center(1),
			       loiter_position(0) - _loiter_center(0));
}

void FlightTaskVtolTakeoffLoiterLand::_setLandSetpoints()
{
	// Land: descend to landing altitude
	const float target_altitude = _takeoff_position(2) + _param_vt_land_alt.get();

	_position_setpoint = matrix::Vector3f(_takeoff_position(0), _takeoff_position(1), target_altitude);
	_velocity_setpoint.setNaN();
	_acceleration_setpoint.setNaN();

	// Set yaw to current heading
	_yaw_setpoint = _yaw;
}
