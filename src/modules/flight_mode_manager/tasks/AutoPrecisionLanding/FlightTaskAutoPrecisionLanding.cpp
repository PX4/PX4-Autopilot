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
 * @file FlightTaskAutoPrecisionLanding.cpp
 */

#include "FlightTaskAutoPrecisionLanding.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskAutoPrecisionLanding::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	PX4_INFO("precland activated");

	_position_setpoint = _position;
	_search_count = 0;

	// TODO: Use target orientation instead
	_yaw_setpoint = _yaw;

	return ret;
}

void FlightTaskAutoPrecisionLanding::do_state_transition(PRECLAND_STATE new_state)
{
	_precland_state = new_state;
	_state_start_time = hrt_absolute_time();
}

bool FlightTaskAutoPrecisionLanding::inside_acceptance_radius()
{
	return Vector3f(_position_setpoint - _position).norm() <= _param_pld_hacc_rad.get();
}

bool FlightTaskAutoPrecisionLanding::update()
{
	bool ret = FlightTaskAuto::update();

	// printf("state: %u\n", _precland_state);

	// Fetch uorb
	if (_landing_target_pose_sub.updated()) {
		_landing_target_pose_sub.copy(&_landing_target_pose);

		_precision_target_ned(0) = _landing_target_pose.x_rel;
		_precision_target_ned(1) = _landing_target_pose.y_rel;
		_precision_target_ned(2) = _landing_target_pose.z_rel;
	}

	// Initialize precision target coordinates with home position if marker has not been spotted yet
	if (_landing_target_pose.timestamp == 0) {
		if (_sub_home_position.get().valid_lpos && _sub_home_position.get().valid_alt) {
			_precision_target_ned(0) = _sub_home_position.get().x;
			_precision_target_ned(1) = _sub_home_position.get().y;
			_precision_target_ned(2) = _sub_home_position.get().z; // TODO: I think this is NAN
		} else {
			// Nothing to do without valid home location or target position
			return ret;
		}

	}

	precision_landing_status_s precision_landing_status{};

	// Fetch runtime parameters
	const float max_search_duration = _param_pld_srch_tout.get();
	const float search_rel_altitude = _param_pld_srch_alt.get();

	switch (_precland_state) {
	case PRECLAND_STATE::AUTORTL_CLIMB:
		PX4_INFO("AUTORTL_CLIMB");

		_position_setpoint = _position;
		_position_setpoint(2) = _precision_target_ned(2) - search_rel_altitude;
		_velocity_setpoint(0) = _velocity_setpoint(1) = _velocity_setpoint(2) = NAN;

		// transition condition
		// Wait for drone to reach z position setpoint
		if (abs(_position(2) - _position_setpoint(2)) <= _param_pld_hacc_rad.get()) {
			mavlink_log_info(&_mavlink_log_pub, "Moving above target");
			do_state_transition(PRECLAND_STATE::AUTORTL_APPROACH);
		}

		break;

	case PRECLAND_STATE::AUTORTL_APPROACH:
		PX4_INFO("AUTORTL_APPROACH");

		_position_setpoint = _precision_target_ned;
		_position_setpoint(2) -= search_rel_altitude;
		_velocity_setpoint(0) = _velocity_setpoint(1) = _velocity_setpoint(2) = NAN;

		// transition condition
		// Wait for drone to reach position setpoint
		if (inside_acceptance_radius()) {
			mavlink_log_info(&_mavlink_log_pub, "Moving to search center");
			do_state_transition(PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE);
		}

		break;

	case PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE:
		PX4_INFO("MOVE_TO_SEARCH_ALTITUDE");

		// Ascend/Descend to search altitude
		_position_setpoint = _precision_target_ned;
		_position_setpoint(2) -= search_rel_altitude;
		_velocity_setpoint(0) = _velocity_setpoint(1) = _velocity_setpoint(2) = NAN;
		_acceleration_setpoint(0) = _acceleration_setpoint(1) = _acceleration_setpoint(2) = NAN;

		// transition condition
		if (inside_acceptance_radius()) {
			mavlink_log_info(&_mavlink_log_pub, "Starting search");
			do_state_transition(PRECLAND_STATE::SEARCHING_TARGET);
		}

		break;

	case PRECLAND_STATE::SEARCHING_TARGET:
		PX4_INFO("SEARCHING_TARGET");

		// Currently no search pattern is implemented. Just wait for target...
		// Workaround for Orion app making unwanted changes to my parameters
		if ((hrt_absolute_time() - _state_start_time) > max_search_duration * SEC2USEC) {
			mavlink_log_info(&_mavlink_log_pub, "Search timed out");

			if (++_search_count >= _param_pld_max_srch.get()) {
				mavlink_log_info(&_mavlink_log_pub, "Abandonning search and landing immediately");
				do_state_transition(PRECLAND_STATE::LANDING);

			} else {
				mavlink_log_info(&_mavlink_log_pub, "Moving to search center");
				do_state_transition(PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE);
			}
		}

		// transition condition
		if (hrt_absolute_time() - _landing_target_pose.timestamp <= _param_pld_btout.get()*SEC2USEC) {
			mavlink_log_info(&_mavlink_log_pub, "Moving above target");
			do_state_transition(PRECLAND_STATE::MOVING_ABOVE_TARGET);
		}

		break;

	case PRECLAND_STATE::MOVING_ABOVE_TARGET:
		PX4_INFO("MOVING_ABOVE_TARGET");

		_position_setpoint(0) = _landing_target_pose.x_abs;
		_position_setpoint(1) = _landing_target_pose.y_abs;
		_position_setpoint(2) = _precision_target_ned(2) - search_rel_altitude;

		_velocity_setpoint(0) = NAN;
		_velocity_setpoint(1) = NAN;
		_velocity_setpoint(2) = 0;

		// Transition condition
		// - Check acceptance radius and
		// - wait long enough to observe if the landing_target_pose timed out before proceeding
		if (abs(_position(0) - _position_setpoint(0)) <= _param_pld_hacc_rad.get()
		    && abs(_position(1) - _position_setpoint(1)) <= _param_pld_hacc_rad.get()
		    && _velocity.norm() <= HOVER_SPEED_THRESHOLD
		    && (hrt_absolute_time() - _state_start_time) > _param_pld_btout.get() * SEC2USEC) {

			if (hrt_absolute_time() - _landing_target_pose.timestamp < _param_pld_btout.get() * SEC2USEC) {
				mavlink_log_info(&_mavlink_log_pub, "Landing on target...");
				do_state_transition(PRECLAND_STATE::LANDING);

			} else {
				mavlink_log_info(&_mavlink_log_pub, "Lost target, back to searching");
				do_state_transition(PRECLAND_STATE::SEARCHING_TARGET);
			}
		}

		break;


	case PRECLAND_STATE::LANDING:
		PX4_INFO("LANDING");
		// Control XY position and Z velocity
		_position_setpoint(0) = _landing_target_pose.x_abs;
		_position_setpoint(1) = _landing_target_pose.y_abs;
		_position_setpoint(2) = NAN;

		_velocity_setpoint(0) = 0;
		_velocity_setpoint(1) = 0;
		_velocity_setpoint(2) = _param_mpc_land_speed.get();
		_acceleration_setpoint(2) = NAN;
		break;

	}

	precision_landing_status.timestamp = hrt_absolute_time();
	precision_landing_status.state = _precland_state;
	_precision_landing_status_pub.publish(precision_landing_status);

	return ret;
}
