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
 * @file FlightTaskPrecisionLanding.cpp
 *
 */

#include "FlightTaskPrecisionLanding.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskPrecisionLanding::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_precland_state = PRECLAND_STATE::AUTORTL_CLIMB;

	_position_setpoint = _position;
	_search_count = 0;

	_initial_yaw = _yaw;
	_initial_position = _position;

	return ret;
}

void FlightTaskPrecisionLanding::do_state_transition(PRECLAND_STATE new_state)
{
	_precland_state = new_state;
	_state_start_time = hrt_absolute_time();
}

bool FlightTaskPrecisionLanding::inside_acceptance_radius()
{
	// TODO: Reuse what FlightTask has...
	return matrix::Vector3f(_position_setpoint - _position).norm() <= _param_pld_hacc_rad.get();
}

bool FlightTaskPrecisionLanding::precision_target_available()
{
	const bool ever_received = _landing_target_pose.timestamp != 0;
	const bool timed_out =  hrt_absolute_time() - _landing_target_pose.timestamp > _param_pld_btout.get() * SEC2USEC;
	return ever_received && !timed_out;
}

void FlightTaskPrecisionLanding::generate_pos_xy_setpoints()
{
	switch (_precland_state) {
	case PRECLAND_STATE::AUTORTL_CLIMB:
		_position_setpoint(0) = _initial_position(0);
		_position_setpoint(1) = _initial_position(1);
		break;

	case PRECLAND_STATE::AUTORTL_APPROACH:
	case PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE:

		// Horizontal approach to home or precision target
		if (precision_target_available()) {
			_position_setpoint(0) = _landing_target_pose.x_abs;
			_position_setpoint(1) = _landing_target_pose.y_abs;

		} else {
			_position_setpoint(0) = _sub_home_position.get().x;
			_position_setpoint(1) = _sub_home_position.get().y;
		}

		break;

	case PRECLAND_STATE::SEARCHING_TARGET:
	case PRECLAND_STATE::MOVING_ABOVE_TARGET:
	case PRECLAND_STATE::LANDING:
		_position_setpoint(0) = _landing_target_pose.x_abs;
		_position_setpoint(1) = _landing_target_pose.y_abs;
		break;
	}
}

void FlightTaskPrecisionLanding::generate_pos_z_setpoints()
{
	const float search_rel_altitude = _param_pld_srch_alt.get();
	switch (_precland_state) {
	case PRECLAND_STATE::AUTORTL_CLIMB:
	case PRECLAND_STATE::AUTORTL_APPROACH:
		// Horizontal approach to home or precision target
		_position_setpoint(2) = _sub_home_position.get().z - _param_rtl_return_alt.get();
		break;

	case PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE:
	case PRECLAND_STATE::SEARCHING_TARGET:
	case PRECLAND_STATE::MOVING_ABOVE_TARGET:
		// TODO: Should use current altitude when state was entered as the altitude setpoint
		if (PX4_ISFINITE(_landing_target_pose.z_abs)){
			// For safety reasons take whichever reference altitude is higher up (lower in NED)
			_position_setpoint(2) = math::min(_sub_home_position.get().z, _landing_target_pose.z_abs) - search_rel_altitude;
		} else {
			_position_setpoint(2) = _sub_home_position.get().z - search_rel_altitude;
		}
		break;

	case PRECLAND_STATE::LANDING:
		_position_setpoint(2) = NAN;
		break;
	}
}

void FlightTaskPrecisionLanding::generate_vel_setpoints()
{
	switch (_precland_state) {
	case PRECLAND_STATE::AUTORTL_CLIMB:
	case PRECLAND_STATE::AUTORTL_APPROACH:
	case PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE:
	case PRECLAND_STATE::SEARCHING_TARGET:
	case PRECLAND_STATE::MOVING_ABOVE_TARGET:
		_velocity_setpoint.setNaN();
		break;

	case PRECLAND_STATE::LANDING:
		_velocity_setpoint(0) = 0;
		_velocity_setpoint(1) = 0;
		_velocity_setpoint(2) = _param_mpc_land_speed.get();
		break;
	}
}

void FlightTaskPrecisionLanding::generate_acc_setpoints()
{
	_acceleration_setpoint.setNaN();
}

void FlightTaskPrecisionLanding::generate_yaw_setpoint()
{
	switch (_precland_state) {
	case PRECLAND_STATE::AUTORTL_CLIMB:
		_yaw_setpoint = _initial_yaw;
		break;

	case PRECLAND_STATE::AUTORTL_APPROACH:
		// TODO: Point in direction of flight
	case PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE:
	case PRECLAND_STATE::SEARCHING_TARGET:
	case PRECLAND_STATE::MOVING_ABOVE_TARGET:
	case PRECLAND_STATE::LANDING:
		_yaw_setpoint = _target_yaw;
		break;
	}
}

void FlightTaskPrecisionLanding::check_state_transitions()
{
	switch (_precland_state) {
	case PRECLAND_STATE::AUTORTL_CLIMB:
		// transition condition
		// Wait for drone to reach z position setpoint or be higher
		if (_position(2) <= _position_setpoint(2) + _param_nav_mc_alt_rad.get()) {
			mavlink_log_info(&_mavlink_log_pub, "Horizontal approach");
			do_state_transition(PRECLAND_STATE::AUTORTL_APPROACH);
		}

		break;

	case PRECLAND_STATE::AUTORTL_APPROACH: {
			// transition condition
			// Wait for drone to reach position setpoint
			if (inside_acceptance_radius()) {
				if (precision_target_available()) {
					mavlink_log_info(&_mavlink_log_pub, "Moving above target");
					do_state_transition(PRECLAND_STATE::MOVING_ABOVE_TARGET);

				} else {
					mavlink_log_info(&_mavlink_log_pub, "Moving to search center");
					do_state_transition(PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE);
				}

				_target_yaw = _yaw_setpoint;
			}

			break;
		}

	case PRECLAND_STATE::MOVE_TO_SEARCH_ALTITUDE:
		// transition condition
		if (inside_acceptance_radius()) {
			mavlink_log_info(&_mavlink_log_pub, "Starting search");
			do_state_transition(PRECLAND_STATE::SEARCHING_TARGET);
		}

		break;

	case PRECLAND_STATE::SEARCHING_TARGET:
		{
			const float max_search_duration = _param_pld_srch_tout.get();

			// Currently no search pattern is implemented. Just wait for target...
			// Workaround for Orion app making unwanted changes to my parameters
			if ((hrt_absolute_time() - _state_start_time) > max_search_duration * SEC2USEC) {
				if (++_search_count >= _param_pld_max_srch.get()) {
					mavlink_log_info(&_mavlink_log_pub, "Abandonning search and landing immediately");
					do_state_transition(PRECLAND_STATE::LANDING);

				} else {
					mavlink_log_info(&_mavlink_log_pub, "Moving to search center");
					do_state_transition(PRECLAND_STATE::SEARCHING_TARGET);
				}
			}

			// transition condition
			if (precision_target_available()) {
				mavlink_log_info(&_mavlink_log_pub, "Moving above target");
				do_state_transition(PRECLAND_STATE::MOVING_ABOVE_TARGET);
			}

			break;
		}

	case PRECLAND_STATE::MOVING_ABOVE_TARGET:
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
		break;
	}
}

bool FlightTaskPrecisionLanding::update()
{
	// Get setpoints from FlightTask and later override if necessary
	bool ret = FlightTask::update();

	// Fetch uorb
	if (_landing_target_pose_sub.updated()) {
		_landing_target_pose_sub.copy(&_landing_target_pose);
	}

	generate_pos_xy_setpoints();
	generate_pos_z_setpoints();
	generate_vel_setpoints();
	generate_acc_setpoints();
	generate_yaw_setpoint();
	check_state_transitions();

	precision_landing_status_s precision_landing_status{};
	precision_landing_status.timestamp = hrt_absolute_time();
	precision_landing_status.state = _precland_state;
	_precision_landing_status_pub.publish(precision_landing_status);

	return ret;
}
