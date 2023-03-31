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

	_precland_state = PRECLAND_STATE::AUTORTL_CLIMB;

	_position_setpoint = _position;
	_search_count = 0;

	// TODO: Use target orientation instead
	_initial_yaw = _yaw;
	_initial_position = _position;

	return ret;
}

void FlightTaskAutoPrecisionLanding::do_state_transition(PRECLAND_STATE new_state)
{
	_precland_state = new_state;
	_state_start_time = hrt_absolute_time();
}

bool FlightTaskAutoPrecisionLanding::inside_acceptance_radius()
{
	// TODO: Reuse what FlightTaskAuto has...
	return Vector3f(_position_setpoint - _position).norm() <= _param_pld_hacc_rad.get();
}

bool FlightTaskAutoPrecisionLanding::precision_target_available()
{
	// TODO: Add timeout
	const bool ever_received = _landing_target_pose.timestamp != 0;
	const bool timed_out =  hrt_absolute_time() - _landing_target_pose.timestamp > _param_pld_btout.get() * SEC2USEC;
	return ever_received && !timed_out;
}

bool FlightTaskAutoPrecisionLanding::update()
{
	// Get setpoints from FlightTaskAuto and later override if necessary
	bool ret = FlightTask::update();

	// Fetch uorb
	if (_landing_target_pose_sub.updated()) {
		_landing_target_pose_sub.copy(&_landing_target_pose);

		_precision_target_ned(0) = _landing_target_pose.x_abs;
		_precision_target_ned(1) = _landing_target_pose.y_abs;

		// Supplement precision target altitude with home postion's altitude if necessary
		_precision_target_ned(2) = PX4_ISFINITE(_landing_target_pose.z_rel) ? _landing_target_pose.z_rel :
					   _sub_home_position.get().z;
	}

	// Fetch runtime parameters
	const float max_search_duration = _param_pld_srch_tout.get();
	const float search_rel_altitude = _param_pld_srch_alt.get();

	switch (_precland_state) {
	case PRECLAND_STATE::AUTORTL_CLIMB:
		PX4_INFO("AUTORTL_CLIMB");
		_position_setpoint(0) = _initial_position(0);
		_position_setpoint(1) = _initial_position(1);
		_position_setpoint(2) = _sub_home_position.get().z - _param_rtl_return_alt.get();

		_velocity_setpoint.setNaN(); // TODO: Is there a RTL climb speed?
		_acceleration_setpoint.setNaN();

		_yaw_setpoint = _initial_yaw;

		// transition condition
		// Wait for drone to reach z position setpoint or be higher
		if (_position(2) <= _position_setpoint(2) + _param_nav_mc_alt_rad.get()) {
			mavlink_log_info(&_mavlink_log_pub, "Horizontal approach");
			do_state_transition(PRECLAND_STATE::AUTORTL_APPROACH);
		}

		break;

	case PRECLAND_STATE::AUTORTL_APPROACH: {
			PX4_INFO("AUTORTL_APPROACH");

			// Horizontal approach to home or precision target
			if (precision_target_available()) {
				_position_setpoint(0) = _precision_target_ned(0);
				_position_setpoint(1) = _precision_target_ned(1);
				_position_setpoint(2) = _precision_target_ned(2);

			} else {
				PX4_INFO("reverting to HOME...");
				_position_setpoint(0) = _sub_home_position.get().x;
				_position_setpoint(1) = _sub_home_position.get().y;
				_position_setpoint(2) = _sub_home_position.get().z - _param_rtl_return_alt.get();
			}

			_velocity_setpoint.setNaN();

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
		PX4_INFO("MOVE_TO_SEARCH_ALTITUDE");

		// Ascend/Descend to search altitude
		_position_setpoint(0) = _sub_home_position.get().x;
		_position_setpoint(1) = _sub_home_position.get().y;
		_position_setpoint(2) = _sub_home_position.get().z - search_rel_altitude;

		_velocity_setpoint.setNaN();
		_acceleration_setpoint.setNaN();

		_yaw_setpoint = _target_yaw;

		// transition condition
		if (inside_acceptance_radius()) {
			mavlink_log_info(&_mavlink_log_pub, "Starting search");
			do_state_transition(PRECLAND_STATE::SEARCHING_TARGET);
		}

		break;

	case PRECLAND_STATE::SEARCHING_TARGET:
		PX4_INFO("SEARCHING_TARGET");

		_yaw_setpoint = _target_yaw;

		// Currently no search pattern is implemented. Just wait for target...
		// Workaround for Orion app making unwanted changes to my parameters
		if ((hrt_absolute_time() - _state_start_time) > max_search_duration * SEC2USEC) {
			mavlink_log_info(&_mavlink_log_pub, "Search timed out");

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

	case PRECLAND_STATE::MOVING_ABOVE_TARGET:
		PX4_INFO("MOVING_ABOVE_TARGET");
		// TODO: Should use current altitude when state was entered as the altitude setpoint
		_position_setpoint(0) = _precision_target_ned(0);
		_position_setpoint(1) = _precision_target_ned(1);
		_position_setpoint(2) = _sub_home_position.get().z - search_rel_altitude;

		_velocity_setpoint.setNaN();
		_acceleration_setpoint.setNaN();

		_yaw_setpoint = _target_yaw;

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
		_position_setpoint(0) = _precision_target_ned(0);
		_position_setpoint(1) = _precision_target_ned(1);
		_position_setpoint(2) = NAN;

		_velocity_setpoint(0) = 0;
		_velocity_setpoint(1) = 0;
		_velocity_setpoint(2) = _param_mpc_land_speed.get();
		_acceleration_setpoint(2) = NAN;

		_yaw_setpoint = _target_yaw;
		break;

	}

	precision_landing_status_s precision_landing_status{};
	precision_landing_status.timestamp = hrt_absolute_time();
	precision_landing_status.state = _precland_state;
	_precision_landing_status_pub.publish(precision_landing_status);

	return ret;
}
