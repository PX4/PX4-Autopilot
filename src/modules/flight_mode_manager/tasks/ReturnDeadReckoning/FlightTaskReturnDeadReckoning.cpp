
/****************************************************************************
 *
 *   Copyright (c) 2019-2025 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskReturnDeadReckoning.cpp
 */

#include "FlightTaskReturnDeadReckoning.hpp"

#include <px4_platform_common/events.h>

bool FlightTaskReturnDeadReckoning::activate(const trajectory_setpoint_s &last_setpoint)
{
	PX4_INFO("FlightTaskReturnDeadReckoning::activate");

	if (!FlightTask::activate(last_setpoint)) {
		PX4_ERR("Failed to activate task");
		return false;
	}

	_updateSubscriptions();
	_state = State::INIT;

	if (!(_updateBearingToHome() && _initializeSmoothers())) {
		PX4_ERR("Failed to initialize task");
		return false;
	}

	_computeReturnParameters();

	return true;
}

bool FlightTaskReturnDeadReckoning::update()
{
	if (!FlightTask::update()) {
		return false;
	}

	_updateSubscriptions();

	if (_isGlobalPositionValid()) {
		// Update the bearing to home if global position is valid
		if (!_updateBearingToHome()) {
			PX4_ERR("Failed to compute bearing to home");
			return false;
		}
	}

	_updateState();
	_updateSetpoints();
	_updateDistanceFlownEstimate();

	return true;
}

void FlightTaskReturnDeadReckoning::_updateState()
{
	switch (_state) {
	case State::INIT:
		if (_isAboveReturnAltitude()) {
			_state = State::RETURN;
			events::send<float, float>(events::ID("dead_reckon_rtl_return_direct"), events::Log::Info,
						   "Returning to home position to home position at {1:.2m_v} with bearing {2:.2} deg", _rtl_alt,
						   math::degrees(_bearing_to_home));

		} else {
			_state = State::ASCENT;
			events::send<float, float>(events::ID("dead_reckon_rtl_ascent"), events::Log::Info,
						   "Ascending to return altitude {1:.2m_v} with bearing {2:.2} deg", _rtl_alt, math::degrees(_bearing_to_home));
		}

		break;

	case State::ASCENT:
		if (_isAboveReturnAltitude()) {
			_state = State::RETURN;
			events::send<float, float>(events::ID("dead_reckon_rtl_return"), events::Log::Info,
						   "Returning to home position to home position at {1:.2m_v} with bearing {2:.2} deg", _rtl_alt,
						   math::degrees(_bearing_to_home));
		}

		break;

	case State::RETURN:
		if (_isReturnComplete()) {
			_state = State::HOLD;
			events::send<float>(events::ID("dead_reckon_rtl_hold"), events::Log::Info,
					    "Holding altitude at {1:.2m_v} over home position", _rtl_alt);
		}

		break;

	case State::HOLD:
		break;

	default:
		PX4_ERR("Unknown state");
		return;
	}
}

void FlightTaskReturnDeadReckoning::_updateSetpoints()
{
	switch (_state) {
	case State::INIT:
		_slew_rate_acceleration_x.update(0.0f, _deltatime);
		_slew_rate_acceleration_y.update(0.0f, _deltatime);
		_slew_rate_velocity_z.update(0.0f, _deltatime);

		// Hold current altitude
		_velocity_setpoint(2) = _slew_rate_velocity_z.getState();
		break;

	case State::ASCENT:
		_slew_rate_acceleration_x.update(0.0f, _deltatime);
		_slew_rate_acceleration_y.update(0.0f, _deltatime);
		_slew_rate_velocity_z.update(-_param_mpc_z_v_auto_up.get(), _deltatime);

		// Ascent until reaching the return altitude
		_velocity_setpoint(2) = _slew_rate_velocity_z.getState();
		break;

	case State::RETURN:
		_slew_rate_acceleration_x.update(_rtl_acc * cosf(_bearing_to_home), _deltatime);
		_slew_rate_acceleration_y.update(_rtl_acc * sinf(_bearing_to_home), _deltatime);
		_slew_rate_velocity_z.update(0.0f, _deltatime);

		// Stay at the return altitude
		_position_setpoint(2) = -(_rtl_alt - (float) _home_position(2));
		break;

	case State::HOLD:
		_slew_rate_acceleration_x.update(0.0f, _deltatime);
		_slew_rate_acceleration_y.update(0.0f, _deltatime);
		_slew_rate_velocity_z.update(0.0f, _deltatime);

		// Stay at the return altitude
		_position_setpoint(2) = -(_rtl_alt - (float) _home_position(2));
		break;

	default:
		PX4_ERR("Unknown state");
		return;

	};

	// Heading setpoint
	_heading_smoothing.update(_bearing_to_home, _deltatime);
	_yaw_setpoint = _heading_smoothing.getSmoothedHeading();
	_yawspeed_setpoint = _heading_smoothing.getSmoothedHeadingRate();

	// Acceleration setpoint
	_acceleration_setpoint.xy() = matrix::Vector2f(
					      _slew_rate_acceleration_x.getState(),
					      _slew_rate_acceleration_y.getState()
				      );

	return;
}

void FlightTaskReturnDeadReckoning::_updateDistanceFlownEstimate()
{
	if (_state == State::RETURN) {
		_distance_flown_estimate += _param_mpc_xy_vel_max.get() * _deltatime;
	}
}

bool FlightTaskReturnDeadReckoning::_updateBearingToHome()
{
	if (_readHomePosition(_home_position)) {
		_readGlobalPosition(_start_vehicle_global_position);
		_bearing_to_home = _computeBearing(_start_vehicle_global_position, _home_position);

		_distance_flown_estimate = .0f;
		_initial_distance_to_home = get_distance_to_next_waypoint(
						    _start_vehicle_global_position(0), _start_vehicle_global_position(1),
						    _home_position(0), _home_position(1));
	}

	return !isnanf(_bearing_to_home);
}

bool FlightTaskReturnDeadReckoning::_readHomePosition(matrix::Vector3d &home_position)
{
	home_position.setNaN();

	if (_sub_home_position.get().valid_hpos && _sub_home_position.get().valid_alt) {
		home_position(0) = _sub_home_position.get().lat;
		home_position(1) = _sub_home_position.get().lon;
		home_position(2) = (double) _sub_home_position.get().alt;

		return true;
	}

	return false;
}

void FlightTaskReturnDeadReckoning::_readGlobalPosition(matrix::Vector3d &global_position)
{

	global_position.setNaN();

	global_position(0) = _sub_vehicle_global_position.get().lat;
	global_position(1) = _sub_vehicle_global_position.get().lon;
	global_position(2) = (double) _sub_vehicle_global_position.get().alt;
}

float FlightTaskReturnDeadReckoning::_computeBearing(const matrix::Vector3d &_global_position_start,
		const matrix::Vector3d &_global_position_end)
{
	float bearing = NAN;

	if (_global_position_start.isAllFinite() && _global_position_end.isAllFinite()) {
		bearing = get_bearing_to_next_waypoint(_global_position_start(0), _global_position_start(1),
						       _global_position_end(0), _global_position_end(1));
	}

	return bearing;
}

void FlightTaskReturnDeadReckoning::_computeReturnParameters()
{
	_rtl_alt = math::max((float) _start_vehicle_global_position(2),
			     (float) _home_position(2) + _param_rtl_return_alt.get());

	_rtl_acc = _param_mpc_acc_hor_max.get();
}

void FlightTaskReturnDeadReckoning::_updateSubscriptions()
{
	_sub_vehicle_global_position.update();
}

bool FlightTaskReturnDeadReckoning::_initializeSmoothers()
{
	// Initialize the heading smoother
	_heading_smoothing.setMaxHeadingRate(_heading_smoothing_scaling_factor * _param_mpc_yawrauto_max.get());
	_heading_smoothing.setMaxHeadingAccel(_heading_smoothing_scaling_factor * _param_mpc_yawrauto_acc.get());
	_heading_smoothing.reset(_sub_vehicle_local_position.get().heading, 0.0f);

	// Initialize the position smoother
	_slew_rate_velocity_z.setSlewRate(_velocity_z_rate_scaling_factor * _param_mpc_acc_up_max.get());
	_slew_rate_velocity_z.setForcedValue((float) _sub_vehicle_local_position.get().vz);

	// Initialize the acceleration smoothers
	_slew_rate_acceleration_x.setSlewRate(_jerk_scaling_factor * _param_mpc_jerk_auto.get());
	_slew_rate_acceleration_x.setForcedValue(_sub_vehicle_local_position.get().ax);
	_slew_rate_acceleration_y.setSlewRate(_jerk_scaling_factor * _param_mpc_jerk_auto.get());
	_slew_rate_acceleration_y.setForcedValue(_sub_vehicle_local_position.get().ay);

	return true;
}

bool FlightTaskReturnDeadReckoning::_isGlobalPositionValid() const
{
	return _sub_vehicle_global_position.get().lat_lon_valid && _sub_vehicle_global_position.get().alt_valid;
}

bool FlightTaskReturnDeadReckoning::_isAboveReturnAltitude() const
{
	float current_alt = _sub_vehicle_global_position.get().alt;
	float target_alt = _rtl_alt - _param_nav_mc_alt_rad.get();
	return current_alt > target_alt;
}

bool FlightTaskReturnDeadReckoning::_isReturnComplete()
{
	bool ret = false;

	if (_isGlobalPositionValid()) {
		// Close enough to home posititon
		_readGlobalPosition(_start_vehicle_global_position);
		ret = get_distance_to_next_waypoint(
			      _start_vehicle_global_position(0), _start_vehicle_global_position(1),
			      _home_position(0), _home_position(1)) < _param_nav_acc_rad.get();
	}

	if (!ret) {
		ret = _distance_flown_estimate > 2.5f * _initial_distance_to_home; // 2.5x the initial distance to home
	}

	return ret;
}
