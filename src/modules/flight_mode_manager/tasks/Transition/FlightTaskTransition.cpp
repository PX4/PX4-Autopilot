/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskTransition.cpp
 */

#include "FlightTaskTransition.hpp"

using namespace matrix;

FlightTaskTransition::FlightTaskTransition()
{
	param_get(param_find("FW_PSP_OFF"), &_param_fw_psp_off);
	param_get(param_find("VT_B_DEC_I"), &_param_vt_b_dec_i);
	param_get(param_find("VT_B_DEC_MSS"), &_param_vt_b_dec_mss);
}

bool FlightTaskTransition::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	if (PX4_ISFINITE(last_setpoint.velocity[2])) {
		_vel_z_filter.reset(last_setpoint.velocity[2]);

	} else {
		_vel_z_filter.reset(_velocity(2));
	}

	if (_sub_vehicle_status.get().in_transition_to_fw) {
		_gear.landing_gear = landing_gear_s::GEAR_UP;

	} else {
		_gear.landing_gear = landing_gear_s::GEAR_DOWN;
	}

	return ret;
}

bool FlightTaskTransition::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	_sub_vehicle_status.update();
	_sub_position_sp_triplet.update();
	return ret;
}

bool FlightTaskTransition::update()
{
	// tailsitters will override attitude and thrust setpoint
	// tiltrotors and standard vtol will overrride roll and pitch setpoint but keep vertical thrust setpoint
	bool ret = FlightTask::update();

	// slowly move vertical velocity setpoint to zero
	_velocity_setpoint(2) = _vel_z_filter.update(0.0f, _deltatime);

	// calculate a horizontal acceleration vector which corresponds to an attitude composed of pitch up by _param_fw_psp_off
	// and zero roll angle
	float pitch_setpoint = math::radians(_param_fw_psp_off);

	if (!_sub_vehicle_status.get().in_transition_to_fw) {
		pitch_setpoint = computeBackTranstionPitchSetpoint();
	}

	// Calculate horizontal acceleration components to follow a pitch setpoint with the current vehicle heading
	const Vector2f horizontal_acceleration_direction = Dcm2f(_yaw) * Vector2f(-1.0f, 0.0f);
	_acceleration_setpoint.xy() = tanf(pitch_setpoint) * CONSTANTS_ONE_G * horizontal_acceleration_direction;

	_yaw_setpoint = NAN;
	return ret;
}

float FlightTaskTransition::computeBackTranstionPitchSetpoint()
{
	const Vector2f position_xy{_position};
	const Vector2f velocity_xy{_velocity};
	const Vector2f velocity_xy_direction =
		velocity_xy.unit_or_zero(); // Zero when velocity invalid Vector2f(NAN, NAN).unit_or_zero() == Vector2f(0.f, 0.f)

	float deceleration_setpoint = _param_vt_b_dec_mss;

	if (_sub_position_sp_triplet.get().current.valid && _sub_vehicle_local_position.get().xy_global
	    && position_xy.isAllFinite() && velocity_xy.isAllFinite()) {
		Vector2f position_setpoint_local;
		_geo_projection.project(_sub_position_sp_triplet.get().current.lat, _sub_position_sp_triplet.get().current.lon,
					position_setpoint_local(0), position_setpoint_local(1));

		const Vector2f pos_to_target = position_setpoint_local - position_xy; // backtransition end-point w.r.t. vehicle
		const float dist_to_target_in_moving_direction = pos_to_target.dot(velocity_xy_direction);

		if (dist_to_target_in_moving_direction > FLT_EPSILON) {
			// Backtransition target point is ahead of the vehicle, compute the desired deceleration
			deceleration_setpoint = velocity_xy.norm_squared() / (2.f * dist_to_target_in_moving_direction);

		} else {
			deceleration_setpoint = 2.f * _param_vt_b_dec_mss;
		}

		deceleration_setpoint = math::min(deceleration_setpoint, 2.f * _param_vt_b_dec_mss);
	}

	// Pitch up to reach a negative accel_in_flight_direction otherwise we decelerate too slow
	const Vector2f acceleration_xy{_sub_vehicle_local_position.get().ax, _sub_vehicle_local_position.get().ay};
	const float deceleration = -acceleration_xy.dot(velocity_xy_direction); // Zero when velocity invalid
	const float deceleration_error = deceleration_setpoint - deceleration;

	// Update back-transition deceleration error integrator
	_decel_error_bt_int += (_param_vt_b_dec_i * deceleration_error) * _deltatime;
	_decel_error_bt_int = math::constrain(_decel_error_bt_int, 0.f, DECELERATION_INTEGRATOR_LIMIT);
	return _decel_error_bt_int;
}
