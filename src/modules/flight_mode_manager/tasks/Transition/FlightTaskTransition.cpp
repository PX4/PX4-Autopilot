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
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskTransition::FlightTaskTransition()
{
	_param_handle_pitch_cruise_degrees = param_find("FW_PSP_OFF");
	_param_handle_vt_b_dec_i = param_find("VT_B_DEC_I");
	_param_handle_vt_b_dec_mss = param_find("VT_B_DEC_MSS");

	updateParametersFromStorage();
}

bool FlightTaskTransition::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	_vel_z_filter.setParameters(math::constrain(_deltatime, 0.01f, 0.1f), _vel_z_filter_time_const);

	_decel_error_bt_int = 0.f;

	if (PX4_ISFINITE(last_setpoint.velocity[2])) {
		_vel_z_filter.reset(last_setpoint.velocity[2]);

	} else {
		_vel_z_filter.reset(_velocity(2));
	}

	_velocity_setpoint(2) = _vel_z_filter.getState();

	if (isVtolFrontTransition()) {
		_gear.landing_gear = landing_gear_s::GEAR_UP;

	} else {
		_gear.landing_gear = landing_gear_s::GEAR_DOWN;
	}

	return ret;
}

bool FlightTaskTransition::updateInitialize()
{
	updateParameters();
	updateSubscribers();
	return FlightTask::updateInitialize();
}

bool FlightTaskTransition::update()
{
	// tailsitters will override attitude and thrust setpoint
	// tiltrotors and standard vtol will overrride roll and pitch setpoint but keep vertical thrust setpoint
	bool ret = FlightTask::update();

	_position_setpoint.setAll(NAN);

	// calculate a horizontal acceleration vector which corresponds to an attitude composed of pitch up by _param_pitch_cruise_degrees
	// and zero roll angle
	const Vector2f tmp = Dcm2f(_yaw) * Vector2f(-1.0f, 0.0f);
	float pitch_setpoint = math::radians(_param_pitch_cruise_degrees);

	if (isVtolBackTransition()) {
		pitch_setpoint = computeBackTranstionPitchSetpoint();
	}

	_acceleration_setpoint.xy() = tmp * tanf(math::radians(pitch_setpoint)) * CONSTANTS_ONE_G;

	// slowly move vertical velocity setpoint to zero
	_vel_z_filter.setParameters(math::constrain(_deltatime, 0.01f, 0.1f), _vel_z_filter_time_const);
	_velocity_setpoint(2) = _vel_z_filter.update(0.0f);

	_yaw_setpoint = NAN;
	return ret;
}

void FlightTaskTransition::updateParameters()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParametersFromStorage();
	}
}

void FlightTaskTransition::updateParametersFromStorage()
{
	if (_param_handle_pitch_cruise_degrees != PARAM_INVALID) {
		param_get(_param_handle_pitch_cruise_degrees, &_param_pitch_cruise_degrees);
	}


	if (_param_handle_vt_b_dec_i != PARAM_INVALID) {
		param_get(_param_handle_vt_b_dec_i, &_param_vt_b_dec_i);
	}


	if (_param_handle_vt_b_dec_mss != PARAM_INVALID) {
		param_get(_param_handle_vt_b_dec_mss, &_param_vt_b_dec_mss);
	}

}

void FlightTaskTransition::updateSubscribers()
{
	_sub_vehicle_status.update();
	_sub_position_sp_triplet.update();
	_sub_vehicle_local_position.update();
}

bool FlightTaskTransition::isVtolFrontTransition() const
{
	return _sub_vehicle_status.get().in_transition_mode
	       && _sub_vehicle_status.get().in_transition_to_fw;

}

bool FlightTaskTransition::isVtolBackTransition() const
{
	return _sub_vehicle_status.get().in_transition_mode
	       && !_sub_vehicle_status.get().in_transition_to_fw;
}



float FlightTaskTransition::computeBackTranstionPitchSetpoint()
{
	const vehicle_local_position_s &local_pos = _sub_vehicle_local_position.get();
	const position_setpoint_s &current_pos_sp = _sub_position_sp_triplet.get().current;

	// Retrieve default decelaration setpoint
	const float default_deceleration_sp = _param_vt_b_dec_mss;
	// Maximum allowed deceleration setpoint as a function of the nominal deceleration setpoint
	const float max_deceleration_sp = 2.5f * default_deceleration_sp;

	float deceleration_sp = default_deceleration_sp;

	const float track = atan2f(local_pos.vy, local_pos.vx);
	const float accel_body_forward = cosf(track) * local_pos.ax + sinf(track) * local_pos.ay;
	const float vel_body_forward = cosf(track) * local_pos.vx + sinf(track) * local_pos.vy;


	if (current_pos_sp.valid && local_pos.xy_valid) {

		// Compute backtransition end-point in local reference frame body x-direction -> dist_body_forward
		MapProjection map_proj{local_pos.ref_lat, local_pos.ref_lon};

		float pos_sp_x, pos_sp_y = 0.f;
		map_proj.project(current_pos_sp.lat, current_pos_sp.lon, pos_sp_x,
				 pos_sp_y);

		// Compute backtransition end-point w.r.t. vehicle
		const float pos_sp_dx = pos_sp_x - local_pos.x;
		const float pos_sp_dy = pos_sp_y - local_pos.y;

		// Compute the deceleration setpoint if the backtransition end-point is ahead of the vehicle
		const float vel_proj = local_pos.vx * pos_sp_dx + local_pos.vy * pos_sp_dy;

		if (vel_proj > 0.0f) {
			const float dist_body_forward = cosf(track) * pos_sp_dx + sinf(track) * pos_sp_dy;

			if (fabsf(dist_body_forward) > FLT_EPSILON) {
				// Compute deceleration setpoint
				// Note this is deceleration (i.e. negative acceleration), and therefore the minus sign is skipped
				deceleration_sp = vel_body_forward * vel_body_forward / (2.f * dist_body_forward);

				PX4_INFO("Computed deceleration_sp: %f", (double)(deceleration_sp));

				// Check if the deceleration setpoint is finite and within limits
				if (!PX4_ISFINITE(deceleration_sp)) {
					deceleration_sp = default_deceleration_sp;

				} else {
					// Limit the deceleration setpoint
					deceleration_sp = math::constrain(deceleration_sp, 0.0f, max_deceleration_sp);
				}
			}
		}
	}

	PX4_INFO("Applied deceleration_sp: %f", (double)(deceleration_sp));

	// get accel error, positive means decelerating too slow, need to pitch up (must reverse accel_body_forward, as it is a positive number)
	float deceleration_error = deceleration_sp - (-accel_body_forward);

	float pitch_sp_new = _decel_error_bt_int;
	PX4_INFO("Pitch setpoint: %f", (double)(math::degrees(pitch_sp_new)));

	updateBackTransitioDecelerationErrorIntegrator(deceleration_error);

	return math::constrain(pitch_sp_new, .0f, _pitch_limit_bt);;
}

void FlightTaskTransition::updateBackTransitioDecelerationErrorIntegrator(const float deceleration_error)
{
	float integrator_input = _param_vt_b_dec_i * deceleration_error;
	_decel_error_bt_int += integrator_input * math::constrain(_deltatime, 0.01f, 0.1f);
	// Saturate the integrator value
	_decel_error_bt_int = math::constrain(_decel_error_bt_int, .0f, _pitch_limit_bt);
}
