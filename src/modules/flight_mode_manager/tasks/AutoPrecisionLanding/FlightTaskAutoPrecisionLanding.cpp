/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "FlightTaskAutoPrecisionLanding.hpp"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

const char* STATE_STRINGS[] = {"Idle", "Start", "HorizontalApproach", "DescendAboveTarget", "Search", "NormalLand", "Finished"};
static constexpr const char *LOST_TARGET_ERROR_MESSAGE = "Lost landing target while landing";

bool FlightTaskAutoPrecisionLanding::activate(const trajectory_setpoint_s &last_setpoint)
{
	// PX4_INFO("FlightTaskAutoPrecisionLanding::activate");
	bool ret = FlightTask::activate(last_setpoint);

	_search_count = 0;
	_last_slewrate_time = 0;
	_landing_target_valid = false;
	_state = PrecLandState::Idle;
	_fix_this_activate_update_loop = true;


	_sub_vehicle_status.update();





	return ret;
}

bool FlightTaskAutoPrecisionLanding::updateInitialize()
{
	// PX4_INFO("updateInitialize");
	bool ret = FlightTask::updateInitialize();

	_sub_home_position.update();
	_sub_vehicle_status.update();

	uint8_t nav_state = _sub_vehicle_status.get().nav_state;

	// Mission precision land
	if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
		_type = WaypointType::land;

	// Auto RTL
	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		_type = WaypointType::loiter;

	// Auto Land
	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		_type = WaypointType::land;
	}

	// require valid position
	ret = ret && _position.isAllFinite() && _velocity.isAllFinite();

	return ret;
}

bool FlightTaskAutoPrecisionLanding::update()
{
	bool ret = FlightTaskAuto::update();

	if (_landing_target_pose_sub.updated()) {
		_landing_target_pose_sub.copy(&_landing_target_pose);
	}

	// target pose can become invalid when the message timed out
	_landing_target_valid = (hrt_elapsed_time(&_landing_target_pose.timestamp) / 1e6f) <= _param_pld_btout.get();

	switch (_state) {

	case PrecLandState::Idle:
		run_state_idle();
		break;

	case PrecLandState::Start:
		run_state_start();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::DescendAboveTarget:
		run_state_descend_above_target();
		break;

	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::NormalLand:
		run_state_normal_land();
		break;

	default:
		// unknown state
		break;
	}

	// Publish status message for debugging
	precision_landing_status_s precision_landing_status{};
	precision_landing_status.timestamp = hrt_absolute_time();
	precision_landing_status.precland_state = (uint8_t) _state;
	_precision_landing_status_pub.publish(precision_landing_status);

	// What is this? Wtf?
	// _constraints.want_takeoff = _checkTakeoff();

	return ret;
}

void FlightTaskAutoPrecisionLanding::switch_state(PrecLandState state)
{
	state_on_exit(_state);
	_state = state;
	state_on_enter(_state);
	PX4_INFO("Switching to %s", STATE_STRINGS[static_cast<int>(state)]);
}

void FlightTaskAutoPrecisionLanding::state_on_enter(PrecLandState state)
{
	switch (state) {
	case PrecLandState::Search:
		_search_count++;
		_search_start_time = hrt_absolute_time();
		break;
	case PrecLandState::HorizontalApproach:
		_horizontal_approach_alt = _position(2);
		break;
	default:
		break;
	}
}

void FlightTaskAutoPrecisionLanding::state_on_exit(PrecLandState state)
{
	// TODO:
	(void)state;
}

void FlightTaskAutoPrecisionLanding::run_state_idle()
{
	if (_fix_this_activate_update_loop) {
		_fix_this_activate_update_loop = false;
		return;
	}

	// PX4_INFO("run_state_idle: waypoint type: %d", (int)_type);

	if (_type == WaypointType::land) {
		switch_state(PrecLandState::Start);
	}
}

void FlightTaskAutoPrecisionLanding::run_state_start()
{
	// Initialize our position setpoint to the current position
	_position_setpoint = _target = _position;


	if (_landing_target_valid) {
		switch_state(PrecLandState::HorizontalApproach);

	} else {

		PX4_INFO("Target not seen");

		// Check if opportunistic mode is enabled
		static constexpr int32_t REQUIRED = 2;
		if (_param_rtl_pld_md.get() == REQUIRED) {
			switch_state(PrecLandState::Search);

		} else {
			switch_state(PrecLandState::NormalLand);
		}
	}
}

void FlightTaskAutoPrecisionLanding::run_state_normal_land()
{
	// Do nothing
}

void FlightTaskAutoPrecisionLanding::run_state_search()
{
	_position_setpoint(2) = _sub_home_position.get().z - _param_pld_srch_alt.get();
	// _velocity_setpoint(0) = _velocity_setpoint(1) = _velocity_setpoint(2) = NAN;

	// If target is seen run horizontal approach
	if (_landing_target_valid) {
		switch_state(PrecLandState::HorizontalApproach);
		return;
	}

	// If we exceed PLD_SRCH_TOUT fallback to normal land
	if ((hrt_elapsed_time(&_search_start_time) / 1e6f) >= _param_pld_srch_tout.get()) {
		PX4_INFO("Search timed out");
		switch_state(PrecLandState::NormalLand);
	}

	// Check if we have exceeded the maximum number of search attempts
	if (_search_count > _param_pld_max_srch.get()) {
		PX4_INFO("Maximum search attempts exceeded (%d)", _search_count);
		switch_state(PrecLandState::NormalLand);
	}
}

void FlightTaskAutoPrecisionLanding::run_state_horizontal_approach()
{
	if (!_landing_target_valid) {
		switch_state(PrecLandState::Search);
		return;
	}

	float x = _landing_target_pose.x_abs;
	float y = _landing_target_pose.y_abs;
	slewrate(x, y); // TODO: Replace this with PX4's AlphaFilter

	// Fly to target XY position, maintain current altitude
	_position_setpoint(0) = x;
	_position_setpoint(1) = y;
	_position_setpoint(2) = _horizontal_approach_alt;

	// Check if it's time to descend
	Vector2f target_position_vector = Vector2f(_landing_target_pose.x_abs,_landing_target_pose.y_abs);
	Vector2f position_delta_vector = Vector2f(target_position_vector - _position.xy());

	if (position_delta_vector.norm() <= _param_pld_hacc_rad.get() ) {
		switch_state(PrecLandState::DescendAboveTarget);
	}
}

void FlightTaskAutoPrecisionLanding::run_state_descend_above_target()
{
	if (!_landing_target_valid) {
		switch_state(PrecLandState::Search);
		return;
	}

	// Let's assume the FlightTask::update() function is preparing our land setpoints.
	_position_setpoint(0) = _landing_target_pose.x_abs;
	_position_setpoint(1) = _landing_target_pose.y_abs;

	// Check if we're within our final approach altitude
	if ((_landing_target_pose.z_abs - _position(2)) < _param_pld_fappr_alt.get()) {
		switch_state(PrecLandState::Finished);
	}
}

void FlightTaskAutoPrecisionLanding::run_state_finished()
{
	// Nothing to do.
}

bool FlightTaskAutoPrecisionLanding::hor_acc_radius_check()
{
	return Vector2f(Vector2f(_landing_target_pose.x_abs,
				 _landing_target_pose.y_abs) - _position.xy()).norm() <= _param_pld_hacc_rad.get();
}

void FlightTaskAutoPrecisionLanding::slewrate(float &sp_x, float &sp_y)
{
	matrix::Vector2f sp_curr(sp_x, sp_y);
	uint64_t now = hrt_absolute_time();

	float dt = (now - _last_slewrate_time);

	if (dt < 1) {
		// bad dt, can't divide by it
		return;
	}

	dt /= 1000000;

	if (_last_slewrate_time == 0) {
		// running the first time since switching to precland
		_sp_pev_prev(0) = sp_x;
		_sp_pev_prev(1) = sp_y;
		_sp_pev(0) = sp_x;
		_sp_pev(1) = sp_y;
	}

	_last_slewrate_time = now;

	// limit the setpoint speed to the maximum cruise speed
	matrix::Vector2f sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > _param_xy_vel_cruise.get()) {
		sp_vel = sp_vel.normalized() * _param_xy_vel_cruise.get();
		sp_curr = _sp_pev + sp_vel * dt;
	}

	// limit the setpoint acceleration to the maximum acceleration
	matrix::Vector2f sp_acc = (sp_curr - _sp_pev * 2 + _sp_pev_prev) / (dt * dt); // acceleration of the setpoints

	if (sp_acc.length() > _param_acceleration_hor.get()) {
		sp_acc = sp_acc.normalized() * _param_acceleration_hor.get();
		sp_curr = _sp_pev * 2 - _sp_pev_prev + sp_acc * (dt * dt);
	}

	// limit the setpoint speed such that we can stop at the setpoint given the maximum acceleration/deceleration
	float max_spd = sqrtf(_param_acceleration_hor.get() * ((matrix::Vector2f)(_sp_pev - matrix::Vector2f(sp_x,
			      sp_y))).length());
	sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > max_spd) {
		sp_vel = sp_vel.normalized() * max_spd;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	_sp_pev_prev = _sp_pev;
	_sp_pev = sp_curr;

	sp_x = sp_curr(0);
	sp_y = sp_curr(1);
}
