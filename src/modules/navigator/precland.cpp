/****************************************************************************
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
 * @file precland.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"

#include <drivers/drv_hrt.h>

using namespace time_literals;

#define SEC2USEC 1000000.0f

static constexpr hrt_abstime state_timeout{10_s}; // Maximum time to spend in any state

static constexpr const char *LOST_TARGET_ERROR_MESSAGE = "Lost landing target while landing";

PrecLand::PrecLand() : ModuleParams(nullptr)
{
	_handle_param_acceleration_hor = param_find("MPC_ACC_HOR");
	_handle_param_xy_vel_cruise = param_find("MPC_XY_CRUISE");

	updateParams();
}

void
PrecLand::initialize(const LandingPosition2D &approximate_landing_pos)
{
	_approximate_landing_pos = approximate_landing_pos;
	_local_pos_sub.update();
	_state = PrecLandState::Start;
	_search_cnt = 0;
	_last_slewrate_time = 0;
	_point_reached_time = 0u;

	if (!_map_ref.isInitialized()) {
		_map_ref.initReference(_local_pos_sub.get().ref_lat, _local_pos_sub.get().ref_lon);
	}

	_sp_pev = matrix::Vector2f(0, 0);
	_sp_pev_prev = matrix::Vector2f(0, 0);
	_last_slewrate_time = 0;

	switch_to_state_start();
}

void
PrecLand::update()
{
	// get new input measurement
	_target_pose_updated = _target_pose_sub.update();
	_land_detected_sub.update();
	_global_pos_sub.update();
	_local_pos_sub.update();

	if (_target_pose_updated) {
		_target_pose_valid = true;
	}

	if ((hrt_elapsed_time(&_target_pose_sub.get().timestamp) / 1e6f) > _param_pld_btout.get()) {
		_target_pose_valid = false;
	}

	// stop if we are landed
	if (_land_detected_sub.get().landed) {
		switch_to_state_done();
	}

	switch (_state) {
	case PrecLandState::Start:
		run_state_start();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::DescendAboveTarget:
		run_state_descend_above_target();
		break;

	case PrecLandState::FinalApproach:
		run_state_final_approach();
		break;

	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::Fallback:
		run_state_fallback();
		break;

	case PrecLandState::Done:
		run_state_done();
		break;

	default:
		// unknown state
		break;
	}
}

void
PrecLand::updateParams()
{
	ModuleParams::updateParams();

	if (_handle_param_acceleration_hor != PARAM_INVALID) {
		param_get(_handle_param_acceleration_hor, &_param_acceleration_hor);
	}

	if (_handle_param_xy_vel_cruise != PARAM_INVALID) {
		param_get(_handle_param_xy_vel_cruise, &_param_xy_vel_cruise);
	}
}

void
PrecLand::run_state_start()
{
	_output.nav_cmd = NAV_CMD_WAYPOINT;
	_output.pos_hor.lat = _approximate_landing_pos.lat;
	_output.pos_hor.lon = _approximate_landing_pos.lon;
	_output.alt = _global_pos_sub.get().alt;

	// check if target visible and go to horizontal approach
	if (switch_to_state_horizontal_approach()) {
		return;
	}

	if (_mode == PrecLandMode::Opportunistic) {
		// could not see the target immediately, so just fall back to normal landing
		switch_to_state_fallback();
	}

	float dist = get_distance_to_next_waypoint(_approximate_landing_pos.lat, _approximate_landing_pos.lon,
			_global_pos_sub.get().lat, _global_pos_sub.get().lon);

	// check if we've reached the start point
	if (dist < _acceptance_radius) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		// if we don't see the target after 1 second, search for it
		if (_param_pld_srch_tout.get() > FLT_EPSILON) {

			if (hrt_absolute_time() - _point_reached_time > 1_s) {
				if (!switch_to_state_search()) {
					switch_to_state_fallback();
				}
			}

		} else {
			switch_to_state_fallback();
		}
	}
}

void
PrecLand::run_state_horizontal_approach()
{
	_output.nav_cmd = NAV_CMD_WAYPOINT;
	_output.pos_hor.lat = _approximate_landing_pos.lat;
	_output.pos_hor.lon = _approximate_landing_pos.lon;
	_output.alt = _approach_alt;

	// check if target visible, if not go to start
	if (!check_state_conditions(PrecLandState::HorizontalApproach)) {
		PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);

		// Stop at current altitude
		_output.alt = _global_pos_sub.get().alt;

		if (!switch_to_state_start()) {
			switch_to_state_fallback();
		}

		return;
	}

	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		if (hrt_absolute_time() - _point_reached_time > 2_s) {
			// if close enough for descent above target go to descend above target
			if (switch_to_state_descend_above_target()) {

				return;
			}
		}
	}

	float x = _target_pose_sub.get().x_abs;
	float y = _target_pose_sub.get().y_abs;

	slewrate(x, y);

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	_map_ref.reproject(x, y, _output.pos_hor.lat, _output.pos_hor.lon);
	_approximate_landing_pos.lat = _output.pos_hor.lat;
	_approximate_landing_pos.lon = _output.pos_hor.lon;
}

void
PrecLand::run_state_descend_above_target()
{
	_output.nav_cmd = NAV_CMD_LAND;
	_output.pos_hor.lat = _approximate_landing_pos.lat;
	_output.pos_hor.lon = _approximate_landing_pos.lon;
	_output.alt = _global_pos_sub.get().alt;

	// check if target visible
	if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!switch_to_state_final_approach()) {
			PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);

			// Stay at current altitude for searching for the target
			_output.nav_cmd = NAV_CMD_WAYPOINT;

			if (!switch_to_state_start()) {
				switch_to_state_fallback();
			}
		}

		return;
	}

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	_map_ref.reproject(_target_pose_sub.get().x_abs, _target_pose_sub.get().y_abs, _output.pos_hor.lat,
			   _output.pos_hor.lon);
	_approximate_landing_pos.lat = _output.pos_hor.lat;
	_approximate_landing_pos.lon = _output.pos_hor.lon;
}

void
PrecLand::run_state_final_approach()
{
	_output.nav_cmd = NAV_CMD_LAND;
	_output.pos_hor.lat = _approximate_landing_pos.lat;
	_output.pos_hor.lon = _approximate_landing_pos.lon;
	_output.alt = _global_pos_sub.get().alt;
}

void
PrecLand::run_state_search()
{
	_output.nav_cmd = NAV_CMD_WAYPOINT;
	_output.pos_hor.lat = _approximate_landing_pos.lat;
	_output.pos_hor.lon = _approximate_landing_pos.lon;

	// check if we can see the target
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		if (!_target_acquired_time) {
			// target just became visible. Stop climbing, but give it some margin so we don't stop too abruptly
			_target_acquired_time = hrt_absolute_time();
			float new_alt = _global_pos_sub.get().alt + 1.0f;
			_output.alt = new_alt < _output.alt ? new_alt : _output.alt;
		}

	} else {
		_target_acquired_time = 0u;
		_output.alt = _local_pos_sub.get().ref_alt + _param_pld_srch_alt.get();
	}

	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1_s) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_pld_srch_tout.get() * 1_s) {
		PX4_WARN("Search timed out");

		switch_to_state_fallback();
	}
}

void
PrecLand::run_state_fallback()
{
	_output.nav_cmd = NAV_CMD_LAND;
	_output.pos_hor.lat = _approximate_landing_pos.lat;
	_output.pos_hor.lon = _approximate_landing_pos.lon;
}

void
PrecLand::run_state_done()
{
	_output.nav_cmd = NAV_CMD_IDLE;
	_output.pos_hor.lat = _global_pos_sub.get().lat;
	_output.pos_hor.lon = _global_pos_sub.get().lon;
	_output.alt = _global_pos_sub.get().alt;
}

bool
PrecLand::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start)) {
		_search_cnt++;

		_point_reached_time = 0;

		_state = PrecLandState::Start;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		print_state_switch_message("horizontal approach");
		_approach_alt = _global_pos_sub.get().alt;

		_point_reached_time = 0;

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_descend_above_target()
{
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		print_state_switch_message("descend");
		_state = PrecLandState::DescendAboveTarget;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach)) {
		print_state_switch_message("final approach");
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_search()
{
	PX4_INFO("Climbing to search altitude.");

	_target_acquired_time = 0;

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_fallback()
{
	print_state_switch_message("fallback");

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_done()
{
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

void PrecLand::print_state_switch_message(const char *state_name)
{
	PX4_INFO("Precland: switching to %s", state_name);
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	switch (state) {
	case PrecLandState::Start:
		return _search_cnt <= _param_pld_max_srch.get();

	case PrecLandState::HorizontalApproach:

		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach) {
			if (fabsf(_target_pose_sub.get().x_abs - _local_pos_sub.get().x) < _param_pld_hacc_rad.get()
			    && fabsf(_target_pose_sub.get().y_abs - _local_pos_sub.get().y) < _param_pld_hacc_rad.get()) {
				// we've reached the position where we last saw the target. If we don't see it now, we need to do something
				return _target_pose_valid && _target_pose_sub.get().abs_pos_valid;

			} else {
				// We've seen the target sometime during horizontal approach.
				// Even if we don't see it as we're moving towards it, continue approaching last known location
				return true;
			}
		}

		// If we're trying to switch to this state, the target needs to be visible
		return _target_pose_updated && _target_pose_valid && _target_pose_sub.get().abs_pos_valid;

	case PrecLandState::DescendAboveTarget:

		// if we're already in this state, only leave it if target becomes unusable, don't care about horizontall offset to target
		if (_state == PrecLandState::DescendAboveTarget) {
			// if we're close to the ground, we're more critical of target timeouts so we quickly go into descend
			if (check_state_conditions(PrecLandState::FinalApproach)) {
				return hrt_absolute_time() - _target_pose_sub.get().timestamp < 500_ms; // 0.5s

			} else {
				return _target_pose_valid && _target_pose_sub.get().abs_pos_valid;
			}

		} else {
			// if not already in this state, need to be above target to enter it
			return _target_pose_updated && _target_pose_sub.get().abs_pos_valid
			       && fabsf(_target_pose_sub.get().x_abs - _local_pos_sub.get().x) < _param_pld_hacc_rad.get()
			       && fabsf(_target_pose_sub.get().y_abs - _local_pos_sub.get().y) < _param_pld_hacc_rad.get();
		}

	case PrecLandState::FinalApproach:
		return _target_pose_valid && _target_pose_sub.get().abs_pos_valid
		       && (_target_pose_sub.get().z_abs - _local_pos_sub.get().z) < _param_pld_fappr_alt.get();

	case PrecLandState::Search:
		return true;

	case PrecLandState::Fallback:
		return true;

	default:
		return false;
	}
}

void PrecLand::slewrate(float &sp_x, float &sp_y)
{
	matrix::Vector2f sp_curr(sp_x, sp_y);
	uint64_t now = hrt_absolute_time();

	float dt = (now - _last_slewrate_time);

	if (dt < 1) {
		// bad dt, can't divide by it
		return;
	}

	dt /= SEC2USEC;

	if (!_last_slewrate_time) {
		// running the first time since switching to precland

		// assume dt will be about 50000us
		dt = 50000 / SEC2USEC;

		// set a best guess for previous setpoints for smooth transition
		_sp_pev = _map_ref.project(_output.pos_hor.lat,
					   _output.pos_hor.lon);
		_sp_pev_prev(0) = _sp_pev(0) - _local_pos_sub.get().vx * dt;
		_sp_pev_prev(1) = _sp_pev(1) - _local_pos_sub.get().vy * dt;
	}

	_last_slewrate_time = now;

	// limit the setpoint speed to the maximum cruise speed
	matrix::Vector2f sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > _param_xy_vel_cruise) {
		sp_vel = sp_vel.normalized() * _param_xy_vel_cruise;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	// limit the setpoint acceleration to the maximum acceleration
	matrix::Vector2f sp_acc = (sp_curr - _sp_pev * 2 + _sp_pev_prev) / (dt * dt); // acceleration of the setpoints

	if (sp_acc.length() > _param_acceleration_hor) {
		sp_acc = sp_acc.normalized() * _param_acceleration_hor;
		sp_curr = _sp_pev * 2 - _sp_pev_prev + sp_acc * (dt * dt);
	}

	// limit the setpoint speed such that we can stop at the setpoint given the maximum acceleration/deceleration
	float max_spd = sqrtf(_param_acceleration_hor * ((matrix::Vector2f)(_sp_pev - matrix::Vector2f(sp_x,
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
