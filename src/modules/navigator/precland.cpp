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
#include "navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#define SEC2USEC 1000000.0f

#define STATE_TIMEOUT 10000000 // [us] Maximum time to spend in any state

static constexpr const char *LOST_TARGET_ERROR_MESSAGE = "Lost landing target while landing";

PrecLand::PrecLand(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND),
	ModuleParams(navigator)
{
	_handle_param_acceleration_hor = param_find("MPC_ACC_HOR");
	_handle_param_xy_vel_cruise = param_find("MPC_XY_CRUISE");

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
# if defined(CONFIG_VTEST_MOVING)
	_handle_param_z_v_auto_dn = param_find("MPC_Z_V_AUTO_DN");
# endif // CONFIG_VTEST_MOVING
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	updateParams();
}

void
PrecLand::on_activation()
{
	_state = PrecLandState::Start;
	_landing_sp_captured = false;
	_last_vertical_aid = VerticalAid::None;
	_search_cnt = 0;
	_last_slewrate_time = 0;

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	reset_target_yaw_state();
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!_map_ref.isInitialized()) {
		_map_ref.initReference(vehicle_local_position->ref_lat, vehicle_local_position->ref_lon, hrt_absolute_time());
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Keep the setpoint the active mode left, then reset the triplet. Started by a mission landing, also
	// one that goes on after Return was commanded, that setpoint is its land point and we continue to it.
	// Return starts it the same way and then sets its own land setpoint. Selected as its own mode, the
	// setpoint belongs to the previous mode and we start at the current position.
	const position_setpoint_s previous_setpoint = pos_sp_triplet->current;
	const bool entered_as_mode = (_navigator->get_vstatus()->nav_state == getNavigatorStateId());
	_navigator->reset_triplets();

	_landing_alt_valid = previous_setpoint.valid && !entered_as_mode;

	if (previous_setpoint.valid && !entered_as_mode) {
		pos_sp_triplet->current = previous_setpoint;

	} else {
		PX4_WARN("Reset");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
		pos_sp_triplet->current.timestamp = hrt_absolute_time();
	}

	_sp_pev = matrix::Vector2f(0, 0);
	_sp_pev_prev = matrix::Vector2f(0, 0);
	_last_slewrate_time = 0;

	_start_alt = _navigator->get_global_position()->alt;

	const home_position_s *home = _navigator->get_home_position();
	_ground_alt = home->valid_alt ? home->alt : 0.0f;

	switch_to_state_start();

	_is_activated = true;
}

void
PrecLand::on_active()
{
	// get new target measurement
	_target_pose_updated = _target_pose_sub.update(&_target_pose);

	if (_target_pose_updated) {
		_target_pose_valid = true;
	}

	if ((hrt_elapsed_time(&_target_pose.timestamp) / 1e6f) > _param_pld_btout.get()) {
		_target_pose_valid = false;
	}

	update_ground_alt();

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	// get orientation measurement
	if (_param_pld_yaw_en.get()) {
		vte_orientation_s target_orientation;

		if (_target_orientation_sub.update(&target_orientation)) {
			if (target_orientation.orientation_valid) {
				_target_yaw_valid = true;
				_last_target_yaw_update = target_orientation.timestamp;
				_target_yaw = target_orientation.yaw;

			} else {
				reset_target_yaw_state();
			}
		}

		if (_target_yaw_valid && ((hrt_elapsed_time(&_last_target_yaw_update) / 1e6f) > _param_pld_btout.get())) {
			reset_target_yaw_state();
		}

	} else {
		reset_target_yaw_state();
	}

#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	// stop if we are landed
	if (_navigator->get_land_detected()->landed && _state != PrecLandState::Done) {
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
		// nothing to do
		break;

	default:
		// unknown state
		break;
	}

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	publish_prec_land_status(map_prec_land_state(_state));
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
}

void
PrecLand::on_inactivation()
{
	clear_current_yaw_setpoint();
	_navigator->set_position_setpoint_triplet_updated();

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	reset_target_yaw_state();
	publish_prec_land_status(prec_land_status_s::PREC_LAND_STATE_STOPPED);
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	_is_activated = false;
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

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
# if defined(CONFIG_VTEST_MOVING)

	if (_handle_param_z_v_auto_dn != PARAM_INVALID) {
		param_get(_handle_param_z_v_auto_dn, &_param_z_v_auto_dn);
	}

# endif // CONFIG_VTEST_MOVING
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
}

void
PrecLand::run_state_start()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	if (!_landing_sp_captured) {
		_landing_alt = pos_sp_triplet->current.alt;
		_landing_lat = pos_sp_triplet->current.lat;
		_landing_lon = pos_sp_triplet->current.lon;
		_landing_sp_captured = true;

		_landing_alt_valid = _landing_alt_valid && PX4_ISFINITE(_landing_alt);

		if (_landing_alt_valid) {
			_ground_alt = _landing_alt;
		}
	}

	float dist = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
			_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	// check if we've reached the start point
	if (dist < _navigator->get_acceptance_radius()) {
		if (!switch_to_state_search()) {
			switch_to_state_fallback();
		}

		return;
	}

	// Hold current altitude during start
	pos_sp_triplet->current.alt = _start_alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::run_state_horizontal_approach()
{
	if (!check_state_conditions(PrecLandState::HorizontalApproach)) {
		PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);

		if (!switch_to_state_search()) {
			switch_to_state_fallback();
		}

		return;
	}

	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		if (hrt_absolute_time() - _point_reached_time > 2000000) {
			// if close enough for descent above target go to descend above target
			if (!switch_to_state_descend_above_target()) {
				switch_to_state_fallback();
				return;
			}
		}

	} else {
		_point_reached_time = 0;
	}

	aid_horizontally(VerticalAid::HoldAlt, true, true);
}

void
PrecLand::run_state_descend_above_target()
{
	if (switch_to_state_final_approach()) {
		return;
	}

	// check if target visible and above approach altitude
	if (!check_state_conditions(PrecLandState::DescendAboveTarget) || below_final_approach_alt()) {
		if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
			PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);
		}

		if (!switch_to_state_horizontal_approach() && !switch_to_state_search()) {
			switch_to_state_fallback();
		}

		return;
	}

	aid_horizontally(VerticalAid::None, true, false);
}

void
PrecLand::run_state_final_approach()
{
	// nothing to do, will land
}

void
PrecLand::run_state_search()
{
	// If higher than SEARCH_ALT AGL, stay in landing and only aid in the horizontal movement if target acquired
	if (!_below_search_alt && _navigator->get_global_position()->alt > _ground_alt + _param_pld_srch_alt.get()) {
		_point_reached_time = hrt_absolute_time(); // Time when search_altitude was reached (beginning of timeout)

		if (!target_acquired()) {
			reset_to_landing_setpoint();
			return;
		}

		aid_horizontally(VerticalAid::None, true, true);
		return;
	}

	_below_search_alt = true;

	// If we cannot see the target, climb to SEARCH_ALT directly above target
	if (!target_acquired()) {
		_target_acquired_time = 0;

		if (_param_pld_srch_tout.get() < FLT_EPSILON || hrt_absolute_time() - _point_reached_time > _param_pld_srch_tout.get()*SEC2USEC) {
			if (!(_param_pld_srch_tout.get() < FLT_EPSILON)) {
				PX4_WARN("Search timed out");
			}

			switch_to_state_fallback();
			return;
		}

		aid_horizontally(VerticalAid::ClimbToSearch, false, true);
		return;
	}

	if (!_target_acquired_time) {
		// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
		_target_acquired_time = hrt_absolute_time();
	}

	// stay at that height for a second to allow the vehicle to settle
	if ((hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	aid_horizontally(VerticalAid::HoldAlt, true, true);
}

void
PrecLand::run_state_fallback()
{
	if (_fallback_xy_frozen) {
		// keep the last horizontal setpoint until touchdown
		return;
	}

	// Stop following the target close to the ground, like in final approach
	const vehicle_land_detected_s *land_detected = _navigator->get_land_detected();

	if (below_final_approach_alt() || land_detected->ground_contact || land_detected->maybe_landed) {
		_fallback_xy_frozen = true;
		return;
	}

	// Keep landing, but move towards the target if it becomes visible again
	aid_horizontally(VerticalAid::None, false, true);
}

bool
PrecLand::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start)) {
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

		_last_slewrate_time = 0;
		aid_horizontally(VerticalAid::HoldAlt, true, true);

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
		clear_current_yaw_setpoint();
		_navigator->set_position_setpoint_triplet_updated();
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_search()
{
	if (check_state_conditions(PrecLandState::Search)) {

		PX4_INFO("Moving to search altitude.");

		_target_acquired_time = 0;
		_point_reached_time = hrt_absolute_time();
		_below_search_alt = false;

		_state = PrecLandState::Search;
		_search_cnt++;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_fallback()
{
	print_state_switch_message("fallback");
	reset_to_landing_setpoint();

	_fallback_xy_frozen = false;
	_last_slewrate_time = 0;

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_done()
{
	clear_current_yaw_setpoint();
	_navigator->set_position_setpoint_triplet_updated();
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

void PrecLand::print_state_switch_message(const char *state_name)
{
	PX4_INFO("Precland: switching to %s", state_name);
}

matrix::Vector2f PrecLand::get_target_position_setpoint()
{
	matrix::Vector2f target_position_sp(_target_pose.x_abs, _target_pose.y_abs);

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
# if defined(CONFIG_VTEST_MOVING)
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (_target_pose.is_static || !_target_pose.rel_vel_valid || !vehicle_local_position->v_xy_valid) {
		return target_position_sp;
	}

	const matrix::Vector2f target_rel_velocity(_target_pose.vx_rel, _target_pose.vy_rel);
	const matrix::Vector2f vehicle_velocity(vehicle_local_position->vx, vehicle_local_position->vy);

	if (!PX4_ISFINITE(_target_pose.z_rel) || !target_rel_velocity.isAllFinite() || !vehicle_velocity.isAllFinite()) {
		return target_position_sp;
	}

	float mpc_z_v_auto_dn = _param_z_v_auto_dn;
	static constexpr float kMinAbsMpcZVAutoDnMps = 1e-3f;

	if (fabsf(mpc_z_v_auto_dn) < kMinAbsMpcZVAutoDnMps) {
		mpc_z_v_auto_dn = (mpc_z_v_auto_dn >= 0.f) ? kMinAbsMpcZVAutoDnMps : -kMinAbsMpcZVAutoDnMps;
	}

	// Reconstruct the absolute target velocity in the local NED frame from the relative
	// velocity published by the VTE and the EKF2 vehicle velocity.
	const matrix::Vector2f target_velocity = target_rel_velocity + vehicle_velocity;
	const float min_prediction_time = math::min(_param_pld_moving_t_min.get(), _param_pld_moving_t_max.get());
	const float max_prediction_time = math::max(_param_pld_moving_t_min.get(), _param_pld_moving_t_max.get());
	float intersection_time_s = fabsf(_target_pose.z_rel / mpc_z_v_auto_dn);
	intersection_time_s = math::constrain(intersection_time_s, min_prediction_time, max_prediction_time);

	target_position_sp += target_velocity * intersection_time_s;
# endif // CONFIG_VTEST_MOVING
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	return target_position_sp;
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	switch (state) {
	case PrecLandState::Start:
		return true;

	case PrecLandState::HorizontalApproach:

		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach) {
			if (within_acceptance_radius()) {
				// we've reached the position where we last saw the target. If we don't see it now, we need to do something
				return target_acquired();

			} else {
				// We've seen the target sometime during horizontal approach.
				// Even if we don't see it as we're moving towards it, continue approaching last known location
				return true;
			}
		}

		// If we're trying to switch to this state, the target needs to be visible
		return target_acquired();

	case PrecLandState::DescendAboveTarget:

		// if we're already in this state, only leave it if target becomes unusable or if outside the drift radius
		if (_state == PrecLandState::DescendAboveTarget) {
			return target_acquired() && within_drift_radius();

		} else {
			// if not already in this state, need to be above target to enter it
			return target_acquired() && within_acceptance_radius();
		}

	case PrecLandState::FinalApproach:
		return target_acquired() && within_acceptance_radius()
		       && hrt_absolute_time() - _target_pose.timestamp < 500000 // 0.5s
		       && below_final_approach_alt();

	case PrecLandState::Search:
		return _search_cnt <= _param_pld_max_srch.get();

	case PrecLandState::Fallback:
		return true;

	default:
		return false;
	}
}

bool PrecLand::target_acquired()
{
	return _target_pose_valid && _target_pose.abs_pos_valid;
}

bool PrecLand::within_acceptance_radius()
{
	const matrix::Vector2f target_position_sp = get_target_position_setpoint();
	const vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	return fabsf(target_position_sp(0) - vehicle_local_position->x) < _param_pld_hacc_rad.get()
	       && fabsf(target_position_sp(1) - vehicle_local_position->y) < _param_pld_hacc_rad.get();
}

void PrecLand::update_ground_alt()
{
	const float current_alt = _navigator->get_global_position()->alt;
	const vehicle_local_position_s *lpos = _navigator->get_local_position();

	const float height_above_target = _target_pose.z_abs - lpos->z;
	const bool z_target_valid = target_acquired() && lpos->z_valid && PX4_ISFINITE(height_above_target)
				    && height_above_target > FLT_EPSILON;
	const bool z_dist_sensor_valid = lpos->dist_bottom_valid
					 && (lpos->dist_bottom_sensor_bitfield & vehicle_local_position_s::DIST_BOTTOM_SENSOR_RANGE)
					 && PX4_ISFINITE(lpos->dist_bottom);

	if (z_target_valid) {
		_ground_alt = current_alt - height_above_target;

	} else if (z_dist_sensor_valid) {
		_ground_alt = current_alt - lpos->dist_bottom;
	} // else keep already stored value
}

bool PrecLand::within_drift_radius()
{
	if (_param_pld_drift_rad.get() < FLT_EPSILON) {
		return true;
	}

	// Make sure drift radius is at least as large as acceptance radius
	const float drift_rad = math::max(_param_pld_drift_rad.get(), _param_pld_hacc_rad.get());

	const matrix::Vector2f target_position_sp = get_target_position_setpoint();
	const vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	return fabsf(target_position_sp(0) - vehicle_local_position->x) < drift_rad
	       && fabsf(target_position_sp(1) - vehicle_local_position->y) < drift_rad;
}

bool PrecLand::below_final_approach_alt()
{
	const float current_alt = _navigator->get_global_position()->alt;
	return current_alt < _ground_alt + _param_pld_fappr_alt.get();
}

void PrecLand::reset_to_landing_setpoint()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.lat = _landing_lat;
	pos_sp_triplet->current.lon = _landing_lon;
	pos_sp_triplet->current.alt = _landing_alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	clear_current_yaw_setpoint();
	_last_vertical_aid = VerticalAid::None;
	_navigator->set_position_setpoint_triplet_updated();
}

void PrecLand::aid_horizontally(VerticalAid vertical_aid, bool match_target_yaw, bool use_slewrate_limit)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	if (target_acquired()) {
		const matrix::Vector2f target_position_sp = get_target_position_setpoint();
		_map_ref.reproject(target_position_sp(0), target_position_sp(1), _landing_lat, _landing_lon);
	}

	if (use_slewrate_limit) {
		matrix::Vector2f sp = _map_ref.project(_landing_lat, _landing_lon);
		slewrate(sp(0), sp(1));
		_map_ref.reproject(sp(0), sp(1), pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

	} else {
		pos_sp_triplet->current.lat = _landing_lat;
		pos_sp_triplet->current.lon = _landing_lon;
	}

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	if (match_target_yaw) {
		update_current_yaw_setpoint();

	} else {
		clear_current_yaw_setpoint();
	}

#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	if (vertical_aid == VerticalAid::ClimbToSearch && get_mode() == PrecLandMode::Opportunistic) {
		vertical_aid = VerticalAid::None;
	}

	float current_alt = _navigator->get_global_position()->alt;
	float new_alt = current_alt;

	switch (vertical_aid) {
	case VerticalAid::None:
		pos_sp_triplet->current.alt = _landing_alt;
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
		break;

	case VerticalAid::HoldAlt:
		if (_last_vertical_aid != VerticalAid::HoldAlt) {
			_hold_alt = current_alt;

			if (_last_vertical_aid == VerticalAid::ClimbToSearch) {
				_hold_alt = math::min(current_alt + 1.0f, pos_sp_triplet->current.alt);
			}
		}

		pos_sp_triplet->current.alt = _hold_alt;
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		break;

	case VerticalAid::ClimbToSearch:
		new_alt = _ground_alt + _param_pld_srch_alt.get();
		pos_sp_triplet->current.alt = new_alt > current_alt ? new_alt : current_alt; // Never descend in SETPOINT_TYPE_POSITION
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		break;

	default:
		break;
	}

	_last_vertical_aid = vertical_aid;
	_navigator->set_position_setpoint_triplet_updated();
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
		_sp_pev = _map_ref.project(_navigator->get_position_setpoint_triplet()->current.lat,
					   _navigator->get_position_setpoint_triplet()->current.lon);
		_sp_pev_prev(0) = _sp_pev(0) - _navigator->get_local_position()->vx * dt;
		_sp_pev_prev(1) = _sp_pev(1) - _navigator->get_local_position()->vy * dt;
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

void PrecLand::clear_current_yaw_setpoint()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.yaw = NAN;
}

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
void PrecLand::update_current_yaw_setpoint()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.yaw = (_param_pld_yaw_en.get() && _target_yaw_valid) ? _target_yaw : NAN;
}

void PrecLand::reset_target_yaw_state()
{
	_target_yaw = 0.f;
	_target_yaw_valid = false;
	_last_target_yaw_update = 0;
}

uint8_t PrecLand::map_prec_land_state(PrecLandState state)
{
	switch (state) {
	case PrecLandState::Start: return prec_land_status_s::PREC_LAND_STATE_START;

	case PrecLandState::HorizontalApproach: return prec_land_status_s::PREC_LAND_STATE_HORIZONTAL;

	case PrecLandState::DescendAboveTarget: return prec_land_status_s::PREC_LAND_STATE_DESCEND;

	case PrecLandState::FinalApproach: return prec_land_status_s::PREC_LAND_STATE_FINAL;

	case PrecLandState::Search: return prec_land_status_s::PREC_LAND_STATE_SEARCH;

	case PrecLandState::Fallback: return prec_land_status_s::PREC_LAND_STATE_FALLBACK;

	case PrecLandState::Done: return prec_land_status_s::PREC_LAND_STATE_DONE;
	}

	return prec_land_status_s::PREC_LAND_STATE_STOPPED;
}

void PrecLand::publish_prec_land_status(uint8_t state)
{
	prec_land_status_s prec_land_status{};
	prec_land_status.timestamp = hrt_absolute_time();
	prec_land_status.state = state;
	_prec_land_status_pub.publish(prec_land_status);
}

#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
