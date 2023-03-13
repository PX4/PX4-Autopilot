/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file MulticopterLandDetector.cpp
 *
 *The MC land-detector goes through 3 states before it will detect landed:
 *
 *State 1 (=ground_contact):
 *ground_contact is detected once the vehicle is not moving along the NED-z direction and has
 *a thrust value below 0.3 of the thrust_range (thrust_hover - thrust_min). The condition has to be true
 *for GROUND_CONTACT_TRIGGER_TIME_US in order to detect ground_contact
 *
 *State 2 (=maybe_landed):
 *maybe_landed can only occur if the internal ground_contact hysteresis state is true. maybe_landed criteria requires to have no motion in x and y,
 *no rotation and a thrust below 0.1 of the thrust_range (thrust_hover - thrust_min). In addition, the mc_pos_control turns off the thrust_sp in
 *body frame along x and y which helps to detect maybe_landed. The criteria for maybe_landed needs to be true for (LNDMC_TRIG_TIME / 3) seconds.
 *
 *State 3 (=landed)
 *landed can only be detected if maybe_landed is true for LAND_DETECTOR_TRIGGER_TIME_US. No farther criteria is tested, but the mc_pos_control goes into
 *idle (thrust_sp = 0) which helps to detect landed. By doing this the thrust-criteria of State 2 will always be met, however the remaining criteria of no rotation and no motion still
 *have to be valid.

 *It is to note that if one criteria is not met, then vehicle exits the state directly without blocking.
 *
 *If the land-detector does not detect ground_contact, then the vehicle is either flying or falling, where free fall detection heavily relies
 *on the acceleration. TODO: verify that free fall is reliable
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include "MulticopterLandDetector.h"

using matrix::Vector2f;
using matrix::Vector3f;

namespace land_detector
{

MulticopterLandDetector::MulticopterLandDetector()
{
	_paramHandle.minManThrottle = param_find("MPC_MANTHR_MIN");
	_paramHandle.minThrottle = param_find("MPC_THR_MIN");
	_paramHandle.useHoverThrustEstimate = param_find("MPC_USE_HTE");
	_paramHandle.hoverThrottle = param_find("MPC_THR_HOVER");
	_paramHandle.landSpeed = param_find("MPC_LAND_SPEED");
	_paramHandle.crawlSpeed = param_find("MPC_LAND_CRWL");
	_minimum_thrust_8s_hysteresis.set_hysteresis_time_from(false, 8_s);
}

void MulticopterLandDetector::_update_topics()
{
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_vehicle_thrust_setpoint_throttle = -vehicle_thrust_setpoint.xyz[2];
	}

	vehicle_control_mode_s vehicle_control_mode;

	if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
		_flag_control_climb_rate_enabled = vehicle_control_mode.flag_control_climb_rate_enabled;
	}

	if (_params.useHoverThrustEstimate) {
		hover_thrust_estimate_s hte;

		if (_hover_thrust_estimate_sub.update(&hte)) {
			if (hte.valid) {
				_params.hoverThrottle = hte.hover_thrust;
				_hover_thrust_estimate_last_valid = hte.timestamp;
			}
		}
	}

	takeoff_status_s takeoff_status;

	if (_takeoff_status_sub.update(&takeoff_status)) {
		_takeoff_state = takeoff_status.takeoff_state;
	}
}

void MulticopterLandDetector::_update_params()
{
	param_get(_paramHandle.minThrottle, &_params.minThrottle);
	param_get(_paramHandle.minManThrottle, &_params.minManThrottle);
	param_get(_paramHandle.landSpeed, &_params.landSpeed);
	param_get(_paramHandle.crawlSpeed, &_params.crawlSpeed);

	// 1.2 corresponds to the margin between the default parameters LNDMC_Z_VEL_MAX = MPC_LAND_CRWL / 1.2
	const float lndmc_upper_threshold = math::min(_params.crawlSpeed, _params.landSpeed) / 1.2f;

	if (_param_lndmc_z_vel_max.get() > lndmc_upper_threshold) {
		PX4_ERR("LNDMC_Z_VEL_MAX > MPC_LAND_CRWL or MPC_LAND_SPEED, updating %.3f -> %.3f",
			(double)_param_lndmc_z_vel_max.get(), (double)(lndmc_upper_threshold));

		_param_lndmc_z_vel_max.set(lndmc_upper_threshold);
		_param_lndmc_z_vel_max.commit_no_notification();
	}

	int32_t use_hover_thrust_estimate = 0;
	param_get(_paramHandle.useHoverThrustEstimate, &use_hover_thrust_estimate);
	_params.useHoverThrustEstimate = (use_hover_thrust_estimate == 1);

	if (!_params.useHoverThrustEstimate || !_hover_thrust_initialized) {
		param_get(_paramHandle.hoverThrottle, &_params.hoverThrottle);

		// HTE runs based on the position controller so, even if we wish to use
		// the estimate, it is only available in altitude and position modes.
		// Therefore, we need to always initialize the hoverThrottle using the hover
		// thrust parameter in case we fly in stabilized
		// TODO: this can be removed once HTE runs in all modes
		_hover_thrust_initialized = true;
	}
}

bool MulticopterLandDetector::_get_freefall_state()
{
	// norm of specific force. Should be close to 9.8 m/s^2 when landed.
	return _acceleration.norm() < 2.f;
}

bool MulticopterLandDetector::_get_ground_contact_state()
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	const bool lpos_available = ((time_now_us - _vehicle_local_position.timestamp) < 1_s);

	if (lpos_available) {
		// Check if we are moving vertically.
		// Use wider threshold if currently in "maybe landed" state, as estimation for
		// vertical speed is often deteriorated when on the ground or due to propeller
		// up/down throttling.

		float vertical_velocity_threshold = _param_lndmc_z_vel_max.get();

		if (_landed_hysteresis.get_state()) {
			vertical_velocity_threshold *= 2.5f;
		}

		if (_vehicle_local_position.v_z_valid && (fabsf(_vehicle_local_position.vz) < vertical_velocity_threshold)) {
			_vertical_movement = false;

		} else if (_vehicle_local_position.z_valid && (fabsf(_vehicle_local_position.z_deriv) < vertical_velocity_threshold)) {
			// The Z derivative is often less accurate than VZ but is less affected by biased velocity measurements.
			_vertical_movement = false;

		} else {
			_vertical_movement = true;
		}

	} else {
		_vertical_movement = true;
	}


	// Check if we are moving horizontally.
	if (lpos_available && _vehicle_local_position.v_xy_valid) {
		const Vector2f v_xy{_vehicle_local_position.vx, _vehicle_local_position.vy};
		_horizontal_movement = v_xy.longerThan(_param_lndmc_xy_vel_max.get());

	} else {
		_horizontal_movement = false; // not known
	}

	if (lpos_available && _vehicle_local_position.dist_bottom_valid && _param_lndmc_alt_gnd_effect.get() > 0) {
		_below_gnd_effect_hgt = _vehicle_local_position.dist_bottom < _param_lndmc_alt_gnd_effect.get();

	} else {
		_below_gnd_effect_hgt = false;
	}

	const bool hover_thrust_estimate_valid = ((time_now_us - _hover_thrust_estimate_last_valid) < 1_s);

	if (!_in_descend || hover_thrust_estimate_valid) {
		// continue using valid hover thrust if it became invalid during descent
		_hover_thrust_estimate_valid = hover_thrust_estimate_valid;
	}

	// low thrust: 30% of throttle range between min and hover, relaxed to 60% if hover thrust estimate available
	const float thr_pct_hover = _hover_thrust_estimate_valid ? 0.6f : 0.3f;
	const float sys_low_throttle = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) * thr_pct_hover;
	_has_low_throttle = (_vehicle_thrust_setpoint_throttle <= sys_low_throttle);
	bool ground_contact = _has_low_throttle;

	// if we have a valid velocity setpoint and the vehicle is demanded to go down but no vertical movement present,
	// we then can assume that the vehicle hit ground
	if (_flag_control_climb_rate_enabled) {
		trajectory_setpoint_s trajectory_setpoint;

		if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
			// Setpoints can be NAN
			_in_descend = PX4_ISFINITE(trajectory_setpoint.velocity[2])
				      && (trajectory_setpoint.velocity[2] >= 1.1f * _param_lndmc_z_vel_max.get());
		}

		// ground contact requires commanded descent until landed
		if (!_maybe_landed_hysteresis.get_state() && !_landed_hysteresis.get_state()) {
			ground_contact &= _in_descend;
		}

	} else {
		_in_descend = false;
	}

	// if there is no distance to ground estimate available then don't enforce using it.
	// if a distance to the ground estimate is generally available (_dist_bottom_is_observable=true), then
	// we already increased the hysteresis for the land detection states in order to reduce the chance of false positives.
	const bool skip_close_to_ground_check = !_dist_bottom_is_observable || !_vehicle_local_position.dist_bottom_valid;
	_close_to_ground_or_skipped_check = _is_close_to_ground() || skip_close_to_ground_check;

	// TODO: we need an accelerometer based check for vertical movement for flying without GPS
	return !_armed ||
	       (_close_to_ground_or_skipped_check && ground_contact
		&& !_horizontal_movement && !_vertical_movement);
}

bool MulticopterLandDetector::_get_maybe_landed_state()
{
	hrt_abstime now = hrt_absolute_time();

	float minimum_thrust_threshold{0.f};

	if (_flag_control_climb_rate_enabled) {
		// 10% of throttle range between min and hover
		minimum_thrust_threshold = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) * 0.1f;

	} else {
		minimum_thrust_threshold = (_params.minManThrottle + 0.01f);
	}

	const bool minimum_thrust_now = _vehicle_thrust_setpoint_throttle <= minimum_thrust_threshold;
	_minimum_thrust_8s_hysteresis.set_state_and_update(minimum_thrust_now, now);

	// Next look if vehicle is not rotating (do not consider yaw)
	float max_rotation_threshold = math::radians(_param_lndmc_rot_max.get());

	// Widen max rotation thresholds if either in landed state, thus making it harder
	// to trigger a false positive !landed e.g. due to propeller throttling up/down.
	if (_landed_hysteresis.get_state()) {
		max_rotation_threshold *= 2.5f;
	}

	_rotational_movement = _angular_velocity.xy().norm() > max_rotation_threshold;

	// If vertical velocity is available: ground contact, no thrust, no movement -> landed
	const bool local_position_updated = (now - _vehicle_local_position.timestamp) < 1_s;
	const bool vertical_velocity_valid = _vehicle_local_position.v_z_valid;
	const bool vertical_estimate = local_position_updated && vertical_velocity_valid;

	return !_armed ||
	       (minimum_thrust_now && !_freefall_hysteresis.get_state() && !_rotational_movement
		&& ((vertical_estimate && _ground_contact_hysteresis.get_state())
		    || (!vertical_estimate && _minimum_thrust_8s_hysteresis.get_state())));
}

bool MulticopterLandDetector::_get_landed_state()
{
	// all maybe_landed conditions need to hold longer
	return !_armed || _maybe_landed_hysteresis.get_state();
}

bool MulticopterLandDetector::_get_ground_effect_state()
{
	return (_in_descend && !_horizontal_movement) ||
	       (_below_gnd_effect_hgt && _takeoff_state == takeoff_status_s::TAKEOFF_STATE_FLIGHT) ||
	       _takeoff_state == takeoff_status_s::TAKEOFF_STATE_RAMPUP;
}

bool MulticopterLandDetector::_is_close_to_ground()
{
	if (_vehicle_local_position.dist_bottom_valid) {
		return _vehicle_local_position.dist_bottom < DIST_FROM_GROUND_THRESHOLD;

	} else {
		return false;
	}
}

void MulticopterLandDetector::_set_hysteresis_factor(const int factor)
{
	_ground_contact_hysteresis.set_hysteresis_time_from(false, _param_lndmc_trig_time.get() * 1_s / 3 * factor);
	_landed_hysteresis.set_hysteresis_time_from(false, _param_lndmc_trig_time.get() * 1_s / 3 * factor);
	_maybe_landed_hysteresis.set_hysteresis_time_from(false, _param_lndmc_trig_time.get() * 1_s / 3 * factor);
	_freefall_hysteresis.set_hysteresis_time_from(false, FREEFALL_TRIGGER_TIME_US);
}

} // namespace land_detector
