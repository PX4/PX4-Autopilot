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
 * @file FlightAutoLine.cpp
 */

#include "FlightTaskAutoLine.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

static constexpr float SIGMA_NORM		=	0.001f;

void FlightTaskAutoLine::_generateSetpoints()
{
	if (!PX4_ISFINITE(_yaw_setpoint)) {
		// no valid heading -> set heading along track
		_generateHeadingAlongTrack();
	}

	_generateAltitudeSetpoints();
	_generateXYsetpoints();
}

void FlightTaskAutoLine::_setSpeedAtTarget()
{
	// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0
	float angle = 2.0f; // minimum angle
	_speed_at_target = 0.0f;

	if (Vector2f(&(_target - _next_wp)(0)).length() > 0.001f &&
	    (Vector2f(&(_target - _prev_wp)(0)).length() > _target_acceptance_radius)) {
		// angle = cos(x) + 1.0
		angle =
			Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
			* Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
			+ 1.0f;
		_speed_at_target = math::expontialFromLimits(angle, 0.0f, _param_mpc_cruise_90.get(), _mc_cruise_speed);
	}
}


void FlightTaskAutoLine::_generateHeadingAlongTrack()
{
	Vector2f prev_to_dest(_target - _prev_wp);
	_compute_heading_from_2D_vector(_yaw_setpoint, prev_to_dest);

}

void FlightTaskAutoLine::_generateXYsetpoints()
{
	_setSpeedAtTarget();
	Vector2f pos_sp_to_dest(_target - _position_setpoint);
	const bool has_reached_altitude = fabsf(_target(2) - _position(2)) < _target_acceptance_radius;

	if ((_speed_at_target < 0.001f && pos_sp_to_dest.length() < _target_acceptance_radius) ||
	    (!has_reached_altitude && pos_sp_to_dest.length() < _target_acceptance_radius)) {

		// Vehicle reached target in xy and no passing required. Lock position
		_position_setpoint(0) = _target(0);
		_position_setpoint(1) = _target(1);
		_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;

	} else {
		// Vehicle needs to pass waypoint
		// Ensure that vehicle never gets stuck because of too low target-speed
		_speed_at_target = math::max(_speed_at_target, 0.5f);

		// Get various path specific vectors. */
		Vector2f u_prev_to_dest = Vector2f(_target - _prev_wp).unit_or_zero();
		Vector2f prev_to_pos(_position - _prev_wp);
		Vector2f closest_pt = Vector2f(_prev_wp) + u_prev_to_dest * (prev_to_pos * u_prev_to_dest);
		Vector2f closest_to_dest(_target - _position);
		Vector2f prev_to_dest(_target - _prev_wp);
		float speed_sp_track = _mc_cruise_speed;
		float speed_sp_prev_track = math::max(Vector2f(_velocity_setpoint) * u_prev_to_dest, 0.0f);

		// Distance to target when brake should occur. The assumption is made that
		// 1.5 * cruising speed is enough to break.
		float target_threshold = 1.5f * _mc_cruise_speed;
		float speed_threshold = _mc_cruise_speed;
		const float threshold_max = target_threshold;

		if (target_threshold > 0.5f * prev_to_dest.length()) {
			// Target threshold cannot be more than distance from previous to target
			target_threshold = 0.5f * prev_to_dest.length();
		}

		// Compute maximum speed at target threshold */
		if (threshold_max > _target_acceptance_radius) {
			float m = (_mc_cruise_speed - _speed_at_target) / (threshold_max - _target_acceptance_radius);
			speed_threshold = m * (target_threshold - _target_acceptance_radius) + _speed_at_target; // speed at transition
		}

		// Either accelerate or decelerate
		if (closest_to_dest.length() < target_threshold) {

			// Vehicle is close to destination. Start to decelerate

			if (!has_reached_altitude) {
				// Altitude is not reached yet. Vehicle has to stop first before proceeding
				_speed_at_target = 0.0f;
			}

			float acceptance_radius = _target_acceptance_radius;

			if (_speed_at_target < 0.01f) {
				// If vehicle wants to stop at the target, then set acceptance radius to zero as well.
				acceptance_radius = 0.0f;
			}

			if ((target_threshold - acceptance_radius) >= SIGMA_NORM) {

				// Slow down depending on distance to target minus acceptance radius.
				float m = (speed_threshold - _speed_at_target) / (target_threshold - acceptance_radius);
				speed_sp_track = m * (closest_to_dest.length() - acceptance_radius) + _speed_at_target; // speed at transition

			} else {
				speed_sp_track = _speed_at_target;
			}

			// If we are close to target and the previous speed setpoint along track was smaller than
			// current speed setpoint along track, then take over the previous one.
			// This ensures smoothness since we anyway want to slow down.
			if ((speed_sp_prev_track < speed_sp_track)
			    && (speed_sp_track * speed_sp_prev_track > 0.0f)
			    && (speed_sp_prev_track > _speed_at_target)) {
				speed_sp_track = speed_sp_prev_track;
			}

		} else {

			// Vehicle is still far from destination. Accelerate or keep maximum target speed.
			float acc_track = (speed_sp_track - speed_sp_prev_track) / _deltatime;

			float yaw_diff = 0.0f;

			if (PX4_ISFINITE(_yaw_setpoint)) {
				yaw_diff = wrap_pi(_yaw_setpoint - _yaw);
			}

			// If yaw offset is large, only accelerate with 0.5 m/s^2.
			float acc_max = (fabsf(yaw_diff) > math::radians(_param_mis_yaw_err.get())) ? 0.5f : _param_mpc_acc_hor.get();

			if (acc_track > acc_max) {
				// accelerate towards target
				speed_sp_track = acc_max * _deltatime + speed_sp_prev_track;
			}

			// ensure that desired speed does not exceed speed at threshold
			speed_sp_track = math::min(speed_threshold, speed_sp_track);
		}

		speed_sp_track = math::constrain(speed_sp_track, 0.0f, _mc_cruise_speed);

		_position_setpoint(0) = closest_pt(0);
		_position_setpoint(1) = closest_pt(1);
		Vector2f velocity_sp_xy = u_prev_to_dest * speed_sp_track;
		_velocity_setpoint(0) =  velocity_sp_xy(0);
		_velocity_setpoint(1) =  velocity_sp_xy(1);
	}
}

void FlightTaskAutoLine::_generateAltitudeSetpoints()
{
	// Total distance between previous and target set-point.
	const float dist = fabsf(_target(2) - _prev_wp(2));

	// If target has not been reached, then compute set-point depending on maximum velocity.
	if ((dist > SIGMA_NORM) && (fabsf(_position(2) - _target(2)) > 0.1f)) {

		// get various distances */
		const float dist_to_prev = fabsf(_position(2) - _prev_wp(2));
		const float dist_to_target = fabsf(_target(2) - _position(2));

		// check sign
		const bool flying_upward = _target(2) < _position(2);

		// limit vertical downwards speed (positive z) close to ground
		// for now we use the altitude above home and assume that we want to land at same height as we took off
		float vel_limit = math::gradual(_alt_above_ground,
						_param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
						_param_mpc_land_speed.get(), _constraints.speed_down);

		// Speed at threshold is by default maximum speed. Threshold defines
		// the point in z at which vehicle slows down to reach target altitude.
		float speed_sp = (flying_upward) ? _constraints.speed_up : vel_limit;

		// Target threshold defines the distance to target(2) at which
		// the vehicle starts to slow down to approach the target smoothly.
		float target_threshold = speed_sp * 1.5f;

		// If the total distance in z is NOT 2x distance of target_threshold, we
		// will need to adjust the final_velocity in z.
		const bool is_2_target_threshold = dist >= 2.0f * target_threshold;
		const float min_vel = 0.2f; // minimum velocity: this is needed since estimation is not perfect
		const float slope = (speed_sp - min_vel) / (target_threshold); // defines the the acceleration when slowing down */

		if (!is_2_target_threshold) {
			// Adjust final_velocity since we are already are close to target and therefore it is not necessary to accelerate
			// upwards with full speed.
			target_threshold = dist * 0.5f;
			// get the velocity at target_threshold
			speed_sp = slope * (target_threshold) + min_vel;
		}

		// we want to slow down
		if (dist_to_target < target_threshold) {

			speed_sp = slope * dist_to_target + min_vel;

		} else if (dist_to_prev < target_threshold) {
			// we want to accelerate

			const float acc = (speed_sp - fabsf(_velocity_setpoint(2))) / _deltatime;
			const float acc_max = (flying_upward) ? (_param_mpc_acc_up_max.get() * 0.5f) : (_param_mpc_acc_down_max.get() * 0.5f);

			if (acc > acc_max) {
				speed_sp = acc_max * _deltatime + fabsf(_velocity_setpoint(2));
			}
		}

		// make sure vel_sp_z is always positive
		if (speed_sp < 0.0f) {
			PX4_WARN("speed cannot be smaller than 0");
			speed_sp = 0.0f;
		}

		// get the sign of vel_sp_z
		_velocity_setpoint(2) = (flying_upward) ? -speed_sp : speed_sp;
		_position_setpoint(2) = NAN; // We don't care about position setpoint

	} else {

		// vehicle reached desired target altitude
		_velocity_setpoint(2) = 0.0f;
		_position_setpoint(2) = _target(2);
	}
}
