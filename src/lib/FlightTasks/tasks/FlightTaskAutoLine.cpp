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

#define SIGMA_SINGLE_OP			0.000001f
#define SIGMA_NORM			0.001f

bool FlightTaskAutoLine::activate()
{
	bool ret = FlightTaskAuto::activate();
	_reset();
	return ret;
}

bool FlightTaskAutoLine::update()
{
	// always reset constraints because they might change depending on the type
	_setDefaultConstraints();

	_updateAltitudeAboveGround();

	bool follow_line = _type == WaypointType::loiter || _type == WaypointType::position;
	bool follow_line_prev = _type_previous == WaypointType::loiter || _type_previous == WaypointType::position;

	// 1st time that vehicle starts to follow line. Reset all setpoints to current vehicle state.
	if (follow_line && !follow_line_prev) {
		_reset();
	}

	// The only time a thrust set-point is sent out is during
	// idle. Hence, reset thrust set-point to NAN in case the
	// vehicle exits idle.

	if (_type_previous == WaypointType::idle) {
		_thrust_setpoint = Vector3f(NAN, NAN, NAN);
	}

	if (_type == WaypointType::idle) {
		_generateIdleSetpoints();

	} else if (_type == WaypointType::land) {
		_generateLandSetpoints();

	} else if (follow_line) {
		_generateSetpoints();

	} else if (_type == WaypointType::takeoff) {
		_generateTakeoffSetpoints();

	} else if (_type == WaypointType::velocity) {
		_generateVelocitySetpoints();
	}

	// update previous type
	_type_previous = _type;

	return true;
}

void FlightTaskAutoLine::_reset()
{
	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;
	_yaw_setpoint = _yaw;
	_destination = _position;
	_origin = _position;
	_speed_at_target = 0.0f;
}

void FlightTaskAutoLine::_generateIdleSetpoints()
{
	// Send zero thrust setpoint */
	_position_setpoint = Vector3f(NAN, NAN, NAN); // Don't require any position/velocity setpoints
	_velocity_setpoint = Vector3f(NAN, NAN, NAN);
	_thrust_setpoint.zero();
}

void FlightTaskAutoLine::_generateLandSetpoints()
{
	// Keep xy-position and go down with landspeed. */
	_position_setpoint = Vector3f(_target(0), _target(1), NAN);
	_velocity_setpoint = Vector3f(Vector3f(NAN, NAN, MPC_LAND_SPEED.get()));
	// set constraints
	_constraints.tilt = MPC_TILTMAX_LND.get();
	_constraints.speed_down = MPC_LAND_SPEED.get();
	_constraints.landing_gear = vehicle_constraints_s::GEAR_DOWN;
}

void FlightTaskAutoLine::_generateTakeoffSetpoints()
{
	// Takeoff is completely defined by target position. */
	_position_setpoint = _target;
	_velocity_setpoint = Vector3f(NAN, NAN, NAN);

	// limit vertical speed during takeoff
	_constraints.speed_up = math::gradual(_alt_above_ground, MPC_LAND_ALT2.get(),
					      MPC_LAND_ALT1.get(), MPC_TKO_SPEED.get(), _constraints.speed_up);

	_constraints.landing_gear = vehicle_constraints_s::GEAR_DOWN;

}

void FlightTaskAutoLine::_generateVelocitySetpoints()
{
	// TODO: Remove velocity force logic from navigator, since
	// navigator should only send out waypoints.
	_position_setpoint = Vector3f(NAN, NAN, _position(2));
	Vector2f vel_sp_xy = Vector2f(&_velocity(0)).unit_or_zero() * _mc_cruise_speed;
	_velocity_setpoint = Vector3f(vel_sp_xy(0), vel_sp_xy(1), NAN);
}

void FlightTaskAutoLine::_generateSetpoints()
{
	_updateInternalWaypoints();
	_generateAltitudeSetpoints();
	_generateXYsetpoints();

	// during mission and reposition, raise the landing gears but only
	// if altitude is high enough
	if (_highEnoughForLandingGear()) {
		_constraints.landing_gear = vehicle_constraints_s::GEAR_UP;
	}
}

void FlightTaskAutoLine::_updateInternalWaypoints()
{
	// The internal Waypoints might differ from previous_wp and target. The cases where it differs:
	// 1. The vehicle already passed the target -> go straight to target
	// 2. The vehicle is more than cruise speed in front of previous waypoint -> go straight to previous waypoint
	// 3. The vehicle is more than cruise speed from track -> go straight to closest point on track
	//
	// If a new target is available, then the speed at the target is computed from the angle previous-target-next.

	// Adjust destination and origin based on current vehicle state.
	Vector2f u_prev_to_target = Vector2f(&(_target - _prev_wp)(0)).unit_or_zero();
	Vector2f pos_to_target = Vector2f(&(_target - _position)(0));
	Vector2f prev_to_pos = Vector2f(&(_position - _prev_wp)(0));
	Vector2f closest_pt = Vector2f(&_prev_wp(0)) + u_prev_to_target * (prev_to_pos * u_prev_to_target);

	if (u_prev_to_target * pos_to_target < 0.0f) {

		// Target is behind. */
		if (_current_state != State::target_behind) {

			_destination = _target;
			_origin = _position;
			_current_state = State::target_behind;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0

			if (Vector2f(&(_destination - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_destination - _origin)(0)).length() > NAV_ACC_RAD.get())) {

				angle = Vector2f(&(_destination - _origin)(0)).unit_or_zero()
					* Vector2f(&(_destination - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}
		}

	} else if (u_prev_to_target * prev_to_pos < 0.0f && prev_to_pos.length() > _mc_cruise_speed) {

		// Current position is more than cruise speed in front of previous setpoint.
		if (_current_state != State::previous_infront) {
			_destination = _prev_wp;
			_origin = _position;
			_current_state = State::previous_infront;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0
			if (Vector2f(&(_destination - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_destination - _origin)(0)).length() > NAV_ACC_RAD.get())) {

				angle = Vector2f(&(_destination - _origin)(0)).unit_or_zero()
					* Vector2f(&(_destination - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}

		}

	} else if (Vector2f(Vector2f(&_position(0)) - closest_pt).length() > _mc_cruise_speed) {

		// Vehicle is more than cruise speed off track.
		if (_current_state != State::offtrack) {
			_destination = matrix::Vector3f(closest_pt(0), closest_pt(1), _target(2));
			_origin = _position;
			_current_state = State::offtrack;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0
			if (Vector2f(&(_destination - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_destination - _origin)(0)).length() > NAV_ACC_RAD.get())) {

				angle = Vector2f(&(_destination - _origin)(0)).unit_or_zero()
					* Vector2f(&(_destination - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}

		}

	} else {

		if ((_target - _destination).length() > 0.01f) {
			// A new target is available. Update speed at target.*/
			_destination = _target;
			_origin = _prev_wp;
			_current_state = State::none;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0
			if (Vector2f(&(_destination - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_destination - _origin)(0)).length() > NAV_ACC_RAD.get())) {

				angle =
					Vector2f(&(_destination - _origin)(0)).unit_or_zero()
					* Vector2f(&(_destination - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}
		}
	}
}

void FlightTaskAutoLine::_generateXYsetpoints()
{
	Vector2f pos_sp_to_dest = Vector2f(&(_destination - _position_setpoint)(0));
	const bool has_reached_altitude = fabsf(_destination(2) - _position(2)) < NAV_ACC_RAD.get();

	if ((_speed_at_target < 0.001f && pos_sp_to_dest.length() < NAV_ACC_RAD.get()) ||
	    (!has_reached_altitude && pos_sp_to_dest.length() < NAV_ACC_RAD.get())) {

		// Vehicle reached target in xy and no passing required. Lock position */
		_position_setpoint(0) = _destination(0);
		_position_setpoint(1) = _destination(1);
		_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;

	} else {

		// Get various path specific vectors. */
		Vector2f u_prev_to_dest = Vector2f(&(_destination - _origin)(0)).unit_or_zero();
		Vector2f prev_to_pos(&(_position - _origin)(0));
		Vector2f closest_pt = Vector2f(&_origin(0)) + u_prev_to_dest * (prev_to_pos * u_prev_to_dest);
		Vector2f closest_to_dest = Vector2f(&(_destination - _position)(0));
		Vector2f prev_to_dest = Vector2f(&(_destination - _origin)(0));
		float speed_sp_track = _mc_cruise_speed;
		float speed_sp_prev_track = math::max(Vector2f(&_velocity_setpoint(0)) * u_prev_to_dest, 0.0f);

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
		if (threshold_max > NAV_ACC_RAD.get()) {
			float m = (_mc_cruise_speed - _speed_at_target) / (threshold_max - NAV_ACC_RAD.get());
			speed_threshold = m * (target_threshold - NAV_ACC_RAD.get()) + _speed_at_target; // speed at transition
		}

		// Either accelerate or decelerate
		if (closest_to_dest.length() < target_threshold) {

			// Vehicle is close to destination. Start to decelerate

			if (!has_reached_altitude) {
				// Altitude is not reached yet. Vehicle has to stop first before proceeding
				_speed_at_target = 0.0f;
			}

			float acceptance_radius = NAV_ACC_RAD.get();

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
			float acc_max = (fabsf(yaw_diff) > math::radians(MIS_YAW_ERR.get())) ? 0.5f : MPC_ACC_HOR.get();

			if (acc_track > acc_max) {
				// accelerate towards target
				speed_sp_track = acc_max * _deltatime + speed_sp_prev_track;
			}
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
	const float dist = fabsf(_destination(2) - _origin(2));

	// If target has not been reached, then compute set-point depending on maximum velocity.
	if ((dist > SIGMA_NORM) && (fabsf(_position(2) - _destination(2)) > 0.1f)) {

		// get various distances */
		const float dist_to_prev = fabsf(_position(2) - _origin(2));
		const float dist_to_target = fabsf(_destination(2) - _position(2));

		// check sign
		const bool flying_upward = _destination(2) < _position(2);

		// limit vertical downwards speed (positive z) close to ground
		// for now we use the altitude above home and assume that we want to land at same height as we took off
		float vel_limit = math::gradual(_alt_above_ground,
						MPC_LAND_ALT2.get(), MPC_LAND_ALT1.get(),
						MPC_LAND_SPEED.get(), _constraints.speed_down);

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
			const float acc_max = (flying_upward) ? (MPC_ACC_UP_MAX.get() * 0.5f) : (MPC_ACC_DOWN_MAX.get() * 0.5f);

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
		_position_setpoint(2) = _destination(2);
	}
}

float FlightTaskAutoLine::_getVelocityFromAngle(const float angle)
{
	// minimum cruise speed when passing waypoint
	float min_cruise_speed = 0.0f;

	// make sure that cruise speed is larger than minimum
	if ((_mc_cruise_speed - min_cruise_speed) < SIGMA_NORM) {
		return _mc_cruise_speed;
	}

	// Middle cruise speed is a number between maximum cruising speed and minimum cruising speed and corresponds to speed at angle of 90degrees.
	// It needs to be always larger than minimum cruise speed.
	float middle_cruise_speed = MPC_CRUISE_90.get();

	if ((middle_cruise_speed - min_cruise_speed) < SIGMA_NORM) {
		middle_cruise_speed = min_cruise_speed + SIGMA_NORM;
	}

	if ((_mc_cruise_speed - middle_cruise_speed) < SIGMA_NORM) {
		middle_cruise_speed = (_mc_cruise_speed + min_cruise_speed) * 0.5f;
	}

	// If middle cruise speed is exactly in the middle, then compute speed linearly.
	bool use_linear_approach = false;

	if (((_mc_cruise_speed + min_cruise_speed) * 0.5f) - middle_cruise_speed < SIGMA_NORM) {
		use_linear_approach = true;
	}

	// compute speed sp at target
	float speed_close;

	if (use_linear_approach) {

		// velocity close to target adjusted to angle:
		// vel_close =  m*x+q
		float slope = -(_mc_cruise_speed - min_cruise_speed) / 2.0f;
		speed_close = slope * angle + _mc_cruise_speed;

	} else {

		// Speed close to target adjusted to angle x.
		// speed_close = a *b ^x + c; where at angle x = 0 -> speed_close = cruise; angle x = 1 -> speed_close = middle_cruise_speed (this means that at 90degrees
		// the velocity at target is middle_cruise_speed);
		// angle x = 2 -> speed_close = min_cruising_speed

		// from maximum cruise speed, minimum cruise speed and middle cruise speed compute constants a, b and c
		float a = -((middle_cruise_speed - _mc_cruise_speed) * (middle_cruise_speed - _mc_cruise_speed))
			  / (2.0f * middle_cruise_speed - _mc_cruise_speed - min_cruise_speed);
		float c = _mc_cruise_speed - a;
		float b = (middle_cruise_speed - c) / a;
		speed_close = a * powf(b, angle) + c;
	}

	// speed_close needs to be in between max and min
	return math::constrain(speed_close, min_cruise_speed, _mc_cruise_speed);
}

void FlightTaskAutoLine::_updateAltitudeAboveGround()
{
	// Altitude above ground is by default just the negation of the current local position in D-direction.
	_alt_above_ground = -_position(2);

	if (PX4_ISFINITE(_dist_to_bottom)) {
		// We have a valid distance to ground measurement
		_alt_above_ground = _dist_to_bottom;

	} else if (_sub_home_position->get().valid_alt) {
		// if home position is set, then altitude above ground is relative to the home position
		_alt_above_ground = -_position(2) + _sub_home_position->get().z;
	}
}

bool FlightTaskAutoLine::_highEnoughForLandingGear()
{
	// return true if altitude is above two meters
	return _alt_above_ground > 2.0f;
}

void FlightTaskAutoLine::updateParams()
{
	FlightTaskAuto::updateParams();

	// make sure that alt1 is above alt2
	MPC_LAND_ALT1.set(math::max(MPC_LAND_ALT1.get(), MPC_LAND_ALT2.get()));
}
