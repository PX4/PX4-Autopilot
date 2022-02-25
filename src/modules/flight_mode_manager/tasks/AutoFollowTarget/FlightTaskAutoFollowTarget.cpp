/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoFollowTarget.cpp
 *
 * Flight Task for follow-me flight mode. It consumes follow_target_estimator messages from
 * TargetEstimator. The drone then tracks this target's GPS coordinates from a specified
 * angle and distance.
 *
 */

#include "FlightTaskAutoFollowTarget.hpp"
#include <mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Quatf;
using matrix::Eulerf;

FlightTaskAutoFollowTarget::FlightTaskAutoFollowTarget()
{
	_target_estimator.Start();
}

FlightTaskAutoFollowTarget::~FlightTaskAutoFollowTarget()
{
	_target_estimator.Stop();
}

bool FlightTaskAutoFollowTarget::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	_position_setpoint = _position;

	if (!PX4_ISFINITE(_position_setpoint(0)) || !PX4_ISFINITE(_position_setpoint(1))
	    || !PX4_ISFINITE(_position_setpoint(2))) {
		_position_setpoint = _position;
	}

	_target_position_filter.reset(Vector3f{NAN, NAN, NAN});
	_offset_vector_filter.reset(Vector2f(0, 0));
	_follow_angle_filter.reset(0.0f);
	_velocity_ff_scale.reset(0.0f);

	// Initialize to something such that the drone at least points at the target, even if it's the wrong angle for the perspective.
	// The drone will move into position as soon as the target starts moving and its heading becomes known.
	Vector2f current_drone_heading_2d{cosf(_yaw), -sinf(_yaw)};

	if (PX4_ISFINITE(current_drone_heading_2d(0)) && PX4_ISFINITE(current_drone_heading_2d(1))) {
		_offset_vector_filter.reset(current_drone_heading_2d);

	} else {
		_offset_vector_filter.reset(Vector2f(1.0f, 0.0f));
	}

	_yawspeed_setpoint = 0.f;

	return ret;
}

Vector2f FlightTaskAutoFollowTarget::calculate_offset_vector_filtered(Vector3f vel_ned_est)
{
	if (_param_nav_ft_fs.get() == FOLLOW_PERSPECTIVE_NONE) {
		// NOTE: Switching between NONE any any other setting currently causes a jump in the setpoints
		_offset_vector_filter.reset(Vector2f{0, 0});

	} else {
		// Define and rotate offset vector based on follow-me perspective setting
		const float new_follow_angle_rad = math::radians(update_follow_me_angle_setting(_param_nav_ft_fs.get()));

		// Use shortest rotation to get to the new angle
		// Example: if the current angle setting is 270, and the new angle setting is 0, it's
		// faster to rotate to 360 rather than 0.
		// Usually the controller would automatically take the shortest path. But here some trickery
		// is necessary because the yaw angle is run through a low-pass filter.
		if (_follow_angle_rad - new_follow_angle_rad  > M_PI_F) {
			_follow_angle_rad = new_follow_angle_rad + M_TWOPI_F;

		} else if (_follow_angle_rad - new_follow_angle_rad  < -M_PI_F) {
			_follow_angle_rad = new_follow_angle_rad - M_TWOPI_F;

		} else {
			_follow_angle_rad = new_follow_angle_rad;
		}

		// Lowpass the angle setting to smoothly transition to a new perspective when the user makes a change.
		// In particular this has an effect when the setting is modified by 180 degrees, in which case the drone
		// would pass above the target without the filter. The filtering makes it so that the drone flies around
		// the target into the new postion.
		_follow_angle_filter.setParameters(_deltatime, FOLLOW_ANGLE_FILTER_ALPHA);
		_follow_angle_filter.update(_follow_angle_rad);

		// Wrap around 360 degrees
		if (_follow_angle_filter.getState() > M_TWOPI_F) {
			_follow_angle_filter.reset(_follow_angle_filter.getState() - M_TWOPI_F);
			_follow_angle_rad = _follow_angle_rad - M_TWOPI_F;

		} else if (_follow_angle_filter.getState() < -M_TWOPI_F) {
			_follow_angle_filter.reset(_follow_angle_filter.getState() + M_TWOPI_F);
			_follow_angle_rad = _follow_angle_rad + M_TWOPI_F;
		}

		// Assume the target's velocity vector is its heading and use it to construct the offset vector
		// such that drone_pos_setpoint = target_pose + offset_vector
		if (vel_ned_est.longerThan(MINIMUM_SPEED_FOR_HEADING_CHANGE) &&
		    vel_ned_est.longerThan(FLT_EPSILON)) {
			// Compute offset vector relative to target position. At the same time the offset vector defines the
			// vieweing angle of the drone
			_target_velocity_unit_vector = Vector2f(vel_ned_est.xy()).unit_or_zero();
		}

		float offset_x = (float)cos(_follow_angle_filter.getState()) * _target_velocity_unit_vector(0) - (float)sin(
					 _follow_angle_filter.getState()) * _target_velocity_unit_vector(1);
		float offset_y = (float)sin(_follow_angle_filter.getState()) * _target_velocity_unit_vector(0) + (float)cos(
					 _follow_angle_filter.getState()) * _target_velocity_unit_vector(1);

		// Lowpass on the offset vector to have smooth transitions when the target turns, or when the
		// setting for the perspective is changed by the user. This introduces only a delay in the
		// tracking / viewing angle without disadvantages
		_offset_vector_filter.setParameters(_deltatime, DIRECTION_FILTER_ALPHA);
		_offset_vector_filter.update(Vector2f{offset_x, offset_y});
	}

	return _offset_vector_filter.getState();
}

Vector3f FlightTaskAutoFollowTarget::calculate_drone_desired_position(Vector3f target_position, Vector2f offset_vector)
{
	Vector3f drone_desired_position{NAN, NAN, NAN};

	// Correct the desired distance by the target scale determined from object detection
	const float desired_distance_to_target = _param_nav_ft_dst.get();
	drone_desired_position.xy() = Vector2f(target_position.xy()) +
				      offset_vector.unit_or_zero() * desired_distance_to_target;

	// Z-position based off curent and initial target altitude
	// TODO: Parameter NAV_MIN_FT_HT has been repurposed to be used as the desired
	// altitude above the target. Clarify best solution.
	switch (_param_nav_ft_alt_m.get()) {
	case FOLLOW_ALTITUDE_MODE_TRACK_TARGET:
		drone_desired_position(2) = target_position(2) - _param_nav_min_ft_ht.get();
		break;

	case FOLLOW_ALTITUDE_MODE_CONSTANT:

	// FALLTHROUGH

	default:
		// use the current position setpoint, unless it's closer to the ground than the minimum
		// altitude setting
		drone_desired_position(2) = math::min(_position_setpoint(2), -_param_nav_min_ft_ht.get());
	}

	return drone_desired_position;
}

Vector3f FlightTaskAutoFollowTarget::calculate_target_position_filtered(Vector3f pos_ned_est, Vector3f vel_ned_est,
		Vector3f acc_ned_est)
{
	// Reset the smoothness filter once the target position estimate is available
	if (!PX4_ISFINITE(_target_position_filter.getState()(0)) || !PX4_ISFINITE(_target_position_filter.getState()(1))
	    || !PX4_ISFINITE(_target_position_filter.getState()(2))) {
		_target_position_filter.reset(pos_ned_est);
	}

	// Low-pass filter on target position.
	_target_position_filter.setParameters(_deltatime, POSITION_FILTER_ALPHA);

	if (_param_nav_ft_delc.get() == 0) {
		_target_position_filter.update(pos_ned_est);

	} else {
		// Use a predicted target's position to compensate the filter delay to some extent.
		const Vector3f target_predicted_position = predict_future_pos_ned_est(POSITION_FILTER_ALPHA, pos_ned_est, vel_ned_est,
				acc_ned_est);
		_target_position_filter.update(target_predicted_position);
	}

	return _target_position_filter.getState();

}

bool FlightTaskAutoFollowTarget::update()
{
	_follow_target_estimator_sub.update(&_follow_target_estimator);
	follow_target_status_s follow_target_status{};

	if (_follow_target_estimator.timestamp > 0 && _follow_target_estimator.valid) {
		const Vector3f pos_ned_est{_follow_target_estimator.pos_est};
		const Vector3f vel_ned_est{_follow_target_estimator.vel_est};
		const Vector3f acc_ned_est{_follow_target_estimator.acc_est};
		const Vector3f target_position_filtered = calculate_target_position_filtered(pos_ned_est, vel_ned_est, acc_ned_est);
		const Vector2f offset_vector_filtered = calculate_offset_vector_filtered(vel_ned_est);
		const Vector3f drone_desired_position = calculate_drone_desired_position(target_position_filtered,
							offset_vector_filtered);

		// Set position and velocity setpoints
		float desired_velocity_ff_scale = 0.0f;  // Used to ramp up velocity feedforward, avoiding harsh jumps in the setpoints

		if (PX4_ISFINITE(drone_desired_position(0)) && PX4_ISFINITE(drone_desired_position(1))
		    && PX4_ISFINITE(drone_desired_position(2))) {
			// Only control horizontally if drone is on target altitude to avoid accidents
			if (fabsf(drone_desired_position(2) - _position(2)) < ALT_ACCEPTANCE_THRESHOLD) {

				// Only apply feed-forward velocity while the target is moving
				if (vel_ned_est.longerThan(MINIMUM_SPEED_FOR_HEADING_CHANGE)) {
					desired_velocity_ff_scale = 1.0f;

				}

				// Velocity setpoints is a feedforward term derived from position setpoints
				_velocity_setpoint = (drone_desired_position - _position_setpoint) / _deltatime * _velocity_ff_scale.getState();
				_position_setpoint = drone_desired_position;

			} else {
				// Achieve target altitude first before controlling horizontally!
				_position_setpoint = _position;
				_position_setpoint(2) = drone_desired_position(2);
			}

		} else {
			// Control setpoint: Stay in current position
			_position_setpoint = _position;
			_velocity_setpoint.setZero();
		}

		_velocity_ff_scale.setParameters(_deltatime, VELOCITY_FF_FILTER_ALPHA);
		_velocity_ff_scale.update(desired_velocity_ff_scale);

		// Emergency ascent when too close to the ground
		_emergency_ascent = PX4_ISFINITE(_dist_to_ground) && _dist_to_ground < MINIMUM_SAFETY_ALTITUDE;

		if (_emergency_ascent) {
			_position_setpoint(0) = _position_setpoint(1) = NAN;
			_position_setpoint(2) = _position(2);
			_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
			_velocity_setpoint(2) = -EMERGENCY_ASCENT_SPEED; // Slowly ascend
		}

		// Yaw setpoint: Face the target
		const Vector2f target_to_drone_xy = Vector2f(_position.xy()) - Vector2f(
				target_position_filtered.xy());

		if (target_to_drone_xy.longerThan(MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL)) {
			_yaw_setpoint = atan2f(-target_to_drone_xy(1), -target_to_drone_xy(0));
		}

		// Gimbal setpoint
		float gimbal_height = 0;

		switch (_param_nav_ft_alt_m.get()) {
		case FOLLOW_ALTITUDE_MODE_TRACK_TARGET:
			// Point the gimbal at the target's 3D coordinates
			gimbal_height = -(_position(2) - (target_position_filtered(2)));
			break;

		case FOLLOW_ALTITUDE_MODE_CONSTANT:
			// Point the gimbal at the ground level in this tracking mode
			gimbal_height = _dist_to_ground;

		// FALLTHROUGH
		default:
			break;
		}

		point_gimbal_at(target_to_drone_xy.norm(), gimbal_height);

	} else {
		// Control setpoint: Stay in current position
		_position_setpoint(0) = _position_setpoint(1) = NAN;
		_velocity_setpoint.xy() = 0;
	}

	// Publish status message for debugging
	_target_position_filter.getState().copyTo(follow_target_status.pos_est_filtered);
	follow_target_status.timestamp = hrt_absolute_time();
	follow_target_status.emergency_ascent = _emergency_ascent;
	_follow_target_status_pub.publish(follow_target_status);

	_constraints.want_takeoff = _checkTakeoff();

	return true;
}

float FlightTaskAutoFollowTarget::update_follow_me_angle_setting(int param_nav_ft_fs) const
{
	switch (param_nav_ft_fs) {
	case FOLLOW_PERSPECTIVE_BEHIND:
		return FOLLOW_PERSPECTIVE_BEHIND_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_FRONT:
		return FOLLOW_PERSPECTIVE_FRONT_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_FRONT_RIGHT:
		return FOLLOW_PERSPECTIVE_FRONT_RIGHT_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_FRONT_LEFT:
		return FOLLOW_PERSPECTIVE_FRONT_LEFT_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_MID_RIGHT:
		return FOLLOW_PERSPECTIVE_MID_RIGHT_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_MID_LEFT:
		return FOLLOW_PERSPECTIVE_MID_LEFT_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_BEHIND_RIGHT:
		return FOLLOW_PERSPECTIVE_BEHIND_RIGHT_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_BEHIND_LEFT:
		return FOLLOW_PERSPECTIVE_BEHIND_LEFT_ANGLE_DEG;

	case FOLLOW_PERSPECTIVE_MIDDLE_FOLLOW:
		return FOLLOW_PERSPECTIVE_BEHIND_ANGLE_DEG;

	default:
		// No or invalid option
		break;
	}

	// Default: follow from behind
	return FOLLOW_PERSPECTIVE_BEHIND_ANGLE_DEG;
}

Vector3f FlightTaskAutoFollowTarget::predict_future_pos_ned_est(float deltatime, const Vector3f &pos_ned_est,
		const Vector3f &vel_ned_est, const Vector3f &acc_ned_est) const
{
	return pos_ned_est + vel_ned_est * deltatime + 0.5f * acc_ned_est * deltatime * deltatime;
}


void FlightTaskAutoFollowTarget::point_gimbal_at(float xy_distance, float z_distance)
{
	gimbal_manager_set_attitude_s msg{};
	float pitch_down_angle = 0.0f;

	if (PX4_ISFINITE(z_distance)) {
		pitch_down_angle = atan2f(z_distance, xy_distance);
	}

	if (!PX4_ISFINITE(pitch_down_angle)) {
		pitch_down_angle = 0.0;
	}

	const Quatf q_gimbal = Quatf(Eulerf(0, -pitch_down_angle, 0));
	q_gimbal.copyTo(msg.q);

	msg.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(msg);
}
