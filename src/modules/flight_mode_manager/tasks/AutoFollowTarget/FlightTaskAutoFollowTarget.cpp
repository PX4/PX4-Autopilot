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
	release_gimbal_control();
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

	_target_pose_filter.reset(Vector3f{NAN, NAN, NAN});
	_target_pose_filter.setParameters(TARGET_POSE_FILTER_NATURAL_FREQUENCY, TARGET_POSE_FILTER_DAMPING_RATIO);

	_yaw_setpoint_filter.reset(NAN);

	// We don't command the yawspeed, since only the yaw can be calculated off of the target
	_yawspeed_setpoint = 0.f;

	return ret;
}

void FlightTaskAutoFollowTarget::update_target_pose_filter(follow_target_estimator_s follow_target_estimator) {
	bool target_estimator_timeout = (follow_target_estimator.timestamp - _last_target_estimator_timestamp) > TARGET_ESTIMATOR_TIMEOUT_SECONDS;

	const Vector3f pos_ned_est{follow_target_estimator.pos_est};
	const Vector3f vel_ned_est{follow_target_estimator.vel_est};

	// Reset the Target pose filter if it's state is not finite or estimator has timed out
	if (target_estimator_timeout || !PX4_ISFINITE(_target_pose_filter.getState()(0)) || !PX4_ISFINITE(_target_pose_filter.getState()(1))
	|| !PX4_ISFINITE(_target_pose_filter.getState()(2)) || !PX4_ISFINITE(_target_pose_filter.getRate()(0)) ||
		!PX4_ISFINITE(_target_pose_filter.getRate()(1)) || !PX4_ISFINITE(_target_pose_filter.getRate()(2))) {
		_target_pose_filter.reset(pos_ned_est, vel_ned_est);
		_last_target_estimator_timestamp = follow_target_estimator.timestamp; // Reset last estimator received timestamp
	}

	// Second order target position filter to calculate kinematically feasible target position
	_target_pose_filter.update(_deltatime, pos_ned_est, vel_ned_est);
}

float FlightTaskAutoFollowTarget::update_target_orientation(Vector2f target_velocity) {

	const float target_velocity_norm = target_velocity.norm(); // Get 2D projected target speed [m/s]

	// Depending on the target velocity, freeze or set new target orientatino value
	if(target_velocity_norm >= TARGET_VELOCITY_DEADZONE_FOR_ORIENTATION_TRACKING) {
		_target_orientation_rad = atan2f(target_velocity(1), target_velocity(0));
	}
	else {
		_target_orientation_rad = _target_orientation_rad;
	}
	return _target_orientation_rad;
}

float FlightTaskAutoFollowTarget::update_orbit_angle(float target_orientation, float follow_angle, float max_orbital_rate) {
	// Raw target orbit (setpoint) angle
	const float raw_target_orbit_angle = matrix::wrap_pi(target_orientation + follow_angle);

	// Calculate orbit angle error
	const float orbit_angle_error = matrix::wrap_pi(raw_target_orbit_angle - _current_orbit_angle);

	// Calculate maximum orbital angle step we can take for this iteration
	const float max_orbital_step = max_orbital_rate * _deltatime;

	if(fabsf(orbit_angle_error) < max_orbital_step) {
		return raw_target_orbit_angle; // Next orbital angle is feasible, set it directly
	}
	else {
		return matrix::wrap_pi(_current_orbit_angle + matrix::sign(orbit_angle_error) * max_orbital_step); // Take a step
	}
}

Vector3f FlightTaskAutoFollowTarget::calculate_drone_desired_position(Vector3f target_position)
{
	Vector3f drone_desired_position{NAN, NAN, NAN};

	// Correct the desired distance by the target scale determined from object detection
	const float desired_distance_to_target = _param_nav_ft_dst.get();

	// Offset from the Target
	Vector2f offset_vector = Vector2f(cos(_current_orbit_angle), sin(_current_orbit_angle)) * desired_distance_to_target;

	// Calculate desired 2D position
	drone_desired_position.xy() = Vector2f(target_position.xy()) + offset_vector;


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


bool FlightTaskAutoFollowTarget::update()
{
	_follow_target_estimator_sub.update(&_follow_target_estimator);

	follow_target_status_s follow_target_status{}; // Debugging uORB message for follow target status

	if (_follow_target_estimator.timestamp > 0 && _follow_target_estimator.valid) {
		// Update second order target pose filter
		update_target_pose_filter(_follow_target_estimator);

		const Vector3f target_position_filtered = _target_pose_filter.getState();
		const Vector3f target_velocity_filtered = _target_pose_filter.getRate();

		// Calculate maximum orbital angular rate, depending on the follow distance
		const float max_orbit_rate = MAXIMUM_TANGENTIAL_ORBITING_SPEED / _param_nav_ft_dst.get();

		// Calculate target orientation to track [rad]
		_target_orientation_rad = update_target_orientation(target_velocity_filtered.xy());

		// Get the follow angle from the parameter
		const float new_follow_angle_rad = math::radians(update_follow_me_angle_setting(_param_nav_ft_fs.get()));
		_follow_angle_rad = new_follow_angle_rad; // Save the follow angle internally

		// Update the new orbit angle (rate constrained)
		_current_orbit_angle = update_orbit_angle(_target_orientation_rad, new_follow_angle_rad, max_orbit_rate);

		// Calculate desired position by applying orbit angle around the target
		Vector3f drone_desired_position = calculate_drone_desired_position(target_position_filtered);

		if (PX4_ISFINITE(drone_desired_position(0)) && PX4_ISFINITE(drone_desired_position(1))
		    && PX4_ISFINITE(drone_desired_position(2))) {
			// Only control horizontally if drone is on target altitude to avoid accidents
			if (fabsf(drone_desired_position(2) - _position(2)) < ALT_ACCEPTANCE_THRESHOLD) {
				_velocity_setpoint = target_velocity_filtered; // Follow target velocity directly
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

		// Emergency ascent when too close to the ground
		_emergency_ascent = PX4_ISFINITE(_dist_to_ground) && _dist_to_ground < MINIMUM_SAFETY_ALTITUDE;

		if (_emergency_ascent) {
			_position_setpoint(0) = _position_setpoint(1) = NAN;
			_position_setpoint(2) = _position(2);
			_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
			_velocity_setpoint(2) = -EMERGENCY_ASCENT_SPEED; // Slowly ascend
		}

		// Yaw Setpoint : Calculate offset 2D vector to target
		const Vector2f drone_to_target_xy  = Vector2f(target_position_filtered.xy()) - Vector2f(
				_position.xy());

		// Update Yaw setpoint if we're far enough for yaw control
		if (drone_to_target_xy.longerThan(MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL)) {
			float yaw_setpoint_raw = atan2f(drone_to_target_xy(1), drone_to_target_xy(0));
			_drone_to_target_heading = yaw_setpoint_raw;

			// Check if yaw setpoint filtering is enabled
			if (_param_nav_ft_yaw_ft.get()) {
				// If the filter hasn't been initialized yet, reset the state to raw heading value
				if (!PX4_ISFINITE(_yaw_setpoint_filter.getState())) {
					_yaw_setpoint_filter.reset(yaw_setpoint_raw);
				}

				// Unwrap : Needed since when filter's tracked state is around -M_PI, and the raw angle goes to
				// +M_PI, the filter can just average them out and give wrong output.
				float yaw_setpoint_raw_unwrapped = matrix::unwrap(_yaw_setpoint_filter.getState(), yaw_setpoint_raw);

				// Set the parameters for the filter to take update time interval into account
				_yaw_setpoint_filter.setParameters(_deltatime, YAW_SETPOINT_FILTER_ALPHA);
				_yaw_setpoint_filter.update(yaw_setpoint_raw_unwrapped);

				// Wrap : keep the tracked filter state within [-M_PI, M_PI], to keep yaw setpoint filter's state from diverging.
				_yaw_setpoint_filter.reset(matrix::wrap_pi(_yaw_setpoint_filter.getState()));
				_yaw_setpoint = _yaw_setpoint_filter.getState();
			}
			else {
				// Yaw setpoint filtering disabled, set raw yaw setpoint
				_yaw_setpoint = yaw_setpoint_raw;
			}


		}

		// Gimbal setpoint
		float gimbal_height = 0;

		switch (_param_nav_ft_gmb_m.get()) {
		case FOLLOW_GIMBAL_MODE_2D:
			gimbal_height = -_position(2);
			break;

		case FOLLOW_GIMBAL_MODE_3D:
			// Point the gimbal at the target's 3D coordinates
			gimbal_height = -(_position(2) - (target_position_filtered(2)));
			break;

		case FOLLOW_GIMBAL_MODE_2D_WITH_TERRAIN:
			// Point the gimbal at the ground level in this tracking mode
			gimbal_height = _dist_to_ground;
			break;

		default:
			break;
		}

		point_gimbal_at(drone_to_target_xy.norm(), gimbal_height);

	} else {
		// Control setpoint: Stay in current position
		_position_setpoint(0) = _position_setpoint(1) = NAN;
		_velocity_setpoint.xy() = 0;
	}

	// Publish status message for debugging
	follow_target_status.timestamp = hrt_absolute_time();
	_target_pose_filter.getState().copyTo(follow_target_status.pos_est_filtered);
	_target_pose_filter.getRate().copyTo(follow_target_status.vel_est_filtered);

	follow_target_status.tracked_target_orientation = _target_orientation_rad;
	follow_target_status.follow_angle = _follow_angle_rad;
	follow_target_status.current_orbit_angle = _current_orbit_angle;

	follow_target_status.drone_to_target_heading = _drone_to_target_heading; // debug

	follow_target_status.emergency_ascent = _emergency_ascent;
	follow_target_status.gimbal_pitch = _gimbal_pitch;

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

void FlightTaskAutoFollowTarget::release_gimbal_control()
{
	// NOTE: If other flight tasks start using gimbal control as well
	// it might be worth moving this release mechanism to a common base
	// class for gimbal-control flight tasks

	vehicle_command_s vehicle_command = {};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vehicle_command.param1 = -3.0f; // Remove control if it had it.
	vehicle_command.param2 = -3.0f; // Remove control if it had it.
	vehicle_command.param3 = -1.0f; // Leave unchanged.
	vehicle_command.param4 = -1.0f; // Leave unchanged.

	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.source_system = _param_mav_sys_id.get();
	vehicle_command.source_component = _param_mav_comp_id.get();
	vehicle_command.target_system = _param_mav_sys_id.get();
	vehicle_command.target_component = _param_mav_comp_id.get();
	vehicle_command.confirmation = false;
	vehicle_command.from_external = false;

	_vehicle_command_pub.publish(vehicle_command);
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
	_gimbal_pitch = pitch_down_angle; // For logging

	q_gimbal.copyTo(msg.q);

	msg.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(msg);
}
