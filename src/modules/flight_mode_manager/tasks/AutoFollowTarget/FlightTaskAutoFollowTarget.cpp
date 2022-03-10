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

// Call Sticks constructor to set Follow Target class as a parent to Sticks object,
// which enables chained paramter update sequence, defined in ModuleParams
// to keep the parameters in Sticks up to date.
FlightTaskAutoFollowTarget::FlightTaskAutoFollowTarget() : _sticks(this)
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

	if (!PX4_ISFINITE(_position_setpoint(0)) || !PX4_ISFINITE(_position_setpoint(1))
	    || !PX4_ISFINITE(_position_setpoint(2))) {
		_position_setpoint = _position;
	}

	_target_pose_filter.reset(Vector3f{NAN, NAN, NAN});
	_target_pose_filter.setParameters(TARGET_POSE_FILTER_NATURAL_FREQUENCY, TARGET_POSE_FILTER_DAMPING_RATIO);

	// We don't command the yawspeed, since only the yaw can be calculated off of the target
	_yawspeed_setpoint = 0.f;
	_yaw_setpoint_filter.reset(NAN);

	// Update the internally tracaked Follow Target characteristics
	_follow_angle_rad = math::radians(get_follow_me_angle_setting(_param_nav_ft_fs.get()));
	_follow_target_distance = _param_nav_ft_dst.get();
	_follow_target_height = _param_nav_ft_ht.get();

	// Save the home position z value, to allow user so that altitude setpoint will be relative to arming position (home) altitude
	if (_sub_home_position.get().valid_alt) {
		_home_position_z = _sub_home_position.get().z;
	}

	return ret;
}

// Update the target pose filter, knowing that target estimator data is valid (checked in the update() main function).
void FlightTaskAutoFollowTarget::update_target_pose_filter(follow_target_estimator_s follow_target_estimator) {
	const Vector3f pos_ned_est{follow_target_estimator.pos_est};
	const Vector3f vel_ned_est{follow_target_estimator.vel_est};

	// Check Follow target estimator's validity & timeout conditions
	bool target_estimator_timeout = ((follow_target_estimator.timestamp - _last_valid_target_estimator_timestamp) > TARGET_ESTIMATOR_TIMEOUT_US);

	// Reset last valid estimator data received timestamp
	_last_valid_target_estimator_timestamp = follow_target_estimator.timestamp;

	// Handle Timetout cases
	if(target_estimator_timeout) {
		// Reset the Target pose filter if it's state is not finite
		if (!PX4_ISFINITE(_target_pose_filter.getState()(0)) || !PX4_ISFINITE(_target_pose_filter.getState()(1))
		|| !PX4_ISFINITE(_target_pose_filter.getState()(2)) || !PX4_ISFINITE(_target_pose_filter.getRate()(0)) ||
			!PX4_ISFINITE(_target_pose_filter.getRate()(1)) || !PX4_ISFINITE(_target_pose_filter.getRate()(2))) {
			_target_pose_filter.reset(pos_ned_est, vel_ned_est);
		}
	}
	else {
		// Second order target position filter to calculate kinematically feasible target position
		_target_pose_filter.update(_deltatime, pos_ned_est, vel_ned_est);
	}
}

// Updates Follow Target Height, Distance and Follow Angle depending on RC input commands
void FlightTaskAutoFollowTarget::update_stick_command() {
	// Update the sticks object to fetch recent data
	_sticks.checkAndUpdateStickInputs();

	// If no valid stick input is available, don't process any command
	if(!_sticks.isAvailable()) {
		return;
	}

	// Only apply Follow height adjustment if height setpoint and current height are within 1 second window
	if(fabsf(_position_setpoint(2) - _position(2)) < FOLLOW_HEIGHT_USER_ADJUST_SPEED) {
		// Throttle for changing follow height
		float height_change_speed = FOLLOW_HEIGHT_USER_ADJUST_SPEED * _sticks.getThrottle();
		float new_height = _follow_target_height + height_change_speed * _deltatime;
		_follow_target_height = constrain(new_height, MINIMUM_SAFETY_ALTITUDE, 100.f);
	}

	// Only apply Follow Angle adjustment if orbit angle setpoint and current orbit angle are within 1 second window
	if(fabsf(_measured_orbit_angle - _orbit_angle_setpoint) < FOLLOW_ANGLE_USER_ADJUST_SPEED) {
		// Roll for changing follow angle. When user commands +Roll (right), angle increases (clockwise)
		// Constrain adjust speed [rad/s], so that drone can actually catch up. Otherwise, the follow angle
		// command can be too ahead that it won't end up being responsive to the user.
		float angle_adjust_speed_max = min(FOLLOW_ANGLE_USER_ADJUST_SPEED, MAXIMUM_TANGENTIAL_ORBITING_SPEED / _follow_target_distance);
		float angle_change_speed = angle_adjust_speed_max * _sticks.getRoll();
		float new_angle = _follow_angle_rad + angle_change_speed * _deltatime;
		_follow_angle_rad = matrix::wrap_pi(new_angle);
	}

	// Only apply Follow distance adjustment if distance setting and current distance are within 1 second window
	if(fabsf(_drone_to_target_vector.length() - _follow_target_distance) < FOLLOW_DISTANCE_USER_ADJUST_SPEED) {
		// Pitch for changing distance
		float distance_change_speed = FOLLOW_DISTANCE_USER_ADJUST_SPEED * _sticks.getPitch();
		float new_distance = _follow_target_distance + distance_change_speed * _deltatime;
		_follow_target_distance = constrain(new_distance, MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL, 50.f);
	}
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

float FlightTaskAutoFollowTarget::update_orbit_angle(float target_orientation, float follow_angle) {
	// Raw target orbit (setpoint) angle
	const float raw_target_orbit_angle = matrix::wrap_pi(target_orientation + follow_angle);

	// Calculate orbit angle error & it's direction
	const float orbit_angle_error = matrix::wrap_pi(raw_target_orbit_angle - _orbit_angle_setpoint);
	const float orbit_angle_error_sign = matrix::sign(orbit_angle_error);

	// Calculate maximum orbital velocity vector perpendicular to current orbit angle setpoint
	const Vector2f orbital_max_velocity_vector = Vector2f(-sin(_orbit_angle_setpoint), cos(_orbit_angle_setpoint)) * orbit_angle_error_sign * MAXIMUM_TANGENTIAL_ORBITING_SPEED;

	// Calculate maximum orbital angle step we can take for this iteration
	float max_orbital_rate = MAXIMUM_TANGENTIAL_ORBITING_SPEED / _follow_target_distance;

	// Calculate maximum orbital angle step we can take [rad]
	const float max_orbital_step = max_orbital_rate * _deltatime;

	if(fabsf(orbit_angle_error) < max_orbital_step) {
		const float orbital_velocity_ratio = fabsf(orbit_angle_error) / max_orbital_step; // Current orbital step / Max orbital step, for velocity calculation
		_orbit_tangential_velocity = orbital_max_velocity_vector * orbital_velocity_ratio;
		return raw_target_orbit_angle; // Next orbital angle is feasible, set it directly
	}
	else {
		_orbit_tangential_velocity = orbital_max_velocity_vector;
		return matrix::wrap_pi(_orbit_angle_setpoint + orbit_angle_error_sign * max_orbital_step); // Take a step
	}
}

Vector3f FlightTaskAutoFollowTarget::calculate_desired_drone_position(Vector3f target_position)
{
	Vector3f drone_desired_position{NAN, NAN, NAN};

	// Correct the desired distance by the target scale determined from object detection
	const float desired_distance_to_target = _follow_target_distance;

	// Offset from the Target
	Vector2f offset_vector = Vector2f(cos(_orbit_angle_setpoint), sin(_orbit_angle_setpoint)) * desired_distance_to_target;

	// Calculate desired 2D position
	drone_desired_position.xy() = Vector2f(target_position.xy()) + offset_vector;

	// Calculate ground's z value in local frame for terrain tracking mode.
	// _dist_to_ground value takes care of distance sensor value if available, otherwise
	// it uses home z value as reference
	float ground_z_estimate = _position(2) + _dist_to_ground;

	// Z-position based off curent and initial target altitude
	switch (_param_nav_ft_alt_m.get()) {
		case FOLLOW_ALTITUDE_MODE_TRACK_TERRAIN:
			drone_desired_position(2) = ground_z_estimate - _follow_target_height;
			break;

		case FOLLOW_ALTITUDE_MODE_TRACK_TARGET:
			drone_desired_position(2) = target_position(2) - _follow_target_height;
			break;

		case FOLLOW_ALTITUDE_MODE_CONSTANT:

		// FALLTHROUGH

		default:
			// Calculate the desired Z position relative to the home position
			drone_desired_position(2) = _home_position_z - _follow_target_height;
	}

	return drone_desired_position;
}


float FlightTaskAutoFollowTarget::calculate_gimbal_height(float target_height) {
	float gimbal_height{0.0f};

	switch (_param_nav_ft_alt_m.get()) {
		case FOLLOW_ALTITUDE_MODE_TRACK_TERRAIN:
			// Point the gimbal at the ground level in this tracking mode
			gimbal_height = _dist_to_ground;
			break;

		case FOLLOW_ALTITUDE_MODE_TRACK_TARGET:
			// Point the gimbal at the target's 3D coordinates
			gimbal_height = -_position(2) - target_height;
			break;

		case FOLLOW_ALTITUDE_MODE_CONSTANT:

		//FALLTHROUGH

		default:
			gimbal_height = _home_position_z -_position(2); // Assume target is at home position's altitude
			break;
	}

	return gimbal_height;
}


bool FlightTaskAutoFollowTarget::update()
{
	_follow_target_estimator_sub.update(&_follow_target_estimator);

	follow_target_status_s follow_target_status{}; // Debugging uORB message for follow target status

	if (_follow_target_estimator.timestamp > 0 && _follow_target_estimator.valid) {
		// Update second order target pose filter, with a valid data
		update_target_pose_filter(_follow_target_estimator);

		const Vector3f target_position_filtered = _target_pose_filter.getState();
		const Vector3f target_velocity_filtered = _target_pose_filter.getRate();

		// Calculate offset 2D vector to target
		_drone_to_target_vector  = Vector2f(target_position_filtered.xy()) - Vector2f(_position.xy());
		// Calculate heading to the target (for Yaw setpoint)
		_drone_to_target_heading = atan2f(_drone_to_target_vector(1), _drone_to_target_vector(0));
		// Calculate current orbit angle around the target, taken into consideration in RC Follow Angle Adjustment
		_measured_orbit_angle = matrix::wrap_pi(_drone_to_target_heading + M_PI_F);

		// Update follow distance, angle and height via RC commands
		update_stick_command();

		// Calculate target orientation to track [rad]
		_target_orientation_rad = update_target_orientation(target_velocity_filtered.xy());

		// Update the new orbit angle (rate constrained)
		_orbit_angle_setpoint = update_orbit_angle(_target_orientation_rad, _follow_angle_rad);

		// Calculate desired position by applying orbit angle around the target
		Vector3f drone_desired_position = calculate_desired_drone_position(target_position_filtered);

		if (PX4_ISFINITE(drone_desired_position(0)) && PX4_ISFINITE(drone_desired_position(1))
		    && PX4_ISFINITE(drone_desired_position(2))) {
			// Only control horizontally if drone is on target altitude to avoid accidents
			if (fabsf(drone_desired_position(2) - _position(2)) < ALT_ACCEPTANCE_THRESHOLD) {
				Vector3f orbit_tangential_velocity_3d = Vector3f(_orbit_tangential_velocity(0), _orbit_tangential_velocity(1), 0.0f); // Zero velocity command for Z
				_velocity_setpoint = target_velocity_filtered + orbit_tangential_velocity_3d; // Target velocity + Orbit Tangential velocity
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

		// Update Yaw setpoint if we're far enough for yaw control
		if (_drone_to_target_vector.longerThan(MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL)) {
			// Check if yaw setpoint filtering is enabled
			if (YAW_SETPOINT_FILTER_ENABLE) {
				// If the filter hasn't been initialized yet, reset the state to raw heading value
				if (!PX4_ISFINITE(_yaw_setpoint_filter.getState())) {
					_yaw_setpoint_filter.reset(_drone_to_target_heading);
				}

				// Unwrap : Needed since when filter's tracked state is around -M_PI, and the raw angle goes to
				// +M_PI, the filter can just average them out and give wrong output.
				float yaw_setpoint_raw_unwrapped = matrix::unwrap_pi(_yaw_setpoint_filter.getState(), _drone_to_target_heading);

				// Set the parameters for the filter to take update time interval into account
				_yaw_setpoint_filter.setParameters(_deltatime, _param_ft_yaw_t.get());
				_yaw_setpoint_filter.update(yaw_setpoint_raw_unwrapped);

				// Wrap : keep the tracked filter state within [-M_PI, M_PI], to keep yaw setpoint filter's state from diverging.
				_yaw_setpoint_filter.reset(matrix::wrap_pi(_yaw_setpoint_filter.getState()));
				_yaw_setpoint = _yaw_setpoint_filter.getState();
			}
			else {
				// Yaw setpoint filtering disabled, set raw yaw setpoint
				_yaw_setpoint = _drone_to_target_heading;
			}
		}
		// Gimbal setpoint
		float gimbal_height = calculate_gimbal_height(target_position_filtered(2));
		point_gimbal_at(_drone_to_target_vector.norm(), gimbal_height);

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
	follow_target_status.emergency_ascent = _emergency_ascent;
	follow_target_status.gimbal_pitch = _gimbal_pitch;

	_follow_target_status_pub.publish(follow_target_status);

	_constraints.want_takeoff = _checkTakeoff();

	return true;
}

float FlightTaskAutoFollowTarget::get_follow_me_angle_setting(int param_nav_ft_fs) const
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
