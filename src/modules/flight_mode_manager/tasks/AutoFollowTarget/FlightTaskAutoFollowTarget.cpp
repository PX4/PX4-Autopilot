/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
#include <float.h>

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

	_target_position_velocity_filter.reset(Vector3f{NAN, NAN, NAN});
	_target_position_velocity_filter.setParameters(TARGET_POS_VEL_FILTER_NATURAL_FREQUENCY, TARGET_POS_VEL_FILTER_DAMPING_RATIO);

	// We don't command the yawspeed, since only the yaw can be calculated off of the target
	_yawspeed_setpoint = NAN;
	_yaw_setpoint_filter.reset(NAN);

	// Update the internally tracked Follow Target characteristics
	_follow_angle_rad = math::radians(get_follow_angle_setting_deg((FollowPerspective)_param_flw_tgt_fs.get()));
	_follow_distance = _param_flw_tgt_dst.get();
	_follow_height = _param_flw_tgt_ht.get();

	// Reset the orbit angle trajectory generator and set maximum angular acceleration and rate
	_orbit_angle_traj_generator.reset(0.f, 0.f, 0.f);
	// Set Orbit Angle limit to float maximum value, to allow tracking of unwrapped orbit angle.
	_orbit_angle_traj_generator.setMaxVel(FLT_MAX);

	// Save the home position z value to enable relative altitude setpoints to arming position (home) altitude
	if (_sub_home_position.get().valid_alt) {
		_home_position_z = _sub_home_position.get().z;
	}
	else {
		ret = false; // Don't activate Follow Target task if home position is not valid
	}

	return ret;
}

void FlightTaskAutoFollowTarget::updateParams() {
	// Copy previous param values to check if it changes after param update
	const float follow_distance_prev = _param_flw_tgt_dst.get();
	const float follow_height_prev = _param_flw_tgt_ht.get();
	const int follow_side_prev = _param_flw_tgt_fs.get();

	// Call updateParams in parent class to update parameters from child up until ModuleParam base class
	FlightTask::updateParams();

	// Compare param values and if they have changed, update the properties
	if (matrix::isEqualF(follow_distance_prev, _param_flw_tgt_dst.get())) {
		_follow_distance = _param_flw_tgt_dst.get();
	}
	if (matrix::isEqualF(follow_height_prev, _param_flw_tgt_ht.get())) {
		_follow_height = _param_flw_tgt_ht.get();
	}
	if (follow_side_prev != _param_flw_tgt_fs.get()) {
		_follow_angle_rad = math::radians(get_follow_angle_setting_deg((FollowPerspective)_param_flw_tgt_fs.get()));
	}
}

void FlightTaskAutoFollowTarget::update_target_position_velocity_filter(const follow_target_estimator_s &follow_target_estimator) {
	const Vector3f pos_ned_est{follow_target_estimator.pos_est};
	const Vector3f vel_ned_est{follow_target_estimator.vel_est};

	// Check Follow target estimator's validity & timeout conditions
	const bool target_estimator_timeout = ((follow_target_estimator.timestamp - _last_valid_target_estimator_timestamp) > TARGET_ESTIMATOR_TIMEOUT_US);

	// Reset last valid estimator data received timestamp
	_last_valid_target_estimator_timestamp = follow_target_estimator.timestamp;

	// Reset the filter if any pose filter states are not finite or estimator timed out
	if ((!PX4_ISFINITE(_target_position_velocity_filter.getState()(0)) || !PX4_ISFINITE(_target_position_velocity_filter.getState()(1))
		|| !PX4_ISFINITE(_target_position_velocity_filter.getState()(2)) || !PX4_ISFINITE(_target_position_velocity_filter.getRate()(0))
		|| !PX4_ISFINITE(_target_position_velocity_filter.getRate()(1)) || !PX4_ISFINITE(_target_position_velocity_filter.getRate()(2))) || target_estimator_timeout) {
		_target_position_velocity_filter.reset(pos_ned_est, vel_ned_est);
	}
	else {
		_target_position_velocity_filter.update(_deltatime, pos_ned_est, vel_ned_est);
	}
}

void FlightTaskAutoFollowTarget::update_rc_adjusted_follow_height(const Sticks &sticks) {
	if (fabsf(sticks.getThrottleZeroCentered()) < USER_ADJUSTMENT_DEADZONE) {
		return;
	}
	// Only apply Follow height adjustment if height setpoint and current height are within time window
	if(fabsf(_position_setpoint(2) - _position(2)) < FOLLOW_HEIGHT_USER_ADJUST_SPEED * USER_ADJUSTMENT_ERROR_TIME_WINDOW) {
		// RC Throttle stick input for changing follow height
		const float height_change_speed = FOLLOW_HEIGHT_USER_ADJUST_SPEED * sticks.getThrottleZeroCentered();
		const float new_height = _follow_height + height_change_speed * _deltatime;
		_follow_height = constrain(new_height, MINIMUM_SAFETY_ALTITUDE, FOLLOW_HEIGHT_MAX);
	}
}

void FlightTaskAutoFollowTarget::update_rc_adjusted_follow_distance(const Sticks &sticks, const Vector2f &drone_to_target_vector) {
	if (fabsf(sticks.getPitch()) < USER_ADJUSTMENT_DEADZONE) {
		return;
	}
	// Only apply Follow distance adjustment if distance setting and current distance are within time window
	if(fabsf(drone_to_target_vector.length() - _follow_distance) < FOLLOW_DISTANCE_USER_ADJUST_SPEED * USER_ADJUSTMENT_ERROR_TIME_WINDOW) {
		// RC Pitch stick input for changing distance
		const float distance_change_speed = FOLLOW_DISTANCE_USER_ADJUST_SPEED * sticks.getPitch();
		const float new_distance = _follow_distance + distance_change_speed * _deltatime;
		_follow_distance = constrain(new_distance, MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL, FOLLOW_DISTANCE_MAX);
	}
}

void FlightTaskAutoFollowTarget::update_rc_adjusted_follow_angle(const Sticks &sticks, const float measured_orbit_angle, const float tracked_orbit_angle_setpoint) {
	if (fabsf(sticks.getRoll()) < USER_ADJUSTMENT_DEADZONE) {
		return;
	}
	// Only apply Follow Angle adjustment if orbit angle setpoint and current orbit angle are within time window
	// Wrap orbit angle difference, to get the shortest angle between them
	if(fabsf(matrix::wrap_pi(measured_orbit_angle - tracked_orbit_angle_setpoint)) < FOLLOW_ANGLE_USER_ADJUST_SPEED * USER_ADJUSTMENT_ERROR_TIME_WINDOW) {
		// RC Roll stick input for changing follow angle. When user commands RC stick input: +Roll (right), angle increases (clockwise)
		// Constrain adjust speed [rad/s], so that drone can actually catch up. Otherwise, the follow angle
		// command can be too ahead that it won't end up being responsive to RC stick inputs.
		const float angle_adjust_speed_max = min(FOLLOW_ANGLE_USER_ADJUST_SPEED, _param_mpc_xy_vel_max.get() / _follow_distance);
		const float angle_change_speed = angle_adjust_speed_max * sticks.getRoll();
		const float new_angle = _follow_angle_rad + angle_change_speed * _deltatime;
		_follow_angle_rad = matrix::wrap_pi(new_angle);
	}
}

void FlightTaskAutoFollowTarget::update_target_orientation(const Vector2f &target_velocity, float &current_target_orientation)
{
	const float target_velocity_norm = target_velocity.norm(); // Get 2D projected target speed [m/s]

	// Depending on the target velocity, freeze or set new target orientatino value
	if(target_velocity_norm >= TARGET_SPEED_DEADZONE_FOR_ORIENTATION_TRACKING) {
		current_target_orientation = atan2f(target_velocity(1), target_velocity(0));
	}
}

float FlightTaskAutoFollowTarget::update_orbit_angle_trajectory(const float target_orientation, const float previous_orbit_angle_setpoint)
{
	// Raw target orbit (setpoint) angle, unwrapped to be relative to the previous orbit angle setpoint
	const float unwrapped_target_orbit_angle = matrix::unwrap_pi(previous_orbit_angle_setpoint, target_orientation + _follow_angle_rad);

	// Calculate limits for orbit angular acceleration and velocity rate
	_orbit_angle_traj_generator.setMaxJerk(_param_flw_tgt_max_acc.get() / _follow_distance);
	_orbit_angle_traj_generator.setMaxAccel(_param_flw_tgt_max_vel.get() / _follow_distance);

	// Calculate trajectory towards the unwrapped target orbit angle
	_orbit_angle_traj_generator.updateDurations(unwrapped_target_orbit_angle);
	_orbit_angle_traj_generator.updateTraj(_deltatime);

	// Get the calculaed angle setpoint
	const float unwrapped_orbit_angle_setpoint = _orbit_angle_traj_generator.getCurrentVelocity();

	return unwrapped_orbit_angle_setpoint;
}

Vector2f FlightTaskAutoFollowTarget::get_orbit_tangential_velocity(const float orbit_angle_setpoint) const
{
	const float angular_rate_setpoint = _orbit_angle_traj_generator.getCurrentAcceleration();
	// Calculate Tangential velocity setpoint vector
	return Vector2f(-sinf(orbit_angle_setpoint), cosf(orbit_angle_setpoint)) * angular_rate_setpoint * _follow_distance;
}

Vector3f FlightTaskAutoFollowTarget::calculate_desired_drone_position(const Vector3f &target_position, const float orbit_angle_setpoint) const
{
	Vector3f drone_desired_position{NAN, NAN, NAN};

	// Offset from the Target
	const Vector2f offset_vector = Vector2f(cosf(orbit_angle_setpoint), sinf(orbit_angle_setpoint)) * _follow_distance;

	// Calculate desired 2D position
	drone_desired_position.xy() = Vector2f(target_position.xy()) + offset_vector;

	// Calculate ground's z value in local frame for terrain tracking mode.
	// _dist_to_ground value takes care of distance sensor value if available, otherwise
	// it uses home z value as reference
	const float ground_z_estimate = _position(2) + _dist_to_ground;

	// Z-position based off curent and initial target altitude
	switch (_param_flw_tgt_alt_m.get()) {
		case FOLLOW_ALTITUDE_MODE_TRACK_TERRAIN:
			drone_desired_position(2) = ground_z_estimate - _follow_height;
			break;

		case FOLLOW_ALTITUDE_MODE_TRACK_TARGET:
			drone_desired_position(2) = target_position(2) - _follow_height;
			break;

		case FOLLOW_ALTITUDE_MODE_CONSTANT:

		// FALLTHROUGH

		default:
			// Calculate the desired Z position relative to the home position
			drone_desired_position(2) = _home_position_z - _follow_height;
	}

	return drone_desired_position;
}


float FlightTaskAutoFollowTarget::calculate_gimbal_height(const FollowAltitudeMode altitude_mode, const float target_pos_z) const
{
	float gimbal_height{0.0f};

	switch (altitude_mode) {
		case FOLLOW_ALTITUDE_MODE_TRACK_TERRAIN:
			// Point the gimbal at the ground level in this tracking mode
			gimbal_height = _dist_to_ground;
			break;

		case FOLLOW_ALTITUDE_MODE_TRACK_TARGET:
			// Point the gimbal at the target's 3D coordinates
			gimbal_height = -(_position(2) - target_pos_z);
			break;

		case FOLLOW_ALTITUDE_MODE_CONSTANT:

		//FALLTHROUGH

		default:
			gimbal_height = _home_position_z - _position(2); // Assume target is at home position's altitude
			break;
	}

	return gimbal_height;
}


bool FlightTaskAutoFollowTarget::update()
{
	// Debugging uORB message for follow target status
	follow_target_status_s follow_target_status{};
	// Members of the uORB message that gets set below
	bool emergency_ascent{false};
	float gimbal_pitch{NAN};

	// Get the latest target estimator message for target position and velocity
	_follow_target_estimator_sub.update(&_follow_target_estimator);

	if (_follow_target_estimator.timestamp > 0 && _follow_target_estimator.valid) {
		// Update second order target pose filter, with a valid data
		update_target_position_velocity_filter(_follow_target_estimator);

		const Vector3f target_position_filtered = _target_position_velocity_filter.getState();
		const Vector3f target_velocity_filtered = _target_position_velocity_filter.getRate();

		// Calculate offset 2D vector to target
		const Vector2f drone_to_target_vector  = Vector2f(target_position_filtered.xy()) - Vector2f(_position.xy());
		// Calculate heading to the target (for Yaw setpoint)
		const float drone_to_target_heading = atan2f(drone_to_target_vector(1), drone_to_target_vector(0));
		// Actual orbit angle measured around the target, which is pointing from target to drone, so M_PI_F difference.
		const float measured_orbit_angle = matrix::wrap_pi(drone_to_target_heading + M_PI_F);

		// Update the sticks object to fetch recent data
		_sticks.checkAndUpdateStickInputs();
		// If valid stick input is available, update follow distance, angle and height via RC commands
		if(_sticks.isAvailable()) {
			update_rc_adjusted_follow_height(_sticks);
			update_rc_adjusted_follow_distance(_sticks, drone_to_target_vector);
			update_rc_adjusted_follow_angle(_sticks, measured_orbit_angle, _orbit_angle_setpoint_rad);
		}

		// Update target orientation to track
		update_target_orientation(target_velocity_filtered.xy(), _target_course_rad);

		// Update the new orbit angle (rate constrained)
		_orbit_angle_setpoint_rad = update_orbit_angle_trajectory(_target_course_rad, _orbit_angle_setpoint_rad);

		// Orbital tangential velocity setpoint from the generated trajectory
		const Vector2f orbit_tangential_velocity = get_orbit_tangential_velocity(_orbit_angle_setpoint_rad);

		// Calculate desired position by applying orbit angle around the target
		const Vector3f drone_desired_position = calculate_desired_drone_position(target_position_filtered, _orbit_angle_setpoint_rad);

		if (PX4_ISFINITE(drone_desired_position(0)) && PX4_ISFINITE(drone_desired_position(1))
		    && PX4_ISFINITE(drone_desired_position(2))) {
			// Only control horizontally if drone is on target altitude to avoid accidents
			if (fabsf(drone_desired_position(2) - _position(2)) < ALT_ACCEPTANCE_THRESHOLD) {
				Vector3f orbit_tangential_velocity_3d = Vector3f(orbit_tangential_velocity(0), orbit_tangential_velocity(1), 0.0f); // Zero velocity command for Z
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
		emergency_ascent = PX4_ISFINITE(_dist_to_ground) && _dist_to_ground < MINIMUM_SAFETY_ALTITUDE;

		if (emergency_ascent) {
			_position_setpoint(0) = _position_setpoint(1) = NAN;
			_position_setpoint(2) = _position(2);
			_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
			_velocity_setpoint(2) = -EMERGENCY_ASCENT_SPEED; // Slowly ascend
		}

		// Update Yaw setpoint if we're far enough for yaw control
		if (drone_to_target_vector.longerThan(MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL)) {
			// If the filter hasn't been initialized yet, reset the state to raw heading value
			if (!PX4_ISFINITE(_yaw_setpoint_filter.getState())) {
				_yaw_setpoint_filter.reset(drone_to_target_heading);
			}

			// Unwrap : Needed since when filter's tracked state is around -M_PI, and the raw angle goes to
			// +M_PI, the filter can just average them out and give wrong output.
			const float yaw_setpoint_raw_unwrapped = matrix::unwrap_pi(_yaw_setpoint_filter.getState(), drone_to_target_heading);

			// Set the parameters for the filter to take update time interval into account
			_yaw_setpoint_filter.setParameters(_deltatime, _param_flw_tgt_yaw_t.get());
			_yaw_setpoint_filter.update(yaw_setpoint_raw_unwrapped);

			// Wrap : keep the tracked filter state within [-M_PI, M_PI], to keep yaw setpoint filter's state from diverging.
			_yaw_setpoint_filter.reset(matrix::wrap_pi(_yaw_setpoint_filter.getState()));
			_yaw_setpoint = _yaw_setpoint_filter.getState();
		}
		// Calculate Gimbal setpoint to track target in the center of the view
		const float gimbal_height = calculate_gimbal_height((FollowAltitudeMode)_param_flw_tgt_alt_m.get(), target_position_filtered(2));
		gimbal_pitch = point_gimbal_at(drone_to_target_vector.norm(), gimbal_height);

	} else {
		// Control setpoint: Stay in current position
		_position_setpoint(0) = _position_setpoint(1) = NAN;
		_velocity_setpoint.xy() = 0;
	}

	// Publish status message for debugging
	follow_target_status.timestamp = hrt_absolute_time();
	_target_position_velocity_filter.getState().copyTo(follow_target_status.pos_est_filtered);
	_target_position_velocity_filter.getRate().copyTo(follow_target_status.vel_est_filtered);

	follow_target_status.tracked_target_course = _target_course_rad;
	follow_target_status.follow_angle = _follow_angle_rad;
	follow_target_status.orbit_angle_setpoint = _orbit_angle_setpoint_rad;

	follow_target_status.emergency_ascent = emergency_ascent; // Log emergency ascent state
	follow_target_status.gimbal_pitch = gimbal_pitch; // Log gimbal pitch

	_follow_target_status_pub.publish(follow_target_status);

	_constraints.want_takeoff = _checkTakeoff();

	return true;
}

float FlightTaskAutoFollowTarget::get_follow_angle_setting_deg(const FollowPerspective follow_perspective) const
{
	switch (follow_perspective) {
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
	q_gimbal.copyTo(msg.q);

	msg.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(msg);

	return pitch_down_angle;
}
