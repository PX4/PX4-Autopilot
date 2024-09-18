/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RoverAckermannGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

RoverAckermannGuidance::RoverAckermannGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_ackermann_guidance_status_pub.advertise();
	updateParams();
	pid_init(&_pid_throttle, PID_MODE_DERIVATIV_NONE, 0.001f);
}

void RoverAckermannGuidance::updateParams()
{
	ModuleParams::updateParams();
	pid_set_parameters(&_pid_throttle,
			   _param_ra_p_speed.get(), // Proportional gain
			   _param_ra_i_speed.get(), // Integral gain
			   0, // Derivative gain
			   1, // Integral limit
			   1); // Output limit
}

RoverAckermannGuidance::motor_setpoint RoverAckermannGuidance::computeGuidance(const int nav_state)
{
	updateSubscriptions();

	// Distances to waypoints
	_distance_to_prev_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
			       _prev_wp(0), _prev_wp(1));
	_distance_to_curr_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
			       _curr_wp(0), _curr_wp(1));
	_distance_to_next_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
			       _next_wp(0), _next_wp(1));

	// Catch return to launch
	if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		_mission_finished = _distance_to_next_wp < _acceptance_radius;
	}

	// Guidance logic
	if (_mission_finished) { // Mission is finished
		_desired_steering = 0.f;
		_desired_speed = 0.f;

	} else if (_distance_to_curr_wp < _acceptance_radius) { // Catch delay command
		_desired_speed = 0.f;

	} else { // Regular guidance algorithm

		_desired_speed = calcDesiredSpeed(_wp_max_desired_vel, _param_ra_miss_vel_min.get(),
						  _param_ra_miss_vel_gain.get(), _distance_to_prev_wp, _distance_to_curr_wp, _acceptance_radius,
						  _prev_acceptance_radius, _param_ra_max_accel.get(), _param_ra_max_jerk.get(), nav_state);

		_desired_steering = calcDesiredSteering(_pure_pursuit, _curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
							_param_ra_wheel_base.get(), _desired_speed, _vehicle_yaw, _param_ra_max_steer_angle.get());

	}

	// Calculate throttle setpoint
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;
	const float throttle = calcThrottleSetpoint(_pid_throttle, _desired_speed, _actual_speed, _param_ra_max_speed.get(),
			       dt);

	// Publish ackermann controller status (logging)
	_rover_ackermann_guidance_status.timestamp = _timestamp;
	_rover_ackermann_guidance_status.desired_speed = _desired_speed;
	_rover_ackermann_guidance_status.pid_throttle_integral = _pid_throttle.integral;
	_rover_ackermann_guidance_status_pub.publish(_rover_ackermann_guidance_status);

	// Return motor setpoints
	motor_setpoint motor_setpoint_temp;
	motor_setpoint_temp.steering = math::interpolate<float>(_desired_steering, -_param_ra_max_steer_angle.get(),
				       _param_ra_max_steer_angle.get(), -1.f, 1.f); // Normalize steering setpoint
	motor_setpoint_temp.throttle = throttle;
	return motor_setpoint_temp;
}

void RoverAckermannGuidance::updateSubscriptions()
{
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);

		if (!_global_ned_proj_ref.isInitialized()
		    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(local_position.ref_lat, local_position.ref_lon, local_position.ref_timestamp);
		}

		_curr_pos_ned = Vector2f(local_position.x, local_position.y);
		const Vector3f rover_velocity = {local_position.vx, local_position.vy, local_position.vz};
		_actual_speed = rover_velocity.norm();
	}

	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypointsAndAcceptanceRadius();
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}
}

void RoverAckermannGuidance::updateWaypointsAndAcceptanceRadius()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

	// Global waypoint coordinates
	_prev_wp = _curr_pos.isAllFinite() ? _curr_pos : Vector2d(0, 0); // Fallback if previous waypoint is invalid
	_curr_wp = Vector2d(0, 0);
	_next_wp = _home_position.isAllFinite() ? _home_position : Vector2d(0, 0); // Enables corner slow down with RTL

	if (position_setpoint_triplet.current.valid && PX4_ISFINITE(position_setpoint_triplet.current.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.current.lon)) {
		_curr_wp = Vector2d(position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);

	}

	if (position_setpoint_triplet.previous.valid && PX4_ISFINITE(position_setpoint_triplet.previous.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.previous.lon)) {
		_prev_wp = Vector2d(position_setpoint_triplet.previous.lat, position_setpoint_triplet.previous.lon);

	}

	if (position_setpoint_triplet.next.valid && PX4_ISFINITE(position_setpoint_triplet.next.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.next.lon)) {
		_next_wp = Vector2d(position_setpoint_triplet.next.lat, position_setpoint_triplet.next.lon);

	}

	// NED waypoint coordinates
	_curr_wp_ned = _global_ned_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_ned = _global_ned_proj_ref.project(_prev_wp(0), _prev_wp(1));
	_next_wp_ned = _global_ned_proj_ref.project(_next_wp(0), _next_wp(1));

	// Update acceptance radius
	_prev_acceptance_radius = _acceptance_radius;

	if (_param_ra_acc_rad_max.get() >= _param_nav_acc_rad.get()) {
		_acceptance_radius = updateAcceptanceRadius(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, _param_nav_acc_rad.get(),
				     _param_ra_acc_rad_gain.get(), _param_ra_acc_rad_max.get(), _param_ra_wheel_base.get(), _param_ra_max_steer_angle.get());

	} else {
		_acceptance_radius = _param_nav_acc_rad.get();
	}

	if (position_setpoint_triplet.current.cruising_speed > 0.f) {
		_wp_max_desired_vel = math::constrain(position_setpoint_triplet.current.cruising_speed, 0.f, _param_ra_max_speed.get());

	} else {
		_wp_max_desired_vel = _param_ra_miss_vel_def.get();
	}
}

float RoverAckermannGuidance::updateAcceptanceRadius(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
		const Vector2f &next_wp_ned, const float default_acceptance_radius, const float acceptance_radius_gain,
		const float acceptance_radius_max, const float wheel_base, const float max_steer_angle)
{
	// Setup variables
	const Vector2f curr_to_prev_wp_ned = prev_wp_ned - curr_wp_ned;
	const Vector2f curr_to_next_wp_ned = next_wp_ned - curr_wp_ned;
	float acceptance_radius = default_acceptance_radius;

	// Calculate acceptance radius s.t. the rover cuts the corner tangential to the current and next line segment
	if (curr_to_next_wp_ned.norm() > FLT_EPSILON && curr_to_prev_wp_ned.norm() > FLT_EPSILON) {
		float cosin = curr_to_prev_wp_ned.unit_or_zero() * curr_to_next_wp_ned.unit_or_zero();
		cosin = math::constrain<float>(cosin, -1.f, 1.f); // Protect against float precision problem
		const float theta = acosf(cosin) / 2.f;
		const float min_turning_radius = wheel_base / sinf(max_steer_angle);
		const float acceptance_radius_temp = min_turning_radius / tanf(theta);
		const float acceptance_radius_temp_scaled = acceptance_radius_gain *
				acceptance_radius_temp; // Scale geometric ideal acceptance radius to account for kinematic and dynamic effects
		acceptance_radius = math::constrain<float>(acceptance_radius_temp_scaled, default_acceptance_radius,
				    acceptance_radius_max);
	}

	// Publish updated acceptance radius
	position_controller_status_s pos_ctrl_status{};
	pos_ctrl_status.acceptance_radius = acceptance_radius;
	pos_ctrl_status.timestamp = hrt_absolute_time();
	_position_controller_status_pub.publish(pos_ctrl_status);
	return acceptance_radius;
}

float RoverAckermannGuidance::calcDesiredSpeed(const float miss_vel_def, const float miss_vel_min,
		const float miss_vel_gain, const float distance_to_prev_wp, const float distance_to_curr_wp, const float acc_rad,
		const float prev_acc_rad, const float max_accel, const float max_jerk, const int nav_state)
{
	// Catch improper values
	if (miss_vel_min < 0.f  || miss_vel_min > miss_vel_def || miss_vel_gain < FLT_EPSILON) {
		return miss_vel_def;
	}

	// Cornering slow down effect
	if (distance_to_prev_wp <= prev_acc_rad && prev_acc_rad > FLT_EPSILON) {
		const float cornering_speed = miss_vel_gain / prev_acc_rad;
		return math::constrain(cornering_speed, miss_vel_min, miss_vel_def);

	} else if (distance_to_curr_wp <= acc_rad && acc_rad > FLT_EPSILON) {
		const float cornering_speed = miss_vel_gain / acc_rad;
		return math::constrain(cornering_speed, miss_vel_min, miss_vel_def);

	}

	// Straight line speed
	if (max_accel > FLT_EPSILON && max_jerk > FLT_EPSILON && acc_rad > FLT_EPSILON) {
		float max_velocity{0.f};

		if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
			max_velocity = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
					max_accel, distance_to_curr_wp, 0.f);

		} else {
			const float cornering_speed = miss_vel_gain / acc_rad;
			max_velocity = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
					max_accel, distance_to_curr_wp - acc_rad, cornering_speed);
		}

		return math::constrain(max_velocity, miss_vel_min, miss_vel_def);

	} else {
		return miss_vel_def;
	}

}

float RoverAckermannGuidance::calcDesiredSteering(PurePursuit &pure_pursuit, const Vector2f &curr_wp_ned,
		const Vector2f &prev_wp_ned, const Vector2f &curr_pos_ned, const float wheel_base, const float desired_speed,
		const float vehicle_yaw, const float max_steering)
{
	const float desired_heading = pure_pursuit.calcDesiredHeading(curr_wp_ned, prev_wp_ned, curr_pos_ned,
				      desired_speed);
	const float lookahead_distance = pure_pursuit.getLookaheadDistance();
	const float heading_error = matrix::wrap_pi(desired_heading - vehicle_yaw);
	// For logging
	_rover_ackermann_guidance_status.lookahead_distance = lookahead_distance;
	_rover_ackermann_guidance_status.heading_error = (heading_error * 180.f) / (M_PI_F);

	float desired_steering{0.f};

	if (math::abs_t(heading_error) <= M_PI_2_F) {
		desired_steering = atanf(2 * wheel_base * sinf(heading_error) / lookahead_distance);


	} else {
		desired_steering = atanf(2 * wheel_base * (sign(heading_error) * 1.0f + sinf(heading_error -
					 sign(heading_error) * M_PI_2_F)) / lookahead_distance);
	}

	return math::constrain(desired_steering, -max_steering, max_steering);

}

float RoverAckermannGuidance::calcThrottleSetpoint(PID_t &pid_throttle, const float desired_speed,
		const float actual_speed, const float max_speed, const float dt)
{
	float throttle = 0.f;

	if (desired_speed < FLT_EPSILON) {
		pid_reset_integral(&pid_throttle);

	} else {
		throttle = pid_calculate(&pid_throttle, desired_speed, actual_speed, 0, dt);
	}

	if (_param_ra_max_speed.get() > 0.f) { // Feed-forward term
		throttle += math::interpolate<float>(desired_speed, 0.f, max_speed, 0.f, 1.f);
	}

	return math::constrain(throttle, 0.f, 1.f);
}
