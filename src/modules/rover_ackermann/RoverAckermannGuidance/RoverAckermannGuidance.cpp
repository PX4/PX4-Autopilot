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

RoverAckermannGuidance::motor_setpoint RoverAckermannGuidance::purePursuit()
{
	// Initializations
	float desired_speed{0.f};
	float desired_steering{0.f};
	float vehicle_yaw{0.f};
	float actual_speed{0.f};

	// uORB subscriber updates
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);

		if (!_global_local_proj_ref.isInitialized()
		    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != local_position.ref_timestamp)) {
			_global_local_proj_ref.initReference(local_position.ref_lat, local_position.ref_lon, local_position.ref_timestamp);
		}

		_curr_pos_local = Vector2f(local_position.x, local_position.y);
		const Vector3f rover_velocity = {local_position.vx, local_position.vy, local_position.vz};
		actual_speed = rover_velocity.norm();
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypoints();
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = Quatf(vehicle_attitude.q);
		vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}

	// Guidance logic
	if (!_mission_finished) {
		// Calculate desired speed
		const float distance_to_prev_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
						  _prev_wp(0),
						  _prev_wp(1));

		if (distance_to_prev_wp <= _prev_acc_rad) { // Cornering speed
			const float cornering_speed = _param_ra_miss_vel_gain.get() / _prev_acc_rad;
			desired_speed = math::constrain(cornering_speed, _param_ra_miss_vel_min.get(), _param_ra_miss_vel_def.get());

		} else { // Default mission speed
			desired_speed = _param_ra_miss_vel_def.get();
		}

		// Calculate desired steering
		desired_steering = calcDesiredSteering(_curr_wp_local, _prev_wp_local, _curr_pos_local, _param_ra_lookahd_gain.get(),
						       _param_ra_lookahd_min.get(), _param_ra_lookahd_max.get(), _param_ra_wheel_base.get(), desired_speed, vehicle_yaw);
		desired_steering = math::constrain(desired_steering, -_param_ra_max_steer_angle.get(),
						   _param_ra_max_steer_angle.get());
	}

	// Throttle PID
	hrt_abstime now = hrt_absolute_time();
	const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e6f;
	_time_stamp_last = now;
	float throttle = 0.f;

	if (desired_speed < FLT_EPSILON) {
		pid_reset_integral(&_pid_throttle);

	} else {
		throttle = pid_calculate(&_pid_throttle, desired_speed, actual_speed, 0,
					 dt);
	}

	if (_param_ra_max_speed.get() > 0.f) { // Feed-forward term
		throttle += math::interpolate<float>(desired_speed,
						     0.f, _param_ra_max_speed.get(),
						     0.f, 1.f);
	}

	// Publish ackermann controller status (logging)
	_rover_ackermann_guidance_status.timestamp = now;
	_rover_ackermann_guidance_status.actual_speed = actual_speed;
	_rover_ackermann_guidance_status.desired_speed = desired_speed;
	_rover_ackermann_guidance_status.pid_throttle_integral = _pid_throttle.integral;
	_rover_ackermann_guidance_status_pub.publish(_rover_ackermann_guidance_status);

	// Return motor setpoints
	motor_setpoint motor_setpoint_temp;
	motor_setpoint_temp.steering = math::interpolate<float>(desired_steering, -_param_ra_max_steer_angle.get(),
				       _param_ra_max_steer_angle.get(),
				       -1.f, 1.f);
	motor_setpoint_temp.throttle = math::constrain(throttle, 0.f, 1.f);
	return motor_setpoint_temp;
}

void RoverAckermannGuidance::updateWaypoints()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

	// Global waypoint coordinates
	_curr_wp = Vector2d(position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);

	if (position_setpoint_triplet.previous.valid) {
		_prev_wp = Vector2d(position_setpoint_triplet.previous.lat, position_setpoint_triplet.previous.lon);

	} else {
		_prev_wp = _curr_pos;
	}

	if (position_setpoint_triplet.next.valid) {
		_next_wp = Vector2d(position_setpoint_triplet.next.lat, position_setpoint_triplet.next.lon);

	} else {
		_next_wp = _curr_wp;
	}

	// Local waypoint coordinates
	_curr_wp_local = _global_local_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_local = _global_local_proj_ref.project(_prev_wp(0), _prev_wp(1));
	_next_wp_local = _global_local_proj_ref.project(_next_wp(0), _next_wp(1));

	// Update acceptance radius
	_prev_acc_rad = _acceptance_radius;
	_acceptance_radius = updateAcceptanceRadius(_curr_wp_local, _prev_wp_local, _next_wp_local, _param_ra_acc_rad_def.get(),
			     _param_ra_acc_rad_gain.get(), _param_ra_acc_rad_max.get(), _param_ra_wheel_base.get(), _param_ra_max_steer_angle.get());
}

float RoverAckermannGuidance::updateAcceptanceRadius(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
		const Vector2f &next_wp_local, const float &default_acceptance_radius, const float &acceptance_radius_gain,
		const float &acceptance_radius_max, const float &wheel_base, const float &max_steer_angle)
{
	// Setup variables
	const Vector2f curr_to_prev_wp_local = prev_wp_local - curr_wp_local;
	const Vector2f curr_to_next_wp_local = next_wp_local - curr_wp_local;
	float acceptance_radius = default_acceptance_radius;

	// Calculate acceptance radius s.t. the rover cuts the corner tangential to the current and next line segment
	if (curr_to_next_wp_local.norm() > FLT_EPSILON && curr_to_prev_wp_local.norm() > FLT_EPSILON) {
		const float theta = acosf((curr_to_prev_wp_local * curr_to_next_wp_local) / (curr_to_prev_wp_local.norm() *
					  curr_to_next_wp_local.norm())) / 2.f;
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

float RoverAckermannGuidance::calcDesiredSteering(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
		const Vector2f &curr_pos_local, const float &lookahead_gain, const float &lookahead_min, const float &lookahead_max,
		const float &wheel_base, const float &desired_speed, const float &vehicle_yaw)
{
	// Calculate crosstrack error
	const Vector2f prev_wp_to_curr_wp_local = curr_wp_local - prev_wp_local;

	if (prev_wp_to_curr_wp_local.norm() < FLT_EPSILON) { // Avoid division by 0 (this case should not happen)
		return 0.f;
	}

	const Vector2f prev_wp_to_curr_pos_local = curr_pos_local - prev_wp_local;
	const Vector2f distance_on_line_segment = ((prev_wp_to_curr_pos_local * prev_wp_to_curr_wp_local) /
			prev_wp_to_curr_wp_local.norm()) * prev_wp_to_curr_wp_local.normalized();
	const Vector2f crosstrack_error = (prev_wp_local + distance_on_line_segment) - curr_pos_local;

	// Calculate desired heading towards lookahead point
	float desired_heading{0.f};
	float lookahead_distance = math::constrain(lookahead_gain * desired_speed,
				   lookahead_min, lookahead_max);

	if (crosstrack_error.longerThan(lookahead_distance)) {
		if (crosstrack_error.norm() < lookahead_max) {
			lookahead_distance = crosstrack_error.norm(); // Scale lookahead radius
			desired_heading = calcDesiredHeading(curr_wp_local, prev_wp_local, curr_pos_local, lookahead_distance);

		} else { // Excessively large crosstrack error
			desired_heading = calcDesiredHeading(curr_wp_local, curr_pos_local, curr_pos_local, lookahead_distance);
		}

	} else { // Crosstrack error smaller than lookahead
		desired_heading = calcDesiredHeading(curr_wp_local, prev_wp_local, curr_pos_local, lookahead_distance);
	}

	// Calculate desired steering to reach lookahead point
	const float heading_error = matrix::wrap_pi(desired_heading - vehicle_yaw);

	// For logging
	_rover_ackermann_guidance_status.lookahead_distance = lookahead_distance;
	_rover_ackermann_guidance_status.heading_error = (heading_error * 180.f) / (M_PI_F);
	_rover_ackermann_guidance_status.crosstrack_error = crosstrack_error.norm();

	// Calculate desired steering
	if (math::abs_t(heading_error) <= M_PI_2_F) {
		return atanf(2 * wheel_base * sinf(heading_error) / lookahead_distance);

	} else if (heading_error > FLT_EPSILON) {
		return atanf(2 * wheel_base * (1.0f + sinf(heading_error - M_PI_2_F)) /
			     lookahead_distance);

	} else {
		return atanf(2 * wheel_base * (-1.0f + sinf(heading_error + M_PI_2_F)) /
			     lookahead_distance);
	}
}

float RoverAckermannGuidance::calcDesiredHeading(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
		const Vector2f &curr_pos_local,
		const float &lookahead_distance)
{
	// Setup variables
	const float line_segment_slope = (curr_wp_local(1) - prev_wp_local(1)) / (curr_wp_local(0) - prev_wp_local(0));
	const float line_segment_rover_offset = prev_wp_local(1) - curr_pos_local(1) + line_segment_slope * (curr_pos_local(
			0) - prev_wp_local(0));
	const float a = -line_segment_slope;
	const float c = -line_segment_rover_offset;
	const float r = lookahead_distance;
	const float x0 = -a * c / (a * a + 1.0f);
	const float y0 = -c / (a * a + 1.0f);

	// Calculate intersection points
	if (c * c > r * r * (a * a + 1.0f) + FLT_EPSILON) { // No intersection points exist
		return 0.f;

	} else if (abs(c * c - r * r * (a * a + 1.0f)) < FLT_EPSILON) { // One intersection point exists
		return atan2f(y0, x0);

	} else { // Two intersetion points exist
		const float d = r * r - c * c / (a * a + 1.0f);
		const float mult = sqrt(d / (a * a + 1.0f));
		const float ax = x0 + mult;
		const float bx = x0 - mult;
		const float ay = y0 - a * mult;
		const float by = y0 + a * mult;
		const Vector2f point1(ax, ay);
		const Vector2f point2(bx, by);
		const Vector2f distance1 = (curr_wp_local - curr_pos_local) - point1;
		const Vector2f distance2 = (curr_wp_local - curr_pos_local) - point2;

		// Return intersection point closer to current waypoint
		if (distance1.norm_squared() < distance2.norm_squared()) {
			return atan2f(ay, ax);

		} else {
			return atan2f(by, bx);
		}
	}
}
