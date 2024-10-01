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
}

void RoverAckermannGuidance::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ra_wheel_base.get() > FLT_EPSILON && _param_ra_max_lat_accel.get() > FLT_EPSILON
	    && _param_ra_max_steer_angle.get() > FLT_EPSILON) {
		_min_speed = sqrt(_param_ra_wheel_base.get() * _param_ra_max_lat_accel.get() / tanf(_param_ra_max_steer_angle.get()));
	}
}

void RoverAckermannGuidance::computeGuidance(const float vehicle_forward_speed,
		const float vehicle_yaw, const int nav_state, const bool armed)
{
	updateSubscriptions();

	// Distances to waypoints
	const float distance_to_prev_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
					  _prev_wp(0), _prev_wp(1));
	const float distance_to_curr_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
					  _curr_wp(0), _curr_wp(1));

	// Catch return to launch
	if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		_mission_finished = distance_to_curr_wp < _param_nav_acc_rad.get();
	}

	// Guidance logic
	if (_mission_finished) { // Mission is finished
		_desired_steering = 0.f;
		_desired_speed = 0.f;

	} else if (_nav_cmd == 93) { // Catch delay command
		_desired_speed = 0.f;

	} else { // Regular guidance algorithm

		_desired_speed = calcDesiredSpeed(_cruising_speed, _min_speed, distance_to_prev_wp, distance_to_curr_wp,
						  _acceptance_radius,
						  _prev_acceptance_radius, _param_ra_max_decel.get(), _param_ra_max_jerk.get(), nav_state, _waypoint_transition_angle,
						  _prev_waypoint_transition_angle, _param_ra_max_speed.get());

		_desired_steering = calcDesiredSteering(_pure_pursuit, _curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
							_param_ra_wheel_base.get(), _desired_speed, vehicle_yaw, _param_ra_max_steer_angle.get(), armed);

	}

	// Publish ackermann controller status (logging)
	hrt_abstime timestamp = hrt_absolute_time();
	_rover_ackermann_guidance_status.timestamp = timestamp;
	_rover_ackermann_guidance_status_pub.publish(_rover_ackermann_guidance_status);

	// Publish speed and steering setpoints
	rover_ackermann_setpoint_s rover_ackermann_setpoint{};
	rover_ackermann_setpoint.timestamp = timestamp;
	rover_ackermann_setpoint.forward_speed_setpoint = _desired_speed;
	rover_ackermann_setpoint.forward_speed_setpoint_normalized = NAN;
	rover_ackermann_setpoint.steering_setpoint = NAN;
	rover_ackermann_setpoint.steering_setpoint_normalized = NAN;
	rover_ackermann_setpoint.lateral_acceleration_setpoint = powf(vehicle_forward_speed,
			2.f) * tanf(_desired_steering) / _param_ra_wheel_base.get();
	_rover_ackermann_setpoint_pub.publish(rover_ackermann_setpoint);

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
	}

	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypointsAndAcceptanceRadius();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}

	if (_navigator_mission_item_sub.updated()) {
		navigator_mission_item_s navigator_mission_item{};
		_navigator_mission_item_sub.copy(&navigator_mission_item);
		_nav_cmd = navigator_mission_item.nav_cmd;
	}
}

void RoverAckermannGuidance::updateWaypointsAndAcceptanceRadius()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

	// Global waypoint coordinates
	_prev_wp = _curr_pos.isAllFinite() ? _curr_pos : Vector2d(0, 0); // Fallback if previous waypoint is invalid
	_curr_wp = _curr_pos.isAllFinite() ? _curr_pos : Vector2d(0, 0); // Fallback if current waypoint is invalid
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

	// Distances
	const Vector2f curr_to_next_wp_ned = _next_wp_ned - _curr_wp_ned;
	const Vector2f curr_to_prev_wp_ned = _prev_wp_ned - _curr_wp_ned;
	float cosin = curr_to_prev_wp_ned.unit_or_zero() * curr_to_next_wp_ned.unit_or_zero();
	cosin = math::constrain<float>(cosin, -1.f, 1.f); // Protect against float precision problem
	_prev_waypoint_transition_angle = _waypoint_transition_angle;
	_waypoint_transition_angle = acosf(cosin);

	// Update acceptance radius
	_prev_acceptance_radius = _acceptance_radius;

	if (_param_ra_acc_rad_max.get() >= _param_nav_acc_rad.get()) {
		_acceptance_radius = updateAcceptanceRadius(_waypoint_transition_angle, _param_nav_acc_rad.get(),
				     _param_ra_acc_rad_gain.get(), _param_ra_acc_rad_max.get(), _param_ra_wheel_base.get(), _param_ra_max_steer_angle.get());

	} else {
		_acceptance_radius = _param_nav_acc_rad.get();
	}

	// Waypoint cruising speed
	if (position_setpoint_triplet.current.cruising_speed > 0.f) {
		_cruising_speed = math::constrain(position_setpoint_triplet.current.cruising_speed, 0.f, _param_ra_max_speed.get());

	} else {
		_cruising_speed = _param_ra_miss_spd_def.get();
	}
}

float RoverAckermannGuidance::updateAcceptanceRadius(const float waypoint_transition_angle,
		const float default_acceptance_radius, const float acceptance_radius_gain,
		const float acceptance_radius_max, const float wheel_base, const float max_steer_angle)
{
	// Calculate acceptance radius s.t. the rover cuts the corner tangential to the current and next line segment
	float acceptance_radius = default_acceptance_radius;
	const float theta = waypoint_transition_angle / 2.f;
	const float min_turning_radius = wheel_base / sinf(max_steer_angle);
	const float acceptance_radius_temp = min_turning_radius / tanf(theta);
	const float acceptance_radius_temp_scaled = acceptance_radius_gain *
			acceptance_radius_temp; // Scale geometric ideal acceptance radius to account for kinematic and dynamic effects
	acceptance_radius = math::constrain<float>(acceptance_radius_temp_scaled, default_acceptance_radius,
			    acceptance_radius_max);

	// Publish updated acceptance radius
	position_controller_status_s pos_ctrl_status{};
	pos_ctrl_status.acceptance_radius = acceptance_radius;
	pos_ctrl_status.timestamp = hrt_absolute_time();
	_position_controller_status_pub.publish(pos_ctrl_status);
	return acceptance_radius;
}

float RoverAckermannGuidance::calcDesiredSpeed(const float cruising_speed, const float miss_speed_min,
		const float distance_to_prev_wp, const float distance_to_curr_wp, const float acc_rad,
		const float prev_acc_rad, const float max_decel, const float max_jerk, const int nav_state,
		const float waypoint_transition_angle, const float prev_waypoint_transition_angle, const float max_speed)
{
	// Catch improper values
	if (miss_speed_min < -FLT_EPSILON  || miss_speed_min > cruising_speed) {
		return cruising_speed;
	}

	// Cornering slow down effect
	if (distance_to_prev_wp <= prev_acc_rad && prev_acc_rad > FLT_EPSILON) {
		const float turning_circle = prev_acc_rad * tanf(prev_waypoint_transition_angle / 2.f);
		const float cornering_speed = sqrtf(turning_circle * _param_ra_max_lat_accel.get());
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	} else if (distance_to_curr_wp <= acc_rad && acc_rad > FLT_EPSILON) {
		const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
		const float cornering_speed = sqrtf(turning_circle * _param_ra_max_lat_accel.get());
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	}

	// Straight line speed
	if (max_decel > FLT_EPSILON && max_jerk > FLT_EPSILON && acc_rad > FLT_EPSILON) {
		float straight_line_speed{0.f};

		if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
			straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
					      max_decel, distance_to_curr_wp, 0.f);

		} else {
			const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
			float cornering_speed = sqrtf(turning_circle * _param_ra_max_lat_accel.get());
			cornering_speed = math::constrain(cornering_speed, miss_speed_min, cruising_speed);
			straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
					      max_decel, distance_to_curr_wp - acc_rad, cornering_speed);
		}

		return math::min(straight_line_speed, cruising_speed);

	} else {
		return cruising_speed;
	}

}

float RoverAckermannGuidance::calcDesiredSteering(PurePursuit &pure_pursuit, const Vector2f &curr_wp_ned,
		const Vector2f &prev_wp_ned, const Vector2f &curr_pos_ned, const float wheel_base, const float desired_speed,
		const float vehicle_yaw, const float max_steering, const bool armed)
{
	const float desired_heading = pure_pursuit.calcDesiredHeading(curr_wp_ned, prev_wp_ned, curr_pos_ned,
				      desired_speed);
	const float lookahead_distance = pure_pursuit.getLookaheadDistance();
	const float heading_error = matrix::wrap_pi(desired_heading - vehicle_yaw);
	// For logging
	_rover_ackermann_guidance_status.lookahead_distance = lookahead_distance;
	_rover_ackermann_guidance_status.heading_error = (heading_error * 180.f) / (M_PI_F);

	float desired_steering{0.f};

	if (!armed) {
		return desired_steering;
	}

	if (math::abs_t(heading_error) <= M_PI_2_F) {
		desired_steering = atanf(2 * wheel_base * sinf(heading_error) / lookahead_distance);


	} else {
		desired_steering = atanf(2 * wheel_base * (sign(heading_error) * 1.0f + sinf(heading_error -
					 sign(heading_error) * M_PI_2_F)) / lookahead_distance);
	}

	return math::constrain(desired_steering, -max_steering, max_steering);

}
