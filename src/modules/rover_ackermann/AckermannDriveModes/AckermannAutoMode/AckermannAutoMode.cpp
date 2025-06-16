/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "AckermannAutoMode.hpp"

using namespace time_literals;

AckermannAutoMode::AckermannAutoMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_position_setpoint_pub.advertise();
}

void AckermannAutoMode::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

	if (_param_ra_wheel_base.get() > FLT_EPSILON && _max_yaw_rate > FLT_EPSILON
	    && _param_ra_max_str_ang.get() > FLT_EPSILON) {
		_min_speed = _param_ra_wheel_base.get() * _max_yaw_rate / tanf(_param_ra_max_str_ang.get());
	}
}

void AckermannAutoMode::autoControl()
{
	if (_position_setpoint_triplet_sub.updated()) {
		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position{};
			_vehicle_local_position_sub.copy(&vehicle_local_position);

			if (!_global_ned_proj_ref.isInitialized()
			    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
				_global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
								   vehicle_local_position.ref_timestamp);
			}

			_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		}

		updateWaypointsAndAcceptanceRadius();

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = _curr_wp_ned(0);
		rover_position_setpoint.position_ned[1] = _curr_wp_ned(1);
		rover_position_setpoint.start_ned[0] = _prev_wp_ned(0);
		rover_position_setpoint.start_ned[1] = _prev_wp_ned(1);
		rover_position_setpoint.arrival_speed = arrivalSpeed(_cruising_speed, _min_speed, _acceptance_radius, _curr_wp_type,
							_waypoint_transition_angle, _max_yaw_rate);
		rover_position_setpoint.cruising_speed = _cruising_speed;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}

}

void AckermannAutoMode::updateWaypointsAndAcceptanceRadius()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);
	_curr_wp_type = position_setpoint_triplet.current.type;

	RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
			_curr_pos_ned, _global_ned_proj_ref);

	_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);

	// Update acceptance radius
	if (_param_ra_acc_rad_max.get() >= _param_nav_acc_rad.get()) {
		_acceptance_radius = updateAcceptanceRadius(_waypoint_transition_angle, _param_nav_acc_rad.get(),
				     _param_ra_acc_rad_gain.get(), _param_ra_acc_rad_max.get(), _param_ra_wheel_base.get(), _param_ra_max_str_ang.get());

	} else {
		_acceptance_radius = _param_nav_acc_rad.get();
	}

	// Waypoint cruising speed
	_cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
				  position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();
}

float AckermannAutoMode::updateAcceptanceRadius(const float waypoint_transition_angle,
		const float default_acceptance_radius, const float acceptance_radius_gain,
		const float acceptance_radius_max, const float wheel_base, const float max_steer_angle)
{
	// Calculate acceptance radius s.t. the rover cuts the corner tangential to the current and next line segment
	float acceptance_radius = default_acceptance_radius;

	if (PX4_ISFINITE(_waypoint_transition_angle)) {
		const float theta = waypoint_transition_angle / 2.f;
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

float AckermannAutoMode::arrivalSpeed(const float cruising_speed, const float min_speed, const float acc_rad,
				      const int curr_wp_type, const float waypoint_transition_angle, const float max_yaw_rate)
{
	if (!PX4_ISFINITE(waypoint_transition_angle)
	    || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		return 0.f; // Stop at the waypoint

	} else if (_param_ro_speed_red.get() > FLT_EPSILON) {
		const float speed_reduction = math::constrain(_param_ro_speed_red.get() * math::interpolate(
						      M_PI_F - waypoint_transition_angle,
						      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
		return math::constrain(_param_ro_max_thr_speed.get() * (1.f - speed_reduction), min_speed,
				       cruising_speed); // Slow down for cornering
	}

	return cruising_speed; // Fallthrough
}
