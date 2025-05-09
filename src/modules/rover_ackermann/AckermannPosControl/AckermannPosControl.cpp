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

#include "AckermannPosControl.hpp"

using namespace time_literals;

AckermannPosControl::AckermannPosControl(ModuleParams *parent) : ModuleParams(parent)
{
	_pure_pursuit_status_pub.advertise();
	_rover_position_setpoint_pub.advertise();
	_rover_velocity_setpoint_pub.advertise();

	// Initially set to NaN to indicate that the rover has no position setpoint
	_rover_position_setpoint.position_ned[0] = NAN;
	_rover_position_setpoint.position_ned[1] = NAN;

	updateParams();
}

void AckermannPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

	if (_param_ra_wheel_base.get() > FLT_EPSILON && _max_yaw_rate > FLT_EPSILON
	    && _param_ra_max_str_ang.get() > FLT_EPSILON) {
		_min_speed = _param_ra_wheel_base.get() * _max_yaw_rate / tanf(_param_ra_max_str_ang.get());
	}
}

void AckermannPosControl::updatePosControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	if (_vehicle_control_mode.flag_control_position_enabled && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		if (_vehicle_control_mode.flag_control_offboard_enabled) {
			generatePositionSetpoint();
		}

		generateVelocitySetpoint();

	}

}

void AckermannPosControl::updateSubscriptions()
{
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

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

}

void AckermannPosControl::generatePositionSetpoint()
{
	if (_offboard_control_mode_sub.updated()) {
		_offboard_control_mode_sub.copy(&_offboard_control_mode);
	}

	if (!_offboard_control_mode.position) {
		return;
	}

	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	// Translate trajectory setpoint to rover position setpoint
	rover_position_setpoint_s rover_position_setpoint{};
	rover_position_setpoint.timestamp = _timestamp;
	rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
	rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
	rover_position_setpoint.cruising_speed = _param_ro_speed_limit.get();
	rover_position_setpoint.yaw = NAN;
	_rover_position_setpoint_pub.publish(rover_position_setpoint);

}

void AckermannPosControl::generateVelocitySetpoint()
{
	// Manual Position Mode
	if (_vehicle_control_mode.flag_control_manual_enabled && _vehicle_control_mode.flag_control_position_enabled) {
		manualPositionMode();
		return;
	}

	// Auto Mode
	if (_vehicle_control_mode.flag_control_auto_enabled) {
		autoPositionMode();
		return;
	}

	// Rover Position Setpoint
	if (_rover_position_setpoint_sub.copy(&_rover_position_setpoint)
	    && PX4_ISFINITE(_rover_position_setpoint.position_ned[0]) && PX4_ISFINITE(_rover_position_setpoint.position_ned[1])) {
		goToPositionMode();
		return;
	}

}

void AckermannPosControl::manualPositionMode()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	const float speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
				     -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
	const float yaw_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
				_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate / _param_ro_yaw_p.get(),
				_max_yaw_rate / _param_ro_yaw_p.get());

	if (fabsf(yaw_delta) > FLT_EPSILON
	    || fabsf(speed_setpoint) < FLT_EPSILON) { // Closed loop yaw rate control
		_course_control = false;
		const float yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + sign(speed_setpoint) * yaw_delta);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = speed_setpoint;
		rover_velocity_setpoint.bearing = yaw_setpoint;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	} else { // Course control if the steering input is zero (keep driving on a straight line)
		if (!_course_control) {
			_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
			_pos_ctl_start_position_ned = _curr_pos_ned;
			_course_control = true;
		}

		// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
		const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
		const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_pp_lookahd_max.get();
		const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(speed_setpoint) *
						     vector_scaling * _pos_ctl_course_direction;
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _pos_ctl_start_position_ned,
					       _curr_pos_ned, fabsf(speed_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = speed_setpoint;
		rover_velocity_setpoint.bearing = speed_setpoint > -FLT_EPSILON ? bearing_setpoint : matrix::wrap_pi(
				bearing_setpoint + M_PI_F);
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	}
}

void AckermannPosControl::autoPositionMode()
{
	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypointsAndAcceptanceRadius();
	}

	// Distances to waypoints
	const float distance_to_prev_wp = sqrt(powf(_curr_pos_ned(0) - _prev_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _prev_wp_ned(1), 2));
	const float distance_to_curr_wp = sqrt(powf(_curr_pos_ned(0) - _curr_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _curr_wp_ned(1), 2));

	// Check stopping conditions
	bool auto_stop{false};

	if (_curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || _curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE
	    || !_next_wp_ned.isAllFinite()) {
		auto_stop = distance_to_curr_wp < _param_nav_acc_rad.get();
	}

	if (auto_stop) {
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = 0.f;
		rover_velocity_setpoint.bearing = _vehicle_yaw;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	} else { // Regular guidance algorithm
		const float speed_setpoint = calcSpeedSetpoint(_cruising_speed, _min_speed, distance_to_prev_wp,
					     distance_to_curr_wp, _acceptance_radius, _prev_acceptance_radius, _param_ro_decel_limit.get(),
					     _param_ro_jerk_limit.get(), _curr_wp_type, _waypoint_transition_angle, _prev_waypoint_transition_angle,
					     _param_ro_speed_limit.get());
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
					   fabsf(speed_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = speed_setpoint;
		rover_velocity_setpoint.bearing = yaw_setpoint;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	}
}

void AckermannPosControl::updateWaypointsAndAcceptanceRadius()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);
	_curr_wp_type = position_setpoint_triplet.current.type;

	RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
			_curr_pos_ned, _global_ned_proj_ref);

	_prev_waypoint_transition_angle = _waypoint_transition_angle;
	_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);

	// Update acceptance radius
	_prev_acceptance_radius = _acceptance_radius;

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

float AckermannPosControl::updateAcceptanceRadius(const float waypoint_transition_angle,
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
	pos_ctrl_status.timestamp = _timestamp;
	_position_controller_status_pub.publish(pos_ctrl_status);
	return acceptance_radius;
}

float AckermannPosControl::calcSpeedSetpoint(const float cruising_speed, const float miss_speed_min,
		const float distance_to_prev_wp, const float distance_to_curr_wp, const float acc_rad,
		const float prev_acc_rad, const float max_decel, const float max_jerk, const int curr_wp_type,
		const float waypoint_transition_angle, const float prev_waypoint_transition_angle, const float max_speed)
{
	// Catch improper values
	if (miss_speed_min < -FLT_EPSILON  || miss_speed_min > cruising_speed) {
		return cruising_speed;
	}

	// Upcoming stop
	if (max_decel > FLT_EPSILON && max_jerk > FLT_EPSILON && (!PX4_ISFINITE(waypoint_transition_angle)
			|| curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
			|| curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {
		const float straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
						  max_decel, distance_to_curr_wp, 0.f);
		return math::min(straight_line_speed, cruising_speed);
	}

	// Cornering slow down effect
	if (distance_to_prev_wp <= prev_acc_rad && prev_acc_rad > FLT_EPSILON && PX4_ISFINITE(prev_waypoint_transition_angle)) {
		const float turning_circle = prev_acc_rad * tanf(prev_waypoint_transition_angle / 2.f);
		const float cornering_speed = _max_yaw_rate * turning_circle;
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	}

	if (distance_to_curr_wp <= acc_rad && acc_rad > FLT_EPSILON && PX4_ISFINITE(waypoint_transition_angle)) {
		const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
		const float cornering_speed = _max_yaw_rate * turning_circle;
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	}

	// Straight line speed
	if (max_decel > FLT_EPSILON && max_jerk > FLT_EPSILON && acc_rad > FLT_EPSILON) {
		const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
		float cornering_speed = _max_yaw_rate * turning_circle;
		cornering_speed = math::constrain(cornering_speed, miss_speed_min, cruising_speed);
		const float straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
						  max_decel, distance_to_curr_wp - acc_rad, cornering_speed);
		return math::min(straight_line_speed, cruising_speed);

	}

	return cruising_speed; // Fallthrough

}

void AckermannPosControl::goToPositionMode()
{
	const Vector2f target_waypoint_ned(_rover_position_setpoint.position_ned[0], _rover_position_setpoint.position_ned[1]);
	const float distance_to_target = (target_waypoint_ned - _curr_pos_ned).norm();

	if (distance_to_target > _param_nav_acc_rad.get()) {
		float speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
				       _param_ro_decel_limit.get(), distance_to_target, 0.f);
		const float max_speed = PX4_ISFINITE(_rover_position_setpoint.cruising_speed) ?
					_rover_position_setpoint.cruising_speed :
					_param_ro_speed_limit.get();
		speed_setpoint = math::min(speed_setpoint, max_speed);
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _curr_pos_ned,
					   _curr_pos_ned, fabsf(speed_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = speed_setpoint;
		rover_velocity_setpoint.bearing = yaw_setpoint;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	} else {
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = 0.f;
		rover_velocity_setpoint.bearing = _vehicle_yaw;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);
	}
}

bool AckermannPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ra_wheel_base.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ra_max_str_ang.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_yaw_p.get() < FLT_EPSILON) {
		ret = false;
	}

	_prev_param_check_passed = ret;
	return ret;
}
