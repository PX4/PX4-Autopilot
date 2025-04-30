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

#include "RoverAckermann.hpp"

using namespace time_literals;

RoverAckermann::RoverAckermann() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool RoverAckermann::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverAckermann::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

	if (_param_ra_wheel_base.get() > FLT_EPSILON && _max_yaw_rate > FLT_EPSILON
	    && _param_ra_max_str_ang.get() > FLT_EPSILON) {
		_min_speed = _param_ra_wheel_base.get() * _max_yaw_rate / tanf(_param_ra_max_str_ang.get());
	}
}

void RoverAckermann::Run()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
		runSanityChecks();
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode{};
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);

		// Run sanity checks if the control mode changes (Note: This has to be done this way, because the topic is periodically updated and not on changes)
		if (vehicle_control_mode.flag_control_position_enabled != _vehicle_control_mode.flag_control_position_enabled ||
		    vehicle_control_mode.flag_control_velocity_enabled != _vehicle_control_mode.flag_control_velocity_enabled ||
		    vehicle_control_mode.flag_control_attitude_enabled != _vehicle_control_mode.flag_control_attitude_enabled ||
		    vehicle_control_mode.flag_control_rates_enabled != _vehicle_control_mode.flag_control_rates_enabled) {
			_vehicle_control_mode = vehicle_control_mode;
			runSanityChecks();

		} else {
			_vehicle_control_mode = vehicle_control_mode;
		}

	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		// Reset all controllers if the navigation state changes
		if (vehicle_status.nav_state != _nav_state) { reset();}

		_nav_state = vehicle_status.nav_state;
	}

	if (_vehicle_control_mode.flag_armed && _sanity_checks_passed) {
		// Generate setpoints
		if (_vehicle_control_mode.flag_control_manual_enabled) {
			manualControl();

		} else if (_vehicle_control_mode.flag_control_auto_enabled) {
			autoPositionMode();

		} else if (_vehicle_control_mode.flag_control_offboard_enabled) {
			offboardControl();
		}

		updateControllers();

	} else if (_was_armed) { // Reset all controllers and stop the vehicle
		reset();
		_ackermann_act_control.stopVehicle();
		_was_armed = false;
	}

}

void RoverAckermann::manualControl()
{
	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		manualManualMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		manualAcroMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		manualStabMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		manualPositionMode();
		break;
	}
}

void RoverAckermann::manualManualMode()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = hrt_absolute_time();
	rover_steering_setpoint.normalized_steering_angle = manual_control_setpoint.roll;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
}

void RoverAckermann::manualAcroMode()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	rover_rate_setpoint_s rover_rate_setpoint{};
	rover_rate_setpoint.timestamp = hrt_absolute_time();
	rover_rate_setpoint.yaw_rate_setpoint = matrix::sign(manual_control_setpoint.throttle) * math::interpolate<float>
						(manual_control_setpoint.roll, -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
	_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
}

void RoverAckermann::manualStabMode()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	const float yaw_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
				_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate / _param_ro_yaw_p.get(),
				_max_yaw_rate / _param_ro_yaw_p.get());

	if (fabsf(yaw_delta) > FLT_EPSILON
	    || fabsf(rover_throttle_setpoint.throttle_body_x) < FLT_EPSILON) { // Closed loop yaw rate control
		_stab_yaw_setpoint = NAN;
		const float yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + matrix::sign(manual_control_setpoint.throttle) * yaw_delta);
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = yaw_setpoint;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

	} else { // Closed loop yaw control if the yaw rate input is zero (keep current yaw)
		if (!PX4_ISFINITE(_stab_yaw_setpoint)) {
			_stab_yaw_setpoint = _vehicle_yaw;
		}

		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = _stab_yaw_setpoint;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
	}
}

void RoverAckermann::manualPositionMode()
{
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

	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	const float speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
				     -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
	const float yaw_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
				_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate / _param_ro_yaw_p.get(),
				_max_yaw_rate / _param_ro_yaw_p.get());

	if (fabsf(yaw_delta) > FLT_EPSILON
	    || fabsf(speed_setpoint) < FLT_EPSILON) { // Closed loop yaw rate control
		_pos_ctl_course_direction = Vector2f(NAN, NAN);
		// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
		const float yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + sign(speed_setpoint) * yaw_delta);
		const Vector2f pos_ctl_course_direction = Vector2f(cos(yaw_setpoint), sin(yaw_setpoint));
		const Vector2f target_waypoint_ned = _curr_pos_ned + sign(speed_setpoint) * _param_pp_lookahd_max.get() *
						     pos_ctl_course_direction;
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = target_waypoint_ned(0);
		rover_position_setpoint.position_ned[1] = target_waypoint_ned(1);
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.cruising_speed = speed_setpoint;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else { // Course control if the steering input is zero (keep driving on a straight line)
		if (!_pos_ctl_course_direction.isAllFinite()) {
			_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
			_pos_ctl_start_position_ned = _curr_pos_ned;
		}

		// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
		const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
		const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_pp_lookahd_max.get();
		const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(speed_setpoint) *
						     vector_scaling * _pos_ctl_course_direction;
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = target_waypoint_ned(0);
		rover_position_setpoint.position_ned[1] = target_waypoint_ned(1);
		rover_position_setpoint.start_ned[0] = _pos_ctl_start_position_ned(0);
		rover_position_setpoint.start_ned[1] = _pos_ctl_start_position_ned(1);
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.cruising_speed = speed_setpoint;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}
}

void RoverAckermann::autoPositionMode()
{
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

	if (_position_setpoint_triplet_sub.updated()) {
		autoUpdateWaypointsAndAcceptanceRadius();
	}

	// Distances to waypoints
	const float distance_to_prev_wp = sqrt(powf(_curr_pos_ned(0) - _prev_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _prev_wp_ned(1), 2));
	const float distance_to_curr_wp = sqrt(powf(_curr_pos_ned(0) - _curr_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _curr_wp_ned(1), 2));

	rover_position_setpoint_s rover_position_setpoint{};
	rover_position_setpoint.timestamp = hrt_absolute_time();
	rover_position_setpoint.position_ned[0] = _curr_wp_ned(0);
	rover_position_setpoint.position_ned[1] = _curr_wp_ned(1);
	rover_position_setpoint.start_ned[0] = _prev_wp_ned(0);
	rover_position_setpoint.start_ned[1] = _prev_wp_ned(1);
	rover_position_setpoint.arrival_speed = autoArrivalSpeed(_cruising_speed, _min_speed, _acceptance_radius, _curr_wp_type,
						_waypoint_transition_angle, _max_yaw_rate);
	rover_position_setpoint.cruising_speed = autoCruisingSpeed(_cruising_speed, _min_speed, distance_to_prev_wp,
			distance_to_curr_wp, _acceptance_radius, _prev_acceptance_radius, _waypoint_transition_angle,
			_prev_waypoint_transition_angle, _max_yaw_rate);
	rover_position_setpoint.yaw = NAN;
	_rover_position_setpoint_pub.publish(rover_position_setpoint);
}

void RoverAckermann::autoUpdateWaypointsAndAcceptanceRadius()
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
		_acceptance_radius = autoUpdateAcceptanceRadius(_waypoint_transition_angle, _param_nav_acc_rad.get(),
				     _param_ra_acc_rad_gain.get(), _param_ra_acc_rad_max.get(), _param_ra_wheel_base.get(), _param_ra_max_str_ang.get());

	} else {
		_acceptance_radius = _param_nav_acc_rad.get();
	}

	// Waypoint cruising speed
	_cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
				  position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();
}

float RoverAckermann::autoUpdateAcceptanceRadius(const float waypoint_transition_angle,
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

float RoverAckermann::autoArrivalSpeed(const float cruising_speed, const float miss_speed_min, const float acc_rad,
				       const int curr_wp_type, const float waypoint_transition_angle, const float max_yaw_rate)
{
	if (!PX4_ISFINITE(waypoint_transition_angle)
	    || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		return 0.f; // Stop at the waypoint

	} else {
		const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
		const float cornering_speed = max_yaw_rate * turning_circle;
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed); // Slow down for cornering
	}
}

float RoverAckermann::autoCruisingSpeed(const float cruising_speed, const float miss_speed_min,
					const float distance_to_prev_wp, const float distance_to_curr_wp, const float acc_rad, const float prev_acc_rad,
					const float waypoint_transition_angle, const float prev_waypoint_transition_angle, const float max_yaw_rate)
{
	// Catch improper values
	if (miss_speed_min < -FLT_EPSILON  || miss_speed_min > cruising_speed) {
		return cruising_speed;
	}

	// Cornering slow down effect
	if (distance_to_prev_wp <= prev_acc_rad && prev_acc_rad > FLT_EPSILON && PX4_ISFINITE(prev_waypoint_transition_angle)) {
		const float turning_circle = prev_acc_rad * tanf(prev_waypoint_transition_angle / 2.f);
		const float cornering_speed = max_yaw_rate * turning_circle;
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	}

	if (distance_to_curr_wp <= acc_rad && acc_rad > FLT_EPSILON && PX4_ISFINITE(waypoint_transition_angle)) {
		const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
		const float cornering_speed = max_yaw_rate * turning_circle;
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	}

	return cruising_speed; // Fallthrough

}

void RoverAckermann::offboardControl()
{
	offboard_control_mode_s offboard_control_mode{};
	_offboard_control_mode_sub.copy(&offboard_control_mode);

	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	if (offboard_control_mode.position) {
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
		rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else if (offboard_control_mode.velocity) {
		const Vector2f velocity_ned(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = hrt_absolute_time();
		rover_velocity_setpoint.speed = velocity_ned.norm();
		rover_velocity_setpoint.bearing = atan2f(velocity_ned(1), velocity_ned(0));
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	}
}

void RoverAckermann::updateControllers()
{
	if (_vehicle_control_mode.flag_control_position_enabled) {
		_ackermann_pos_control.updatePosControl();
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled) {
		_ackermann_vel_control.updateVelControl();
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled) {
		_ackermann_att_control.updateAttControl();
	}

	if (_vehicle_control_mode.flag_control_rates_enabled) {
		_ackermann_rate_control.updateRateControl();
	}

	if (_vehicle_control_mode.flag_control_allocation_enabled) {
		_ackermann_act_control.updateActControl();
	}
}

void RoverAckermann::runSanityChecks()
{
	if (_vehicle_control_mode.flag_control_rates_enabled && !_ackermann_rate_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled && !_ackermann_att_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled && !_ackermann_vel_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_position_enabled && !_ackermann_pos_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	_sanity_checks_passed = true;
}

void RoverAckermann::reset()
{
	_ackermann_vel_control.reset();
	_ackermann_att_control.reset();
	_ackermann_rate_control.reset();
	_stab_yaw_setpoint = NAN;
	_pos_ctl_course_direction = Vector2f(NAN, NAN);
	_pos_ctl_start_position_ned = Vector2f(NAN, NAN);
	_curr_pos_ned = Vector2f(NAN, NAN);
}

int RoverAckermann::task_spawn(int argc, char *argv[])
{
	RoverAckermann *instance = new RoverAckermann();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverAckermann::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverAckermann::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover ackermann module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_ackermann", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_ackermann_main(int argc, char *argv[])
{
	return RoverAckermann::main(argc, argv);
}
