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

#include "DifferentialPosVelControl.hpp"

using namespace time_literals;

DifferentialPosVelControl::DifferentialPosVelControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_velocity_status_pub.advertise();
	_pure_pursuit_status_pub.advertise();
	updateParams();
}

void DifferentialPosVelControl::updateParams()
{
	ModuleParams::updateParams();
	_pid_speed.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed.setIntegralLimit(1.f);
	_pid_speed.setOutputLimit(1.f);
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

	if (_param_ro_accel_limit.get() > FLT_EPSILON) {
		_speed_setpoint.setSlewRate(_param_ro_accel_limit.get());
	}
}

void DifferentialPosVelControl::updatePosVelControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	if ((_vehicle_control_mode.flag_control_position_enabled || _vehicle_control_mode.flag_control_velocity_enabled)
	    && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		generateAttitudeSetpoint();

		if (_param_ro_max_thr_speed.get() > FLT_EPSILON) {

			const float speed_body_x_setpoint_normalized = math::interpolate<float>(_speed_body_x_setpoint,
					-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

			if (_rover_steering_setpoint_sub.updated()) {
				_rover_steering_setpoint_sub.copy(&_rover_steering_setpoint);
			}

			if (fabsf(speed_body_x_setpoint_normalized) > 1.f -
			    fabsf(_rover_steering_setpoint.normalized_speed_diff)) { // Adjust speed setpoint if it is infeasible due to the desired speed difference of the left/right wheels
				_speed_body_x_setpoint = sign(_speed_body_x_setpoint) *
							 math::interpolate<float>(1.f - fabsf(_rover_steering_setpoint.normalized_speed_diff), -1.f, 1.f,
									 -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
			}
		}

		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = _timestamp;
		_speed_body_x_setpoint = fabsf(_speed_body_x_setpoint) > _param_ro_speed_th.get() ? _speed_body_x_setpoint : 0.f;
		rover_throttle_setpoint.throttle_body_x = RoverControl::speedControl(_speed_setpoint, _pid_speed,
				_speed_body_x_setpoint, _vehicle_speed_body_x, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
				_param_ro_max_thr_speed.get(), _dt);
		rover_throttle_setpoint.throttle_body_y = 0.f;
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	} else { // Reset controller and slew rate when position control is not active
		_pid_speed.resetIntegral();
		_speed_setpoint.setForcedValue(0.f);
	}

	// Publish position controller status (logging only)
	rover_velocity_status_s rover_velocity_status;
	rover_velocity_status.timestamp = _timestamp;
	rover_velocity_status.measured_speed_body_x = _vehicle_speed_body_x;
	rover_velocity_status.speed_body_x_setpoint = _speed_body_x_setpoint;
	rover_velocity_status.adjusted_speed_body_x_setpoint = _speed_setpoint.getState();
	rover_velocity_status.measured_speed_body_y = _vehicle_speed_body_y;
	rover_velocity_status.speed_body_y_setpoint = NAN;
	rover_velocity_status.adjusted_speed_body_y_setpoint = NAN;
	rover_velocity_status.pid_throttle_body_x_integral = _pid_speed.getIntegral();
	rover_velocity_status.pid_throttle_body_y_integral = NAN;
	_rover_velocity_status_pub.publish(rover_velocity_status);
}

void DifferentialPosVelControl::updateSubscriptions()
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
		const Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_speed_body_x = fabsf(velocity_in_body_frame(0)) > _param_ro_speed_th.get() ? velocity_in_body_frame(0) : 0.f;
		_vehicle_speed_body_y = fabsf(velocity_in_body_frame(1)) > _param_ro_speed_th.get() ? velocity_in_body_frame(1) : 0.f;
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;
		_vehicle_status_sub.copy(&vehicle_status);
		_nav_state = vehicle_status.nav_state;
	}

}

void DifferentialPosVelControl::generateAttitudeSetpoint()
{
	if (_vehicle_control_mode.flag_control_manual_enabled
	    && _vehicle_control_mode.flag_control_position_enabled) { // Position Mode
		manualPositionMode();

	} else if (_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard Control
		if (_offboard_control_mode_sub.updated()) {
			_offboard_control_mode_sub.copy(&_offboard_control_mode);
		}

		if (_offboard_control_mode.position) {
			offboardPositionMode();

		} else if (_offboard_control_mode.velocity) {
			offboardVelocityMode();
		}

	} else if (_vehicle_control_mode.flag_control_auto_enabled) { // Auto Mode
		autoPositionMode();
	}
}

void DifferentialPosVelControl::manualPositionMode()
{
	manual_control_setpoint_s manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		_speed_body_x_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
					 -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
		const float yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
						_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);

		if (fabsf(yaw_rate_setpoint) > FLT_EPSILON
		    || fabsf(_speed_body_x_setpoint) < FLT_EPSILON) { // Closed loop yaw rate control
			_course_control = false;
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = yaw_rate_setpoint;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

		} else { // Course control if the steering input is zero (keep driving on a straight line)
			if (!_course_control) {
				_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
				_pos_ctl_start_position_ned = _curr_pos_ned;
				_course_control = true;
			}

			// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
			const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
			const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_pp_lookahd_max.get();
			const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(_speed_body_x_setpoint) *
							     vector_scaling * _pos_ctl_course_direction;
			pure_pursuit_status_s pure_pursuit_status{};
			pure_pursuit_status.timestamp = _timestamp;
			float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					     _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _pos_ctl_start_position_ned,
					     _curr_pos_ned, fabsf(_speed_body_x_setpoint));
			_pure_pursuit_status_pub.publish(pure_pursuit_status);
			yaw_setpoint = _speed_body_x_setpoint > FLT_EPSILON ? yaw_setpoint : matrix::wrap_pi(yaw_setpoint + M_PI_F);
			rover_attitude_setpoint_s rover_attitude_setpoint{};
			rover_attitude_setpoint.timestamp = _timestamp;
			rover_attitude_setpoint.yaw_setpoint = yaw_setpoint;
			_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
		}
	}
}

void DifferentialPosVelControl::offboardPositionMode()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	// Translate trajectory setpoint to rover setpoints
	const Vector2f target_waypoint_ned(trajectory_setpoint.position[0], trajectory_setpoint.position[1]);
	const float distance_to_target = (target_waypoint_ned - _curr_pos_ned).norm();

	if (target_waypoint_ned.isAllFinite() && distance_to_target > _param_nav_acc_rad.get()) {
		const float speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
					     _param_ro_decel_limit.get(), distance_to_target, 0.f);
		_speed_body_x_setpoint = math::min(speed_setpoint, _param_ro_speed_limit.get());
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _curr_pos_ned,
					   _curr_pos_ned, fabsf(_speed_body_x_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		rover_attitude_setpoint.yaw_setpoint = yaw_setpoint;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

	} else {
		_speed_body_x_setpoint = 0.f;
	}
}

void DifferentialPosVelControl::offboardVelocityMode()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	const Vector2f velocity_in_local_frame(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);

	if (velocity_in_local_frame.isAllFinite()) {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		rover_attitude_setpoint.yaw_setpoint = atan2f(velocity_in_local_frame(1), velocity_in_local_frame(0));
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
		_speed_body_x_setpoint = velocity_in_local_frame.norm();
	}
}

void DifferentialPosVelControl::autoPositionMode()
{
	updateAutoSubscriptions();

	// Distances to waypoints
	const float distance_to_curr_wp = sqrt(powf(_curr_pos_ned(0) - _curr_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _curr_wp_ned(1), 2));

	if (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) { // Check RTL arrival
		_mission_finished = distance_to_curr_wp < _param_nav_acc_rad.get();
	}

	// State machine
	pure_pursuit_status_s pure_pursuit_status{};
	pure_pursuit_status.timestamp = _timestamp;
	float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
			     _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
			     fabsf(_speed_body_x_setpoint));
	_pure_pursuit_status_pub.publish(pure_pursuit_status);
	const float heading_error = matrix::wrap_pi(yaw_setpoint - _vehicle_yaw);

	if (!_mission_finished && distance_to_curr_wp > _param_nav_acc_rad.get()) {
		if (_currentState == GuidanceState::STOPPED) {
			_currentState = GuidanceState::DRIVING;
		}

		if (_currentState == GuidanceState::DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
			_currentState = GuidanceState::SPOT_TURNING;

		} else if (_currentState == GuidanceState::SPOT_TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
			_currentState = GuidanceState::DRIVING;
		}

	} else { // Mission finished or delay command
		_currentState = GuidanceState::STOPPED;
	}

	// Guidance logic
	switch (_currentState) {
	case GuidanceState::DRIVING: {
			// Calculate desired speed in body x direction
			_speed_body_x_setpoint = calcSpeedSetpoint(_cruising_speed, distance_to_curr_wp, _param_ro_decel_limit.get(),
						 _param_ro_jerk_limit.get(), _waypoint_transition_angle, _param_ro_speed_limit.get(), _param_rd_trans_drv_trn.get(),
						 _param_rd_miss_spd_gain.get());

		} break;

	case GuidanceState::SPOT_TURNING:
		if (fabsf(_vehicle_speed_body_x) > 0.f) {
			yaw_setpoint = _vehicle_yaw; // Wait for the rover to stop

		}

		_speed_body_x_setpoint = 0.f;
		break;

	case GuidanceState::STOPPED:
	default:
		yaw_setpoint = _vehicle_yaw;
		_speed_body_x_setpoint = 0.f;
		break;

	}

	rover_attitude_setpoint_s rover_attitude_setpoint{};
	rover_attitude_setpoint.timestamp = _timestamp;
	rover_attitude_setpoint.yaw_setpoint = yaw_setpoint;
	_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
}

void DifferentialPosVelControl::updateAutoSubscriptions()
{
	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		position_setpoint_triplet_s position_setpoint_triplet{};
		_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

		RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
				_curr_pos_ned, _home_position, _global_ned_proj_ref);

		_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);

		// Waypoint cruising speed
		_cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
					  position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}
}

float DifferentialPosVelControl::calcSpeedSetpoint(const float cruising_speed, const float distance_to_curr_wp,
		const float max_decel, const float max_jerk, const float waypoint_transition_angle, const float max_speed,
		const float trans_drv_trn, const float miss_spd_gain)
{
	float speed_body_x_setpoint = cruising_speed;

	if (_waypoint_transition_angle < M_PI_F - trans_drv_trn && max_jerk > FLT_EPSILON && max_decel > FLT_EPSILON) {
		speed_body_x_setpoint = math::trajectory::computeMaxSpeedFromDistance(max_jerk, max_decel, distance_to_curr_wp, 0.0f);

	} else if (_waypoint_transition_angle >= M_PI_F - trans_drv_trn && max_jerk > FLT_EPSILON && max_decel > FLT_EPSILON
		   && miss_spd_gain > FLT_EPSILON) {
		const float speed_reduction = math::constrain(miss_spd_gain * math::interpolate(M_PI_F - _waypoint_transition_angle,
					      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
		speed_body_x_setpoint = math::trajectory::computeMaxSpeedFromDistance(max_jerk, max_decel, distance_to_curr_wp,
					max_speed * (1.f - speed_reduction));
	}

	return math::constrain(speed_body_x_setpoint, -cruising_speed, cruising_speed);

}

bool DifferentialPosVelControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float>(events::ID("differential_posVel_control_conf_invalid_speed_lim"), events::Log::Error,
					    "Invalid configuration of necessary parameter RO_SPEED_LIM", _param_ro_speed_limit.get());
		}

	}

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON && _param_ro_speed_p.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float, float>(events::ID("differential_posVel_control_conf_invalid_speed_control"), events::Log::Error,
						   "Invalid configuration for speed control: Neither feed forward (RO_MAX_THR_SPEED) nor feedback (RO_SPEED_P) is setup",
						   _param_ro_max_thr_speed.get(),
						   _param_ro_speed_p.get());
		}
	}

	_prev_param_check_passed = ret;
	return ret;
}
