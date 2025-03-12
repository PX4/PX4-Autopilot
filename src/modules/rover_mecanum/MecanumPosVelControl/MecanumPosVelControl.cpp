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

#include "MecanumPosVelControl.hpp"

using namespace time_literals;

MecanumPosVelControl::MecanumPosVelControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_velocity_status_pub.advertise();
	_pure_pursuit_status_pub.advertise();
	updateParams();
}

void MecanumPosVelControl::updateParams()
{
	ModuleParams::updateParams();
	_pid_speed_x.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed_x.setIntegralLimit(1.f);
	_pid_speed_x.setOutputLimit(1.f);
	_pid_speed_y.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed_y.setIntegralLimit(1.f);
	_pid_speed_y.setOutputLimit(1.f);
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

	if (_param_ro_accel_limit.get() > FLT_EPSILON) {
		_speed_x_setpoint.setSlewRate(_param_ro_accel_limit.get());
		_speed_y_setpoint.setSlewRate(_param_ro_accel_limit.get());
	}
}

void MecanumPosVelControl::updatePosControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	if ((_vehicle_control_mode.flag_control_position_enabled || _vehicle_control_mode.flag_control_velocity_enabled)
	    && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		generateAttitudeSetpoint();

		if (_param_ro_max_thr_speed.get() > FLT_EPSILON) { // Adjust speed setpoints if infeasible
			if (_rover_steering_setpoint_sub.updated()) {
				_rover_steering_setpoint_sub.copy(&_rover_steering_setpoint);
			}

			float speed_body_x_setpoint_normalized = math::interpolate<float>(_speed_body_x_setpoint,
					-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

			float speed_body_y_setpoint_normalized = math::interpolate<float>(_speed_body_y_setpoint,
					-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

			const float total_speed = fabsf(speed_body_x_setpoint_normalized) + fabsf(speed_body_y_setpoint_normalized) + fabsf(
							  _rover_steering_setpoint.normalized_speed_diff);

			if (total_speed > 1.f) {
				const float theta = atan2f(fabsf(speed_body_y_setpoint_normalized), fabsf(speed_body_x_setpoint_normalized));
				const float magnitude = (1.f - fabsf(_rover_steering_setpoint.normalized_speed_diff)) / (sinf(theta) + cosf(theta));
				const float normalization = 1.f / (sqrtf(powf(speed_body_x_setpoint_normalized,
								   2.f) + powf(speed_body_y_setpoint_normalized, 2.f)));
				speed_body_x_setpoint_normalized *= magnitude * normalization;
				speed_body_y_setpoint_normalized *= magnitude * normalization;
				_speed_body_x_setpoint = math::interpolate<float>(speed_body_x_setpoint_normalized, -1.f, 1.f,
							 -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
				_speed_body_y_setpoint = math::interpolate<float>(speed_body_y_setpoint_normalized, -1.f, 1.f,
							 -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
			}
		}

		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = _timestamp;
		_speed_body_x_setpoint = fabsf(_speed_body_x_setpoint) > _param_ro_speed_th.get() ? _speed_body_x_setpoint : 0.f;
		_speed_body_y_setpoint = fabsf(_speed_body_y_setpoint) > _param_ro_speed_th.get() ? _speed_body_y_setpoint : 0.f;
		rover_throttle_setpoint.throttle_body_x = RoverControl::speedControl(_speed_x_setpoint, _pid_speed_x,
				_speed_body_x_setpoint, _vehicle_speed_body_x, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
				_param_ro_max_thr_speed.get(), _dt);
		rover_throttle_setpoint.throttle_body_y = RoverControl::speedControl(_speed_y_setpoint, _pid_speed_y,
				_speed_body_y_setpoint, _vehicle_speed_body_y, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
				_param_ro_max_thr_speed.get(), _dt);
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	} else { // Reset controller and slew rate when position control is not active
		_pid_speed_x.resetIntegral();
		_speed_x_setpoint.setForcedValue(0.f);
		_pid_speed_y.resetIntegral();
		_speed_y_setpoint.setForcedValue(0.f);
	}

	// Publish position controller status (logging only)
	rover_velocity_status_s rover_velocity_status;
	rover_velocity_status.timestamp = _timestamp;
	rover_velocity_status.measured_speed_body_x = _vehicle_speed_body_x;
	rover_velocity_status.speed_body_x_setpoint = _speed_body_x_setpoint;
	rover_velocity_status.adjusted_speed_body_x_setpoint = _speed_x_setpoint.getState();
	rover_velocity_status.measured_speed_body_y = _vehicle_speed_body_y;
	rover_velocity_status.speed_body_y_setpoint = _speed_body_y_setpoint;
	rover_velocity_status.adjusted_speed_body_y_setpoint = _speed_y_setpoint.getState();
	rover_velocity_status.pid_throttle_body_x_integral = _pid_speed_x.getIntegral();
	rover_velocity_status.pid_throttle_body_y_integral = _pid_speed_y.getIntegral();
	_rover_velocity_status_pub.publish(rover_velocity_status);
}

void MecanumPosVelControl::updateSubscriptions()
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
		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_speed_body_x = fabsf(velocity_in_body_frame(0)) > _param_ro_speed_th.get() ? velocity_in_body_frame(0) : 0.f;
		_vehicle_speed_body_y = fabsf(velocity_in_body_frame(1)) > _param_ro_speed_th.get() ? velocity_in_body_frame(1) : 0.f;
	}

}

void MecanumPosVelControl::generateAttitudeSetpoint()
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

void MecanumPosVelControl::manualPositionMode()
{
	manual_control_setpoint_s manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		_speed_body_x_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
					 -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
		_speed_body_y_setpoint = math::interpolate<float>(manual_control_setpoint.roll,
					 -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
		const float yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.yaw,
						_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);

		if (fabsf(yaw_rate_setpoint) > FLT_EPSILON) { // Closed loop yaw rate control
			_pos_ctl_yaw_setpoint = NAN;
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = yaw_rate_setpoint;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

		} else if ((fabsf(_speed_body_x_setpoint) > FLT_EPSILON
			    || fabsf(_speed_body_y_setpoint) >
			    FLT_EPSILON)) { // Course control if the steering input is zero (keep driving on a straight line)
			const Vector3f velocity = Vector3f(_speed_body_x_setpoint, _speed_body_y_setpoint, 0.f);
			const float velocity_magnitude_setpoint = velocity.norm();
			const Vector3f pos_ctl_course_direction_local = _vehicle_attitude_quaternion.rotateVector(velocity.normalized());
			const Vector2f pos_ctl_course_direction_temp = Vector2f(pos_ctl_course_direction_local(0),
					pos_ctl_course_direction_local(1));

			// Reset course control if course direction change is above threshold
			if (fabsf(asinf(pos_ctl_course_direction_temp % _pos_ctl_course_direction)) > _param_rm_course_ctl_th.get()) {
				_pos_ctl_yaw_setpoint = NAN;
			}

			if (!PX4_ISFINITE(_pos_ctl_yaw_setpoint)) {
				_pos_ctl_start_position_ned = _curr_pos_ned;
				_pos_ctl_yaw_setpoint = _vehicle_yaw;
				_pos_ctl_course_direction = pos_ctl_course_direction_temp;
			}

			// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
			const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
			const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_pp_lookahd_max.get();
			const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + vector_scaling * _pos_ctl_course_direction;
			pure_pursuit_status_s pure_pursuit_status{};
			pure_pursuit_status.timestamp = _timestamp;
			const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
						       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _pos_ctl_start_position_ned,
						       _curr_pos_ned, velocity_magnitude_setpoint);
			_pure_pursuit_status_pub.publish(pure_pursuit_status);
			const float bearing_setpoint_body_frame = matrix::wrap_pi(bearing_setpoint - _vehicle_yaw);
			_speed_body_x_setpoint = velocity_magnitude_setpoint * cosf(bearing_setpoint_body_frame);
			_speed_body_y_setpoint = velocity_magnitude_setpoint * sinf(bearing_setpoint_body_frame);
			rover_attitude_setpoint_s rover_attitude_setpoint{};
			rover_attitude_setpoint.timestamp = _timestamp;
			rover_attitude_setpoint.yaw_setpoint = _pos_ctl_yaw_setpoint;
			_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		} else { // Reset course control and yaw rate setpoint
			_pos_ctl_yaw_setpoint = NAN;
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = 0.f;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
		}
	}
}

void MecanumPosVelControl::offboardPositionMode()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	// Translate trajectory setpoint to rover setpoints
	const Vector2f target_waypoint_ned(trajectory_setpoint.position[0], trajectory_setpoint.position[1]);
	const float distance_to_target = (target_waypoint_ned - _curr_pos_ned).norm();

	if (target_waypoint_ned.isAllFinite() && distance_to_target > _param_nav_acc_rad.get()) {
		const float velocity_magnitude_setpoint = math::min(math::trajectory::computeMaxSpeedFromDistance(
					_param_ro_jerk_limit.get(),
					_param_ro_decel_limit.get(), distance_to_target, 0.f), _param_ro_speed_limit.get());
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _curr_pos_ned,
					       _curr_pos_ned, velocity_magnitude_setpoint);
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		const float bearing_setpoint_body_frame = matrix::wrap_pi(bearing_setpoint - _vehicle_yaw);
		_speed_body_x_setpoint = velocity_magnitude_setpoint * cosf(bearing_setpoint_body_frame);
		_speed_body_y_setpoint = velocity_magnitude_setpoint * sinf(bearing_setpoint_body_frame);

	} else {
		_speed_body_x_setpoint = 0.f;
		_speed_body_y_setpoint = 0.f;
	}
}

void MecanumPosVelControl::offboardVelocityMode()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	const Vector3f velocity_in_local_frame(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1],
					       trajectory_setpoint.velocity[2]);
	const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);

	if (velocity_in_body_frame.isAllFinite()) {
		_speed_body_x_setpoint = velocity_in_body_frame(0);
		_speed_body_y_setpoint = velocity_in_body_frame(1);

	} else {
		_speed_body_x_setpoint = 0.f;
		_speed_body_y_setpoint = 0.f;
	}
}

void MecanumPosVelControl::autoPositionMode()
{
	if (_position_setpoint_triplet_sub.updated()) {
		position_setpoint_triplet_s position_setpoint_triplet{};
		_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);
		_curr_wp_type = position_setpoint_triplet.current.type;

		RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
				_curr_pos_ned, _global_ned_proj_ref);

		_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);

		// Waypoint cruising speed
		_auto_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
				      position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();

		// Waypoint yaw setpoint
		if (PX4_ISFINITE(position_setpoint_triplet.current.yaw)) {
			_auto_yaw = position_setpoint_triplet.current.yaw;

		} else {
			_auto_yaw = _vehicle_yaw;
		}
	}

	const float distance_to_curr_wp = sqrt(powf(_curr_pos_ned(0) - _curr_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _curr_wp_ned(1), 2));

	// Check stopping conditions
	bool auto_stop{false};

	if (_curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || _curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE
	    || !_next_wp_ned.isAllFinite()) { // Check stopping conditions
		auto_stop = distance_to_curr_wp < _param_nav_acc_rad.get();
	}

	if (auto_stop) {
		_speed_body_x_setpoint = 0.f;
		_speed_body_y_setpoint = 0.f;
		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = _timestamp;
		rover_rate_setpoint.yaw_rate_setpoint = 0.f;
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

	} else { // Regular guidance algorithm
		const float velocity_magnitude = calcVelocityMagnitude(_auto_speed, distance_to_curr_wp, _param_ro_decel_limit.get(),
						 _param_ro_jerk_limit.get(), _waypoint_transition_angle, _param_ro_speed_limit.get(), _param_rm_miss_spd_gain.get(),
						 _curr_wp_type);
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
					       velocity_magnitude);
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		const float bearing_setpoint_body_frame = matrix::wrap_pi(bearing_setpoint - _vehicle_yaw);
		Vector2f desired_velocity(0.f, 0.f);
		_speed_body_x_setpoint = velocity_magnitude * cosf(bearing_setpoint_body_frame);
		_speed_body_y_setpoint = velocity_magnitude * sinf(bearing_setpoint_body_frame);
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		rover_attitude_setpoint.yaw_setpoint = _auto_yaw;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
	}
}

float MecanumPosVelControl::calcVelocityMagnitude(const float auto_speed, const float distance_to_curr_wp,
		const float max_decel, const float max_jerk, const float waypoint_transition_angle, const float max_speed,
		const float miss_spd_gain, const int curr_wp_type)
{
	// Upcoming stop
	if (max_decel > FLT_EPSILON && max_jerk > FLT_EPSILON && (!PX4_ISFINITE(waypoint_transition_angle)
			|| curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
			|| curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {
		const float max_velocity_magnitude = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
						     max_decel, distance_to_curr_wp, 0.f);
		return math::constrain(max_velocity_magnitude, -auto_speed, auto_speed);
	}

	// Straight line speed
	if (max_jerk > FLT_EPSILON && max_decel > FLT_EPSILON && miss_spd_gain > FLT_EPSILON) {
		const float speed_reduction = math::constrain(miss_spd_gain * math::interpolate(M_PI_F - waypoint_transition_angle, 0.f,
					      M_PI_F, 0.f, 1.f), 0.f, 1.f);
		const float max_velocity_magnitude = math::trajectory::computeMaxSpeedFromDistance(max_jerk, max_decel,
						     distance_to_curr_wp,
						     max_speed * (1.f - speed_reduction));

		return math::constrain(max_velocity_magnitude, -auto_speed, auto_speed);
	}

	return auto_speed; // Fallthrough
}

bool MecanumPosVelControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float>(events::ID("mecanum_posVel_control_conf_invalid_speed_lim"), events::Log::Error,
					    "Invalid configuration of necessary parameter RO_SPEED_LIM", _param_ro_speed_limit.get());
		}

	}

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON && _param_ro_speed_p.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float, float>(events::ID("mecanum_posVel_control_conf_invalid_speed_control"), events::Log::Error,
						   "Invalid configuration for speed control: Neither feed forward (RO_MAX_THR_SPD) nor feedback (RO_SPEED_P) is setup",
						   _param_ro_max_thr_speed.get(),
						   _param_ro_speed_p.get());
		}
	}

	_prev_param_check_passed = ret;
	return ret;
}
