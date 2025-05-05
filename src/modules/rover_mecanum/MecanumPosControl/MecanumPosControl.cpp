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

#include "MecanumPosControl.hpp"

using namespace time_literals;

MecanumPosControl::MecanumPosControl(ModuleParams *parent) : ModuleParams(parent)
{
	_mecanum_velocity_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
	_pure_pursuit_status_pub.advertise();

	// Initially set to NaN to indicate that the rover has no position setpoint
	_rover_position_setpoint.position_ned[0] = NAN;
	_rover_position_setpoint.position_ned[1] = NAN;

	updateParams();
}

void MecanumPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

}

void MecanumPosControl::updatePosControl()
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

void MecanumPosControl::updateSubscriptions()
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

void MecanumPosControl::generatePositionSetpoint()
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
	rover_position_setpoint.yaw = _vehicle_yaw;
	_rover_position_setpoint_pub.publish(rover_position_setpoint);

}

void MecanumPosControl::generateVelocitySetpoint()
{
	if (_vehicle_control_mode.flag_control_manual_enabled && _vehicle_control_mode.flag_control_position_enabled) {
		manualPositionMode();

	} else if (_vehicle_control_mode.flag_control_auto_enabled) {
		autoPositionMode();

	} else if (_rover_position_setpoint_sub.copy(&_rover_position_setpoint)
		   && PX4_ISFINITE(_rover_position_setpoint.position_ned[0]) && PX4_ISFINITE(_rover_position_setpoint.position_ned[1])) {
		goToPositionMode();
	}

}

void MecanumPosControl::manualPositionMode()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	Vector3f velocity_setpoint_body{};
	velocity_setpoint_body(0) = math::interpolate<float>(manual_control_setpoint.throttle,
				    -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
	velocity_setpoint_body(1) = math::interpolate<float>(manual_control_setpoint.roll,
				    -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
	const float yaw_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.yaw,
				_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate / _param_ro_yaw_p.get(),
				_max_yaw_rate / _param_ro_yaw_p.get());

	if (fabsf(yaw_delta) > FLT_EPSILON || velocity_setpoint_body.norm() < FLT_EPSILON) { // Closed loop yaw rate control
		_pos_ctl_yaw_setpoint = NAN;
		const float yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + yaw_delta);
		const Vector3f velocity_setpoint_local = _vehicle_attitude_quaternion.rotateVector(velocity_setpoint_body);
		mecanum_velocity_setpoint_s mecanum_velocity_setpoint{};
		mecanum_velocity_setpoint.timestamp = _timestamp;
		mecanum_velocity_setpoint.speed = velocity_setpoint_body.norm();
		mecanum_velocity_setpoint.bearing = atan2f(velocity_setpoint_local(1), velocity_setpoint_local(0));
		mecanum_velocity_setpoint.yaw = yaw_setpoint;
		_mecanum_velocity_setpoint_pub.publish(mecanum_velocity_setpoint);

	} else { // Course control if the steering input is zero (keep driving on a straight line)
		const Vector3f velocity = Vector3f(velocity_setpoint_body(0), velocity_setpoint_body(1), 0.f);
		const float velocity_magnitude_setpoint = velocity.norm();
		const Vector3f pos_ctl_course_direction_local = _vehicle_attitude_quaternion.rotateVector(velocity.normalized());
		const Vector2f pos_ctl_course_direction_temp = Vector2f(pos_ctl_course_direction_local(0),
				pos_ctl_course_direction_local(1));

		// Reset course control if course direction change is above threshold
		if (fabsf(acosf(pos_ctl_course_direction_temp(0) * _pos_ctl_course_direction(0) + pos_ctl_course_direction_temp(
					1) * _pos_ctl_course_direction(1))) > _param_rm_course_ctl_th.get()) {
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
		mecanum_velocity_setpoint_s mecanum_velocity_setpoint{};
		mecanum_velocity_setpoint.timestamp = _timestamp;
		mecanum_velocity_setpoint.speed = velocity_magnitude_setpoint;
		mecanum_velocity_setpoint.bearing = bearing_setpoint;
		mecanum_velocity_setpoint.yaw = _pos_ctl_yaw_setpoint;
		_mecanum_velocity_setpoint_pub.publish(mecanum_velocity_setpoint);
	}
}

void MecanumPosControl::autoPositionMode()
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
		mecanum_velocity_setpoint_s mecanum_velocity_setpoint{};
		mecanum_velocity_setpoint.timestamp = _timestamp;
		mecanum_velocity_setpoint.speed = 0.f;
		mecanum_velocity_setpoint.bearing = 0.f;
		mecanum_velocity_setpoint.yaw = _vehicle_yaw;
		_mecanum_velocity_setpoint_pub.publish(mecanum_velocity_setpoint);

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
		mecanum_velocity_setpoint_s mecanum_velocity_setpoint{};
		mecanum_velocity_setpoint.timestamp = _timestamp;
		mecanum_velocity_setpoint.speed = velocity_magnitude;
		mecanum_velocity_setpoint.bearing = bearing_setpoint;
		mecanum_velocity_setpoint.yaw = _auto_yaw;
		_mecanum_velocity_setpoint_pub.publish(mecanum_velocity_setpoint);
	}
}


float MecanumPosControl::calcVelocityMagnitude(const float auto_speed, const float distance_to_curr_wp,
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

void MecanumPosControl::goToPositionMode()
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
		const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _curr_pos_ned,
					       _curr_pos_ned, fabsf(speed_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		mecanum_velocity_setpoint_s mecanum_velocity_setpoint{};
		mecanum_velocity_setpoint.timestamp = _timestamp;
		mecanum_velocity_setpoint.speed = speed_setpoint;
		mecanum_velocity_setpoint.bearing = bearing_setpoint;
		mecanum_velocity_setpoint.yaw = _pos_ctl_yaw_setpoint;
		_mecanum_velocity_setpoint_pub.publish(mecanum_velocity_setpoint);

	} else {
		mecanum_velocity_setpoint_s mecanum_velocity_setpoint{};
		mecanum_velocity_setpoint.timestamp = _timestamp;
		mecanum_velocity_setpoint.speed = 0.f;
		mecanum_velocity_setpoint.bearing = 0.f;
		mecanum_velocity_setpoint.yaw = _vehicle_yaw;
		_mecanum_velocity_setpoint_pub.publish(mecanum_velocity_setpoint);
	}
}

bool MecanumPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON && _param_ro_speed_p.get() < FLT_EPSILON) {
		ret = false;
	}

	_prev_param_check_passed = ret;
	return ret;
}
