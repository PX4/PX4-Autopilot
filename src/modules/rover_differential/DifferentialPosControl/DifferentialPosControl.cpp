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

#include "DifferentialPosControl.hpp"

using namespace time_literals;

DifferentialPosControl::DifferentialPosControl(ModuleParams *parent) : ModuleParams(parent)
{
	_pure_pursuit_status_pub.advertise();
	_rover_position_setpoint_pub.advertise();
	_rover_velocity_setpoint_pub.advertise();

	updateParams();
}

void DifferentialPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
}

void DifferentialPosControl::updatePosControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	if (_vehicle_control_mode.flag_control_position_enabled && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		if (_vehicle_control_mode.flag_control_offboard_enabled) {
			generatePositionSetpoint();

		} else 	if (_vehicle_control_mode.flag_control_manual_enabled && _vehicle_control_mode.flag_control_position_enabled) {
			manualPositionMode();

		} else if (_vehicle_control_mode.flag_control_auto_enabled) {
			autoPositionMode();

		}

		generateVelocitySetpoint();

	}

}

void DifferentialPosControl::updateSubscriptions()
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
		Vector3f velocity_ned(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_xyz = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_ned);
		Vector2f velocity_2d = Vector2f(velocity_xyz(0), velocity_xyz(1));
		_vehicle_speed = velocity_2d.norm() > _param_ro_speed_th.get() ? sign(velocity_2d(0)) * velocity_2d.norm() : 0.f;
	}

}

void DifferentialPosControl::generatePositionSetpoint()
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
	rover_position_setpoint.timestamp = hrt_absolute_time();
	rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
	rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
	rover_position_setpoint.start_ned[0] = NAN;
	rover_position_setpoint.start_ned[1] = NAN;
	rover_position_setpoint.cruising_speed = NAN;
	rover_position_setpoint.arrival_speed = NAN;
	rover_position_setpoint.yaw = NAN;
	_rover_position_setpoint_pub.publish(rover_position_setpoint);

}

void DifferentialPosControl::generateVelocitySetpoint()
{
	if (_rover_position_setpoint_sub.updated()) {
		_rover_position_setpoint_sub.copy(&_rover_position_setpoint);
		_start_ned = Vector2f(_rover_position_setpoint.start_ned[0], _rover_position_setpoint.start_ned[1]);
		_start_ned = _start_ned.isAllFinite() ? _start_ned : _curr_pos_ned;
	}

	const Vector2f target_waypoint_ned(_rover_position_setpoint.position_ned[0], _rover_position_setpoint.position_ned[1]);
	float distance_to_target = target_waypoint_ned.isAllFinite() ? (target_waypoint_ned - _curr_pos_ned).norm() : NAN;

	if (PX4_ISFINITE(distance_to_target) && distance_to_target > _param_nav_acc_rad.get()) {

		float arrival_speed = PX4_ISFINITE(_rover_position_setpoint.arrival_speed) ? _rover_position_setpoint.arrival_speed :
				      0.f;
		const float distance = arrival_speed > 0.f + FLT_EPSILON ? distance_to_target - _param_nav_acc_rad.get() :
				       distance_to_target;
		float speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
				       _param_ro_decel_limit.get(), distance, fabsf(arrival_speed));
		speed_setpoint = math::min(speed_setpoint, _param_ro_speed_limit.get());

		if (PX4_ISFINITE(_rover_position_setpoint.cruising_speed)) {
			speed_setpoint = sign(_rover_position_setpoint.cruising_speed) * math::min(speed_setpoint,
					 fabsf(_rover_position_setpoint.cruising_speed));
		}

		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;

		const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _start_ned,
					   _curr_pos_ned, fabsf(speed_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = speed_setpoint;
		rover_velocity_setpoint.bearing = speed_setpoint > -FLT_EPSILON ? yaw_setpoint : matrix::wrap_pi(
				yaw_setpoint + M_PI_F);
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	} else {
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = 0.f;
		rover_velocity_setpoint.bearing = _vehicle_yaw;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);
	}

}

void DifferentialPosControl::manualPositionMode()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	const float speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
				     -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
	const float bearing_scaling = math::min(_max_yaw_rate / _param_ro_yaw_p.get(),
						_param_rd_trans_drv_trn.get() - FLT_EPSILON);
	const float bearing_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
				    _param_ro_yaw_stick_dz.get()), -1.f, 1.f, -bearing_scaling, bearing_scaling);

	if (fabsf(bearing_delta) > FLT_EPSILON || fabsf(speed_setpoint) < FLT_EPSILON) {
		_pos_ctl_course_direction = Vector2f(NAN, NAN);
		// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
		const float yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + bearing_delta);
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

void DifferentialPosControl::autoPositionMode()
{
	if (_position_setpoint_triplet_sub.updated()) {
		position_setpoint_triplet_s position_setpoint_triplet{};
		_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);
		_curr_wp_type = position_setpoint_triplet.current.type;

		RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
				_curr_pos_ned, _global_ned_proj_ref);

		_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);

		// Waypoint cruising speed
		_cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
					  position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = _curr_wp_ned(0);
		rover_position_setpoint.position_ned[1] = _curr_wp_ned(1);
		rover_position_setpoint.start_ned[0] = _prev_wp_ned(0);
		rover_position_setpoint.start_ned[1] = _prev_wp_ned(1);
		rover_position_setpoint.arrival_speed = autoArrivalSpeed(_cruising_speed, _waypoint_transition_angle,
							_param_ro_speed_limit.get(), _param_rd_trans_drv_trn.get(), _param_rd_miss_spd_gain.get(), _curr_wp_type);
		rover_position_setpoint.cruising_speed = _cruising_speed;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}
}

float DifferentialPosControl::autoArrivalSpeed(const float cruising_speed, const float waypoint_transition_angle,
		const float max_speed, const float trans_drv_trn, const float miss_spd_gain, int curr_wp_type)
{
	// Upcoming stop
	if (!PX4_ISFINITE(waypoint_transition_angle) || waypoint_transition_angle < M_PI_F - trans_drv_trn
	    || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		return 0.f;
	}

	// Straight line speed
	if (miss_spd_gain > FLT_EPSILON) {
		const float speed_reduction = math::constrain(miss_spd_gain * math::interpolate(M_PI_F - waypoint_transition_angle,
					      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
		return max_speed * (1.f - speed_reduction);
	}

	return cruising_speed; // Fallthrough

}

bool DifferentialPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	_prev_param_check_passed = ret;
	return ret;
}
