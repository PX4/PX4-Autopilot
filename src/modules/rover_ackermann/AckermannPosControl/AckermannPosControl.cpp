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
	_rover_speed_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();

	updateParams();
}

void AckermannPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
	_min_speed = _param_ra_wheel_base.get() * _max_yaw_rate / tanf(_param_ra_max_str_ang.get());

}

void AckermannPosControl::updatePosControl()
{
	updateSubscriptions();

	hrt_abstime timestamp = hrt_absolute_time();

	if (_target_waypoint_ned.isAllFinite()) {
		float distance_to_target = (_target_waypoint_ned - _curr_pos_ned).norm();

		if (_arrival_speed > FLT_EPSILON) {
			distance_to_target -= _acceptance_radius; // shift target to the edge of the acceptance radius if arrival speed not zero
		}

		if (distance_to_target > _acceptance_radius || _arrival_speed > FLT_EPSILON) {

			float speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
					       _param_ro_decel_limit.get(), distance_to_target, fabsf(_arrival_speed));
			speed_setpoint = math::min(speed_setpoint, _cruising_speed);

			pure_pursuit_status_s pure_pursuit_status{};
			pure_pursuit_status.timestamp = timestamp;

			const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
						       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _target_waypoint_ned, _start_ned,
						       _curr_pos_ned, fabsf(speed_setpoint));

			if (_param_ro_speed_red.get() > FLT_EPSILON) {
				const float course_error = fabsf(matrix::wrap_pi(bearing_setpoint - _vehicle_yaw));
				const float speed_reduction = math::constrain(_param_ro_speed_red.get() * math::interpolate(course_error,
							      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
				const float max_speed = math::constrain(_param_ro_max_thr_speed.get() * (1.f - speed_reduction), _min_speed,
									_param_ro_max_thr_speed.get());
				speed_setpoint = math::constrain(speed_setpoint, -max_speed, max_speed);
			}

			_pure_pursuit_status_pub.publish(pure_pursuit_status);
			rover_speed_setpoint_s rover_speed_setpoint{};
			rover_speed_setpoint.timestamp = timestamp;
			rover_speed_setpoint.speed_body_x = speed_setpoint;
			_rover_speed_setpoint_pub.publish(rover_speed_setpoint);
			rover_attitude_setpoint_s rover_attitude_setpoint{};
			rover_attitude_setpoint.timestamp = timestamp;
			rover_attitude_setpoint.yaw_setpoint = speed_setpoint > -FLT_EPSILON ? bearing_setpoint : matrix::wrap_pi(
					bearing_setpoint + M_PI_F);
			_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		} else {
			rover_speed_setpoint_s rover_speed_setpoint{};
			rover_speed_setpoint.timestamp = timestamp;
			rover_speed_setpoint.speed_body_x = 0.f;
			_rover_speed_setpoint_pub.publish(rover_speed_setpoint);
			rover_attitude_setpoint_s rover_attitude_setpoint{};
			rover_attitude_setpoint.timestamp = timestamp;
			rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw;
			_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

			if (!_stopped && fabsf(_vehicle_speed) < FLT_EPSILON) {
				_stopped = true;
				_target_waypoint_ned = _curr_pos_ned;
			}

			if (_stopped && _updated_reset_counter != _reset_counter) {
				_target_waypoint_ned = _curr_pos_ned;
				_reset_counter = _updated_reset_counter;
			}
		}
	}
}

void AckermannPosControl::updateSubscriptions()
{
	if (_position_controller_status_sub.updated()) {
		position_controller_status_s position_controller_status{};
		_position_controller_status_sub.copy(&position_controller_status);
		_acceptance_radius = position_controller_status.acceptance_radius;
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
		_updated_reset_counter = vehicle_local_position.xy_reset_counter;
		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		Vector3f velocity_ned(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_xyz = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_ned);
		Vector2f velocity_2d = Vector2f(velocity_xyz(0), velocity_xyz(1));
		_vehicle_speed = velocity_2d.norm() > _param_ro_speed_th.get() ? sign(velocity_2d(0)) * velocity_2d.norm() : 0.f;
	}

	if (_rover_position_setpoint_sub.updated()) {
		rover_position_setpoint_s rover_position_setpoint;
		_rover_position_setpoint_sub.copy(&rover_position_setpoint);
		_start_ned = Vector2f(rover_position_setpoint.start_ned[0], rover_position_setpoint.start_ned[1]);
		_start_ned = _start_ned.isAllFinite() ? _start_ned : _curr_pos_ned;
		_arrival_speed = PX4_ISFINITE(rover_position_setpoint.arrival_speed) ? rover_position_setpoint.arrival_speed : 0.f;
		_cruising_speed = PX4_ISFINITE(rover_position_setpoint.cruising_speed) ? rover_position_setpoint.cruising_speed :
				  _param_ro_speed_limit.get();
		_target_waypoint_ned = Vector2f(rover_position_setpoint.position_ned[0], rover_position_setpoint.position_ned[1]);
		_stopped = false;
	}

}

bool AckermannPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}
