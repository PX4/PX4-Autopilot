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
	_surface_vehicle_speed_setpoint_pub.advertise();
	_pure_pursuit_status_pub.advertise();
	_surface_vehicle_attitude_setpoint_pub.advertise();

	updateParams();
}

void MecanumPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_sv_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

}

void MecanumPosControl::updatePosControl()
{
	updateSubscriptions();

	hrt_abstime timestamp = hrt_absolute_time();

	if (_target_waypoint_ned.isAllFinite()) {

		float distance_to_target = (_target_waypoint_ned - _curr_pos_ned).norm();

		if (_arrival_speed > FLT_EPSILON) {
			distance_to_target -=
				_param_nav_acc_rad.get(); // shift target to the edge of the acceptance radius if arrival speed not zero
		}

		if (distance_to_target > _param_nav_acc_rad.get() || _arrival_speed > FLT_EPSILON) {

			float speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_sv_jerk_limit.get(),
					       _param_sv_decel_limit.get(), distance_to_target, fabsf(_arrival_speed));
			speed_setpoint = math::min(speed_setpoint, _cruising_speed);

			pure_pursuit_status_s pure_pursuit_status{};
			pure_pursuit_status.timestamp = timestamp;

			const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
						       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _target_waypoint_ned, _start_ned,
						       _curr_pos_ned, fabsf(speed_setpoint));
			_pure_pursuit_status_pub.publish(pure_pursuit_status);

			const Vector3f velocity_in_local_frame(speed_setpoint * cosf(bearing_setpoint),
							       speed_setpoint * sinf(bearing_setpoint), 0.f);
			const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);

			surface_vehicle_speed_setpoint_s surface_vehicle_speed_setpoint{};
			surface_vehicle_speed_setpoint.timestamp = timestamp;
			surface_vehicle_speed_setpoint.speed_body_x = velocity_in_body_frame(0);
			surface_vehicle_speed_setpoint.speed_body_y = velocity_in_body_frame(1);
			_surface_vehicle_speed_setpoint_pub.publish(surface_vehicle_speed_setpoint);
			surface_vehicle_attitude_setpoint_s surface_vehicle_attitude_setpoint{};
			surface_vehicle_attitude_setpoint.timestamp = timestamp;
			surface_vehicle_attitude_setpoint.yaw_setpoint = _yaw_setpoint;
			_surface_vehicle_attitude_setpoint_pub.publish(surface_vehicle_attitude_setpoint);

		} else {
			surface_vehicle_speed_setpoint_s surface_vehicle_speed_setpoint{};
			surface_vehicle_speed_setpoint.timestamp = timestamp;
			surface_vehicle_speed_setpoint.speed_body_x = 0.f;
			surface_vehicle_speed_setpoint.speed_body_y = 0.f;
			_surface_vehicle_speed_setpoint_pub.publish(surface_vehicle_speed_setpoint);
			surface_vehicle_attitude_setpoint_s surface_vehicle_attitude_setpoint{};
			surface_vehicle_attitude_setpoint.timestamp = timestamp;
			surface_vehicle_attitude_setpoint.yaw_setpoint = _vehicle_yaw;
			_surface_vehicle_attitude_setpoint_pub.publish(surface_vehicle_attitude_setpoint);

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

void MecanumPosControl::updateSubscriptions()
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
		_updated_reset_counter = vehicle_local_position.xy_reset_counter;
		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		Vector3f velocity_ned(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_xyz = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_ned);
		Vector2f velocity_2d = Vector2f(velocity_xyz(0), velocity_xyz(1));
		_vehicle_speed = velocity_2d.norm() > _param_sv_speed_th.get() ? sign(velocity_2d(0)) * velocity_2d.norm() : 0.f;
	}

	if (_surface_vehicle_position_setpoint_sub.updated()) {
		surface_vehicle_position_setpoint_s surface_vehicle_position_setpoint;
		_surface_vehicle_position_setpoint_sub.copy(&surface_vehicle_position_setpoint);
		_start_ned = Vector2f(surface_vehicle_position_setpoint.start_ned[0], surface_vehicle_position_setpoint.start_ned[1]);
		_start_ned = _start_ned.isAllFinite() ? _start_ned : _curr_pos_ned;
		_arrival_speed = PX4_ISFINITE(surface_vehicle_position_setpoint.arrival_speed) ?
				 surface_vehicle_position_setpoint.arrival_speed : 0.f;
		_cruising_speed = PX4_ISFINITE(surface_vehicle_position_setpoint.cruising_speed) ?
				  surface_vehicle_position_setpoint.cruising_speed :
				  _param_sv_speed_limit.get();
		_target_waypoint_ned = Vector2f(surface_vehicle_position_setpoint.position_ned[0],
						surface_vehicle_position_setpoint.position_ned[1]);
		_stopped = false;
	}
}

bool MecanumPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_sv_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_sv_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_sv_max_thr_speed.get() < FLT_EPSILON && _param_sv_speed_p.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}
