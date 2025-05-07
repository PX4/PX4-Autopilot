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
	_rover_velocity_setpoint_pub.advertise();
	_pure_pursuit_status_pub.advertise();

	updateParams();
}

void MecanumPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

}

void MecanumPosControl::updatePosControl()
{
	updateSubscriptions();

	hrt_abstime timestamp = hrt_absolute_time();

	if (_rover_position_setpoint_sub.updated()) {
		_rover_position_setpoint_sub.copy(&_rover_position_setpoint);
		_start_ned = Vector2f(_rover_position_setpoint.start_ned[0], _rover_position_setpoint.start_ned[1]);
		_start_ned = _start_ned.isAllFinite() ? _start_ned : _curr_pos_ned;
		_yaw_setpoint = PX4_ISFINITE(_rover_position_setpoint.yaw) ? _rover_position_setpoint.yaw : _vehicle_yaw;
	}

	const Vector2f target_waypoint_ned(_rover_position_setpoint.position_ned[0], _rover_position_setpoint.position_ned[1]);

	if (target_waypoint_ned.isAllFinite()) {

		float distance_to_target = (target_waypoint_ned - _curr_pos_ned).norm();

		if (distance_to_target > _param_nav_acc_rad.get()) {
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
			pure_pursuit_status.timestamp = timestamp;

			const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
						       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _start_ned,
						       _curr_pos_ned, fabsf(speed_setpoint));
			_pure_pursuit_status_pub.publish(pure_pursuit_status);
			rover_velocity_setpoint_s rover_velocity_setpoint{};
			rover_velocity_setpoint.timestamp = timestamp;
			rover_velocity_setpoint.speed = speed_setpoint;
			rover_velocity_setpoint.bearing = speed_setpoint > -FLT_EPSILON ? bearing_setpoint : matrix::wrap_pi(
					bearing_setpoint + M_PI_F);
			rover_velocity_setpoint.yaw = _yaw_setpoint;
			_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

		} else {
			rover_velocity_setpoint_s rover_velocity_setpoint{};
			rover_velocity_setpoint.timestamp = timestamp;
			rover_velocity_setpoint.speed = 0.f;
			rover_velocity_setpoint.bearing = _vehicle_yaw;
			_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);
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

		if (!_global_ned_proj_ref.isInitialized()
		    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
							   vehicle_local_position.ref_timestamp);
		}

		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
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

	return ret;
}
