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
	_rover_velocity_setpoint_pub.advertise();

	updateParams();
}

void DifferentialPosControl::updateParams()
{
	ModuleParams::updateParams();
}

void DifferentialPosControl::updatePosControl()
{
	updateSubscriptions();

	hrt_abstime timestamp = hrt_absolute_time();

	const Vector2f target_waypoint_ned(_rover_position_setpoint.position_ned[0], _rover_position_setpoint.position_ned[1]);

	if (target_waypoint_ned.isAllFinite()) {
		float distance_to_target = (target_waypoint_ned - _curr_pos_ned).norm();
		bool isArrivalFast = PX4_ISFINITE(_arrival_speed) && _arrival_speed > FLT_EPSILON;

		float distance_from_start = (_curr_pos_ned - _start_ned).norm();

		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = timestamp;

		if (isArrivalFast || distance_to_target > _param_nav_acc_rad.get()) {

			// apply PurePursuit - figure out desired yaw:
			const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
						   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned,
						   _start_ned, _curr_pos_ned,
						   fabsf(PX4_ISFINITE(_ground_speed_abs) ? _ground_speed_abs : 0.f));  // _speed_setpoint from last cycle can be used here

			const float heading_error = matrix::wrap_pi(yaw_setpoint - _vehicle_yaw);

			// if out yaw is not aligned well with the target waypoint, we need to turn on the spot:
			if (_current_state == DrivingState::DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
				_current_state = DrivingState::SPOT_TURNING;

			} else if (_current_state == DrivingState::SPOT_TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
				_current_state = DrivingState::DRIVING;
				_start_ned = _curr_pos_ned; // next trapezoid calculation will start here, where we stopped
			}

			if (_current_state == DrivingState::DRIVING) {

				// speed profile while DRIVING - trapezoid calculation:

				// assume we are driving towards the target waypoint and might be decelerating.
				// shift target to the edge of the acceptance radius if arrival speed not zero:
				//float arr_dep_distance = isArrivalFast ?  distance_to_target - _param_nav_acc_rad.get() : distance_to_target;
				float arr_dep_distance = distance_to_target - _param_rd_acc_rad_margin.get() * _param_nav_acc_rad.get();
				float arr_dep_speed = isArrivalFast ?  _arrival_speed : 0.f;
				float acc_dec_limit = _param_ro_decel_limit.get();

				if (distance_from_start > FLT_EPSILON && distance_from_start < distance_to_target) {
					// we are departing from the start point and accelerating:
					arr_dep_distance = distance_from_start;
					arr_dep_speed = _ground_speed_abs;
					acc_dec_limit = _param_ro_accel_limit.get();
				}

				if (arr_dep_distance < FLT_EPSILON) {
					arr_dep_distance = 0.f;
				}

				_speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
						  acc_dec_limit, arr_dep_distance, fabsf(arr_dep_speed));
				_speed_setpoint = math::min(_speed_setpoint, _param_ro_speed_limit.get());

				if (PX4_ISFINITE(_cruising_speed)) {
					_speed_setpoint = sign(_cruising_speed) * math::min(_speed_setpoint, fabsf(_cruising_speed));
				}

			} else {
				// speed profile while SPOT_TURNING - (TBD: small constant speed by parameter):
				_speed_setpoint = 0.f;
			}

			_pure_pursuit_status_pub.publish(pure_pursuit_status);

			rover_velocity_setpoint_s rover_velocity_setpoint{};
			rover_velocity_setpoint.timestamp = timestamp;
			rover_velocity_setpoint.speed = _speed_setpoint;
			rover_velocity_setpoint.bearing = _speed_setpoint > -FLT_EPSILON ? yaw_setpoint : matrix::wrap_pi(
					yaw_setpoint + M_PI_F);
			_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

		}  else {

			pure_pursuit_status.lookahead_distance = NAN;
			pure_pursuit_status.target_bearing = NAN;
			pure_pursuit_status.crosstrack_error = NAN;
			pure_pursuit_status.distance_to_waypoint = NAN;
			pure_pursuit_status.bearing_to_waypoint = NAN;
			_pure_pursuit_status_pub.publish(pure_pursuit_status);

			rover_velocity_setpoint_s rover_velocity_setpoint{};
			rover_velocity_setpoint.timestamp = timestamp;
			rover_velocity_setpoint.speed = 0.f;
			rover_velocity_setpoint.bearing = _vehicle_yaw;
			//rover_velocity_setpoint.state = (int)_current_state; // would be nice to have this field published
			_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);
		}
	}

}

void DifferentialPosControl::updateSubscriptions()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);

		if (vehicle_local_position.v_xy_valid) {
			Vector3f ground_speed3f = Vector3f{vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz};
			_ground_speed_abs = ground_speed3f.norm();

		} else {
			_ground_speed_abs = NAN;
		}
	}

	if (_rover_position_setpoint_sub.updated()) {
		_rover_position_setpoint_sub.copy(&_rover_position_setpoint);
		_start_ned = Vector2f(_rover_position_setpoint.start_ned[0], _rover_position_setpoint.start_ned[1]);
		_start_ned = _start_ned.isAllFinite() ? _start_ned : _curr_pos_ned;
		_arrival_speed = _rover_position_setpoint.arrival_speed;
		_cruising_speed = _rover_position_setpoint.cruising_speed;

		PX4_WARN("PosControl: new rover_position_setpoint:  arrival_speed=%.2f  cruising_speed=%.2f",
			 (double)_arrival_speed, (double)_cruising_speed);
	}

}

bool DifferentialPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}
