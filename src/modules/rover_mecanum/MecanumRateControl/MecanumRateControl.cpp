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

#include "MecanumRateControl.hpp"

using namespace time_literals;

MecanumRateControl::MecanumRateControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_status_pub.advertise();
	updateParams();
}

void MecanumRateControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
	_max_yaw_accel = _param_ro_yaw_accel_limit.get() * M_DEG_TO_RAD_F;
	_max_yaw_decel = _param_ro_yaw_decel_limit.get() * M_DEG_TO_RAD_F;
	_pid_yaw_rate.setGains(_param_ro_yaw_rate_p.get(), _param_ro_yaw_rate_i.get(), 0.f);
	_pid_yaw_rate.setIntegralLimit(1.f);
	_pid_yaw_rate.setOutputLimit(1.f);
	_adjusted_yaw_rate_setpoint.setSlewRate(_max_yaw_accel);
}

void MecanumRateControl::updateRateControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = fabsf(vehicle_angular_velocity.xyz[2]) > _param_ro_yaw_rate_th.get() * M_DEG_TO_RAD_F ?
				    vehicle_angular_velocity.xyz[2] : 0.f;
	}

	if (_vehicle_control_mode.flag_control_rates_enabled  && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		if (_vehicle_control_mode.flag_control_manual_enabled || _vehicle_control_mode.flag_control_offboard_enabled) {
			generateRateSetpoint();
		}

		generateSteeringSetpoint();

	} else { // Reset controller and slew rate when rate control is not active
		_pid_yaw_rate.resetIntegral();
		_adjusted_yaw_rate_setpoint.setForcedValue(0.f);
	}

	// Publish rate controller status (logging only)
	rover_rate_status_s rover_rate_status;
	rover_rate_status.timestamp = _timestamp;
	rover_rate_status.measured_yaw_rate = _vehicle_yaw_rate;
	rover_rate_status.adjusted_yaw_rate_setpoint = _adjusted_yaw_rate_setpoint.getState();
	rover_rate_status.pid_yaw_rate_integral = _pid_yaw_rate.getIntegral();
	_rover_rate_status_pub.publish(rover_rate_status);

}

void MecanumRateControl::generateRateSetpoint()
{
	const bool acro_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
				       && !_vehicle_control_mode.flag_control_position_enabled && !_vehicle_control_mode.flag_control_attitude_enabled;

	if (acro_mode_enabled && _manual_control_setpoint_sub.updated()) { // Acro Mode
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			rover_throttle_setpoint_s rover_throttle_setpoint{};
			rover_throttle_setpoint.timestamp = _timestamp;
			rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
			rover_throttle_setpoint.throttle_body_y = manual_control_setpoint.roll;
			_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = math::interpolate<float> (manual_control_setpoint.yaw, -1.f, 1.f,
								-_max_yaw_rate, _max_yaw_rate);
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
		}

	} else if (_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard rate control
		trajectory_setpoint_s trajectory_setpoint{};
		_trajectory_setpoint_sub.copy(&trajectory_setpoint);

		if (_offboard_control_mode_sub.updated()) {
			_offboard_control_mode_sub.copy(&_offboard_control_mode);
		}

		const bool offboard_rate_control = _offboard_control_mode.body_rate && !_offboard_control_mode.attitude;

		if (offboard_rate_control && PX4_ISFINITE(trajectory_setpoint.yawspeed)) {
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = trajectory_setpoint.yawspeed;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
		}
	}
}

void MecanumRateControl::generateSteeringSetpoint()
{
	if (_rover_rate_setpoint_sub.updated()) {
		_rover_rate_setpoint_sub.copy(&_rover_rate_setpoint);

	}

	float speed_diff_normalized{0.f};

	if (PX4_ISFINITE(_rover_rate_setpoint.yaw_rate_setpoint) && PX4_ISFINITE(_vehicle_yaw_rate)) {
		const float yaw_rate_setpoint = fabsf(_rover_rate_setpoint.yaw_rate_setpoint) > _param_ro_yaw_rate_th.get() *
						M_DEG_TO_RAD_F ?
						_rover_rate_setpoint.yaw_rate_setpoint : 0.f;
		speed_diff_normalized = RoverControl::rateControl(_adjusted_yaw_rate_setpoint, _pid_yaw_rate,
					yaw_rate_setpoint, _vehicle_yaw_rate, _param_rm_max_thr_yaw_r.get(), _max_yaw_accel,
					_max_yaw_decel, _param_rm_wheel_track.get(), _dt);
	}

	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = _timestamp;
	rover_steering_setpoint.normalized_speed_diff = speed_diff_normalized;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
}

bool MecanumRateControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float>(events::ID("mecanum_rate_control_conf_invalid_yaw_rate_lim"), events::Log::Error,
					    "Invalid configuration of necessary parameter RO_YAW_RATE_LIM", _param_ro_yaw_rate_limit.get());
		}

	}

	if ((_param_rm_wheel_track.get() < FLT_EPSILON || _param_rm_max_thr_yaw_r.get() < FLT_EPSILON)
	    && _param_ro_yaw_rate_p.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float, float, float>(events::ID("mecanum_rate_control_conf_invalid_rate_control"), events::Log::Error,
							  "Invalid configuration for rate control: Neither feed forward (RM_MAX_THR_YAW_R) nor feedback (RO_YAW_RATE_P) is setup",
							  _param_rm_wheel_track.get(),
							  _param_rm_max_thr_yaw_r.get(), _param_ro_yaw_rate_p.get());
		}
	}

	_prev_param_check_passed = ret;
	return ret;
}
