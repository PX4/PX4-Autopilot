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

#include "RoverMecanum.hpp"

using namespace time_literals;

RoverMecanum::RoverMecanum() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	updateParams();
}

bool RoverMecanum::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverMecanum::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ro_accel_limit.get() > FLT_EPSILON && _param_ro_max_thr_speed.get() > FLT_EPSILON) {
		_throttle_body_x_setpoint.setSlewRate(_param_ro_accel_limit.get() / _param_ro_max_thr_speed.get());
		_throttle_body_y_setpoint.setSlewRate(_param_ro_accel_limit.get() / _param_ro_max_thr_speed.get());
	}
}

void RoverMecanum::Run()
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	_mecanum_pos_control.updatePosControl();
	_mecanum_vel_control.updateVelControl();
	_mecanum_att_control.updateAttControl();
	_mecanum_rate_control.updateRateControl();

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	const bool full_manual_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
					      && !_vehicle_control_mode.flag_control_position_enabled && !_vehicle_control_mode.flag_control_attitude_enabled
					      && !_vehicle_control_mode.flag_control_rates_enabled;

	if (full_manual_mode_enabled) { // Manual mode
		generateSteeringAndThrottleSetpoint();
	}

	if (_vehicle_control_mode.flag_armed) {
		generateActuatorSetpoint();

	}

}

void RoverMecanum::generateSteeringAndThrottleSetpoint()
{
	manual_control_setpoint_s manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		rover_steering_setpoint.timestamp = _timestamp;
		rover_steering_setpoint.normalized_speed_diff = manual_control_setpoint.yaw;
		_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = _timestamp;
		rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
		rover_throttle_setpoint.throttle_body_y = manual_control_setpoint.roll;
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	}
}

void RoverMecanum::generateActuatorSetpoint()
{
	if (_rover_throttle_setpoint_sub.updated()) {
		_rover_throttle_setpoint_sub.copy(&_rover_throttle_setpoint);
	}

	if (_actuator_motors_sub.updated()) {
		actuator_motors_s actuator_motors{};
		_actuator_motors_sub.copy(&actuator_motors);
		_current_throttle_body_x = (actuator_motors.control[0] + actuator_motors.control[1]) / 2.f;
		_current_throttle_body_y = (actuator_motors.control[2] - actuator_motors.control[0]) / 2.f;
	}

	if (_rover_steering_setpoint_sub.updated()) {
		_rover_steering_setpoint_sub.copy(&_rover_steering_setpoint);
	}

	const float throttle_body_x = RoverControl::throttleControl(_throttle_body_x_setpoint,
				      _rover_throttle_setpoint.throttle_body_x, _current_throttle_body_x, _param_ro_accel_limit.get(),
				      _param_ro_decel_limit.get(), _param_ro_max_thr_speed.get(), _dt);
	const float throttle_body_y = RoverControl::throttleControl(_throttle_body_y_setpoint,
				      _rover_throttle_setpoint.throttle_body_y, _current_throttle_body_y, _param_ro_accel_limit.get(),
				      _param_ro_decel_limit.get(), _param_ro_max_thr_speed.get(), _dt);
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	computeInverseKinematics(throttle_body_x, throttle_body_y,
				 _rover_steering_setpoint.normalized_speed_diff).copyTo(actuator_motors.control);
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);


}

Vector4f RoverMecanum::computeInverseKinematics(float throttle_body_x, float throttle_body_y,
		const float speed_diff_normalized)
{
	const float total_speed = fabsf(throttle_body_x) + fabsf(throttle_body_y) + fabsf(speed_diff_normalized);

	if (total_speed > 1.f) { // Adjust speed setpoints if infeasible
		const float theta = atan2f(fabsf(throttle_body_y), fabsf(throttle_body_x));
		const float magnitude = (1.f - fabsf(speed_diff_normalized)) / (sinf(theta) + cosf(theta));
		const float normalization = 1.f / (sqrtf(powf(throttle_body_x, 2.f) + powf(throttle_body_y, 2.f)));
		throttle_body_x *= magnitude * normalization;
		throttle_body_y *= magnitude * normalization;

	}

	// Calculate motor commands
	const float input_data[3] = {throttle_body_x, throttle_body_y, speed_diff_normalized};
	const Matrix<float, 3, 1> input(input_data);
	const float m_data[12] = {1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f, 1.f, -1.f, 1.f, -1.f, 1.f};
	const Matrix<float, 4, 3> m(m_data);
	const Vector4f motor_commands = m * input;

	return motor_commands;
}

int RoverMecanum::task_spawn(int argc, char *argv[])
{
	RoverMecanum *instance = new RoverMecanum();

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

int RoverMecanum::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverMecanum::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover mecanum module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_mecanum", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_mecanum_main(int argc, char *argv[])
{
	return RoverMecanum::main(argc, argv);
}
