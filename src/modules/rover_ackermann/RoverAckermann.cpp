/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RoverAckermann.hpp"

using namespace time_literals;
using namespace matrix;

RoverAckermann::RoverAckermann() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_rover_ackermann_status_pub.advertise();
	updateParams();
}

bool RoverAckermann::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverAckermann::updateParams()
{
	ModuleParams::updateParams();

	// Update slew rates
	if (_param_ra_max_accel.get() > FLT_EPSILON && _param_ra_max_speed.get() > FLT_EPSILON) {
		_throttle_with_accel_limit.setSlewRate(_param_ra_max_accel.get() / _param_ra_max_speed.get());
	}

	if (_param_ra_max_steering_rate.get() > FLT_EPSILON && _param_ra_max_steer_angle.get() > FLT_EPSILON) {
		_steering_with_rate_limit.setSlewRate((M_DEG_TO_RAD_F * _param_ra_max_steering_rate.get()) /
						      _param_ra_max_steer_angle.get());
	}
}

void RoverAckermann::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateSubscriptions();

	// Timestamps
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// Generate motor setpoints
	if (_armed) {
		switch (_nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL: {
				manual_control_setpoint_s manual_control_setpoint{};

				if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
					_motor_setpoint.steering = manual_control_setpoint.roll;
					_motor_setpoint.throttle =  manual_control_setpoint.throttle;
				}

			} break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			_motor_setpoint = _ackermann_guidance.computeGuidance(_nav_state);
			break;

		default: // Unimplemented nav states will stop the rover
			_motor_setpoint.steering = 0.f;
			_motor_setpoint.throttle =  0.f;
			_throttle_with_accel_limit.setForcedValue(0.f);
			_steering_with_rate_limit.setForcedValue(0.f);
			break;
		}

	} else { // Reset on disarm
		_motor_setpoint.steering = 0.f;
		_motor_setpoint.throttle =  0.f;
		_throttle_with_accel_limit.setForcedValue(0.f);
		_steering_with_rate_limit.setForcedValue(0.f);
	}

	publishMotorSetpoints(applySlewRates(_motor_setpoint, dt));
}

void RoverAckermann::updateSubscriptions()
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;
		_vehicle_status_sub.copy(&vehicle_status);
		_nav_state = vehicle_status.nav_state;
		_armed = vehicle_status.arming_state == 2;
	}

	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);
		const Vector3f rover_velocity = {local_position.vx, local_position.vy, local_position.vz};
		_actual_speed = rover_velocity.norm();
	}
}
motor_setpoint_struct RoverAckermann::applySlewRates(motor_setpoint_struct motor_setpoint, const float dt)
{
	// Sanitize actuator commands
	if (!PX4_ISFINITE(motor_setpoint.steering)) {
		motor_setpoint.steering = 0.f;
	}

	if (!PX4_ISFINITE(motor_setpoint.throttle)) {
		motor_setpoint.throttle = 0.f;
	}

	// Acceleration slew rate
	if (_param_ra_max_accel.get() > FLT_EPSILON && _param_ra_max_speed.get() > FLT_EPSILON
	    && fabsf(motor_setpoint.throttle) > fabsf(_throttle_with_accel_limit.getState())) {
		_throttle_with_accel_limit.update(motor_setpoint.throttle, dt);

	} else {
		_throttle_with_accel_limit.setForcedValue(motor_setpoint.throttle);
	}

	// Steering slew rate
	if (_param_ra_max_steering_rate.get() > FLT_EPSILON && _param_ra_max_steer_angle.get() > FLT_EPSILON) {
		_steering_with_rate_limit.update(motor_setpoint.steering, dt);

	} else {
		_steering_with_rate_limit.setForcedValue(motor_setpoint.steering);
	}

	motor_setpoint_struct motor_setpoint_temp{};
	motor_setpoint_temp.steering = math::constrain(_steering_with_rate_limit.getState(), -1.f, 1.f);
	motor_setpoint_temp.throttle = math::constrain(_throttle_with_accel_limit.getState(), -1.f, 1.f);
	return motor_setpoint_temp;
}

void RoverAckermann::publishMotorSetpoints(motor_setpoint_struct motor_setpoint_with_slew_rates)
{
	// Publish rover Ackermann status (logging)
	rover_ackermann_status_s rover_ackermann_status{};
	rover_ackermann_status.timestamp = _timestamp;
	rover_ackermann_status.throttle_setpoint = _motor_setpoint.throttle;
	rover_ackermann_status.steering_setpoint = _motor_setpoint.steering;
	rover_ackermann_status.actual_speed = _actual_speed;
	_rover_ackermann_status_pub.publish(rover_ackermann_status);

	// Publish to motor
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	actuator_motors.control[0] = motor_setpoint_with_slew_rates.throttle;
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

	// Publish to servo
	actuator_servos_s actuator_servos{};
	actuator_servos.control[0] = motor_setpoint_with_slew_rates.steering;
	actuator_servos.timestamp = _timestamp;
	_actuator_servos_pub.publish(actuator_servos);
}

int RoverAckermann::task_spawn(int argc, char *argv[])
{
	RoverAckermann *instance = new RoverAckermann();

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

int RoverAckermann::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverAckermann::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover ackermann module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_ackermann", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_ackermann_main(int argc, char *argv[])
{
	return RoverAckermann::main(argc, argv);
}
