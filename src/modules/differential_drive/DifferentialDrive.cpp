/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "DifferentialDrive.hpp"

DifferentialDrive::DifferentialDrive() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool DifferentialDrive::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void DifferentialDrive::updateParams()
{
	ModuleParams::updateParams();

	_differential_drive_kinematics.setWheelBase(_param_rdd_wheel_base.get());

	_max_speed = _param_rdd_wheel_speed.get() * _param_rdd_wheel_radius.get();
	_differential_drive_guidance.setMaxSpeed(_max_speed);
	_differential_drive_kinematics.setMaxSpeed(_max_speed);

	_max_angular_velocity = _max_speed / (_param_rdd_wheel_base.get() / 2.f);
	_differential_drive_guidance.setMaxAngularVelocity(_max_angular_velocity);
	_differential_drive_kinematics.setMaxAngularVelocity(_max_angular_velocity);
}

void DifferentialDrive::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	hrt_abstime now = hrt_absolute_time();
	const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e6f;
	_time_stamp_last = now;

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode{};

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_manual_driving = vehicle_control_mode.flag_control_manual_enabled;
			_mission_driving = vehicle_control_mode.flag_control_auto_enabled;
		}
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			const bool spooled_up = armed && (hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s);
			_differential_drive_kinematics.setArmed(spooled_up);
			_acro_driving = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO);
		}
	}

	if (_manual_driving) {
		// Manual mode
		// directly produce setpoints from the manual control setpoint (joystick)
		if (_manual_control_setpoint_sub.updated()) {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
				differential_drive_setpoint_s setpoint{};
				setpoint.speed = manual_control_setpoint.throttle * math::max(0.f, _param_rdd_speed_scale.get()) * _max_speed;
				setpoint.yaw_rate = manual_control_setpoint.roll * _param_rdd_ang_velocity_scale.get() * _max_angular_velocity;

				// if acro mode, we activate the yaw rate control
				if (_acro_driving) {
					setpoint.closed_loop_speed_control = false;
					setpoint.closed_loop_yaw_rate_control = true;

				} else {
					setpoint.closed_loop_speed_control = false;
					setpoint.closed_loop_yaw_rate_control = false;
				}

				setpoint.timestamp = now;
				_differential_drive_setpoint_pub.publish(setpoint);
			}
		}

	} else if (_mission_driving) {
		// Mission mode
		// directly receive setpoints from the guidance library
		_differential_drive_guidance.computeGuidance(
			_differential_drive_control.getVehicleYaw(),
			_differential_drive_control.getVehicleBodyYawRate(),
			dt
		);
	}

	_differential_drive_control.control(dt);
	_differential_drive_kinematics.allocate();
}

int DifferentialDrive::task_spawn(int argc, char *argv[])
{
	DifferentialDrive *instance = new DifferentialDrive();

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

int DifferentialDrive::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DifferentialDrive::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover Differential Drive controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("differential_drive", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int differential_drive_main(int argc, char *argv[])
{
	return DifferentialDrive::main(argc, argv);
}
