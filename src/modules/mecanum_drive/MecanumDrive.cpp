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

#include "MecanumDrive.hpp"

MecanumDrive::MecanumDrive() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool MecanumDrive::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void MecanumDrive::updateParams()
{
	ModuleParams::updateParams();

	_mecanum_drive_kinematics.setWheelBase(_param_md_wheel_base.get(), _param_md_wheel_base.get());
	_mecanum_drive_kinematics.setWheelRadius(_param_md_wheel_radius.get());

	_max_speed = _param_md_wheel_speed.get() * _param_md_wheel_radius.get();
	// _mecanum_drive_guidance.setMaxSpeed(_max_speed);
	_mecanum_drive_kinematics.setMaxSpeed(_max_speed, _max_speed);

	_max_angular_velocity = _max_speed / (_param_md_wheel_base.get() / 2.f);
	// _mecanum_drive_guidance.setMaxAngularVelocity(_max_angular_velocity);
	_mecanum_drive_kinematics.setMaxAngularVelocity(_max_angular_velocity);
}

void MecanumDrive::Run()
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
			_mecanum_drive_kinematics.setArmed(vehicle_control_mode.flag_armed);
			_manual_driving = vehicle_control_mode.flag_control_manual_enabled;
			_mission_driving = vehicle_control_mode.flag_control_auto_enabled;
		}
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_acro_driving = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO);
		}
	}

	if (_manual_driving) {
		// Manual mode
		// directly produce setpoints from the manual control setpoint (joystick)
		if (_manual_control_setpoint_sub.updated()) {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
				mecanum_drive_setpoint_s setpoint{};
				setpoint.yaw_rate = manual_control_setpoint.throttle * math::max(0.f, _param_md_speed_scale.get()) * _max_speed;
				setpoint.speed[1] = manual_control_setpoint.yaw * math::max(0.f, _param_md_speed_scale.get()) * _max_speed;
				setpoint.speed[0] = manual_control_setpoint.roll * _param_md_ang_velocity_scale.get() * _max_angular_velocity;
				// setpoint.speed[0] = 0;
				// setpoint.speed[1] = 0.0;
				// setpoint.yaw_rate = 0;

				// if acro mode, we activate the yaw rate control
				if (_acro_driving) {
					setpoint.closed_loop_speed_control = false;
					setpoint.closed_loop_yaw_rate_control = true;

				} else {
					setpoint.closed_loop_speed_control = false;
					setpoint.closed_loop_yaw_rate_control = false;
				}

				setpoint.timestamp = now;
				_mecanum_drive_setpoint_pub.publish(setpoint);
			}
		}

	}

	// else if (_mission_driving) {
	// 	Mission mode
	// 	directly receive setpoints from the guidance library
	// 	_mecanum_drive_guidance.computeGuidance(
	// 		_mecanum_drive_control.getVehicleYaw(),
	// 		_mecanum_drive_control.getVehicleBodyYawRate(),
	// 		dt
	// 	);
	// }

	_mecanum_drive_control.control(dt);
	_mecanum_drive_kinematics.allocate();
}

int MecanumDrive::task_spawn(int argc, char *argv[])
{
	MecanumDrive *instance = new MecanumDrive();

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

int MecanumDrive::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MecanumDrive::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover Mecanum Drive controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mecanum_drive", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mecanum_drive_main(int argc, char *argv[])
{
	return MecanumDrive::main(argc, argv);
}
