/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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


#include "boat_pos_control.hpp"

BoatPosControl::BoatPosControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

BoatPosControl::~BoatPosControl()
{

}

bool BoatPosControl::init()
{
	//Run on fixed interval
	ScheduleOnInterval(10_ms); // 2000 us interval, 200 Hz rate

	return true;
}

void BoatPosControl::parameters_update()
{
	if (_parameter_update_sub.updated()){
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
	pid_init(&_velocity_pid, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_set_parameters(&_velocity_pid,
			   _param_usv_speed_p.get(),  // Proportional gain
			   0,  // Integral gain
			   0,  // Derivative gain
			   2,  // Integral limit
			   200);  // Output limit
}

void BoatPosControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&_vehicle_att);
	}
}

void BoatPosControl::Run()
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	parameters_update();
	vehicle_attitude_poll();

	vehicle_control_mode_s vehicle_control_mode;

	vehicle_thrust_setpoint_s v_thrust_sp{};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.xyz[0] = 0.0f;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = 0.0f;

	vehicle_torque_setpoint_s v_torque_sp{};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.xyz[0] = 0.f;
	v_torque_sp.xyz[1] = 0.f;

	if (_vehicle_control_mode_sub.updated()) {


		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
			_position_ctrl_ena = vehicle_control_mode.flag_control_position_enabled; // change this when more modes are supported
		}
	}
	if (_armed && _position_ctrl_ena){
		if (_local_pos_sub.update(&_local_pos)) {
			_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
			const Vector3f vel = R_to_body * Vector3f(_local_pos.vx, _local_pos.vy, _local_pos.vz);

			// Speed control
			float _thrust = pid_calculate(&_velocity_pid, _manual_control_setpoint.throttle*7.f, vel(0), 0, dt);

			v_thrust_sp.xyz[0] = _thrust;
			_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);


			// yaw rate control
			v_torque_sp.xyz[2] = 0.f;
			_vehicle_torque_setpoint_pub.publish(v_torque_sp);
		}
	}
	else if (_armed && vehicle_control_mode.flag_control_manual_enabled) {
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)){

			v_thrust_sp.xyz[0] = _manual_control_setpoint.throttle;
			_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

			v_torque_sp.xyz[2] = _manual_control_setpoint.roll;
			_vehicle_torque_setpoint_pub.publish(v_torque_sp);
		}

	}
	else {

			v_thrust_sp.xyz[0] = 0.0f;
			_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);



			v_torque_sp.xyz[0] = 0.f;
			v_torque_sp.xyz[1] = 0.f;
			v_torque_sp.xyz[2] = 0.f;
			_vehicle_torque_setpoint_pub.publish(v_torque_sp);
	}


}

int BoatPosControl::task_spawn(int argc, char *argv[])
{
	BoatPosControl *instance = new BoatPosControl();

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

int BoatPosControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int BoatPosControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BoatPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
boat controller

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("boat_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int boat_pos_control_main(int argc, char *argv[])
{
	return BoatPosControl::main(argc, argv);
}
