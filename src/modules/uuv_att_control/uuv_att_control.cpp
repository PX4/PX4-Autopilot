/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 *
 * This module is a modification of the fixed wing / rover module  and it is designed for unmanned underwater vehicles  (UUV).
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 * @author Philipp Hastedt <philipp.hastedt@tuhh.de>
 * @author Tim Hansen <t.hansen@tuhh.de>
 */

#include "uuv_att_control.hpp"


/**
 * UUV attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uuv_att_control_main(int argc, char *argv[]);


UUVAttitudeControl::UUVAttitudeControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

UUVAttitudeControl::~UUVAttitudeControl()
{
	perf_free(_loop_perf);
}

bool UUVAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void UUVAttitudeControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void UUVAttitudeControl::constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u,
		float thrust_x, float thrust_y, float thrust_z)
{
	if (PX4_ISFINITE(roll_u)) {
		roll_u = math::constrain(roll_u, -1.0f, 1.0f);
		_vehicle_torque_setpoint.xyz[0] = roll_u;

	} else {
		_vehicle_torque_setpoint.xyz[0] = 0.0f;
	}

	if (PX4_ISFINITE(pitch_u)) {
		pitch_u = math::constrain(pitch_u, -1.0f, 1.0f);
		_vehicle_torque_setpoint.xyz[1] = pitch_u;

	} else {
		_vehicle_torque_setpoint.xyz[1] = 0.0f;
	}

	if (PX4_ISFINITE(yaw_u)) {
		yaw_u = math::constrain(yaw_u, -1.0f, 1.0f);
		_vehicle_torque_setpoint.xyz[2] = yaw_u;

	} else {
		_vehicle_torque_setpoint.xyz[2] = 0.0f;
	}

	if (PX4_ISFINITE(thrust_x)) {
		thrust_x = math::constrain(thrust_x, -1.0f, 1.0f);
		_vehicle_thrust_setpoint.xyz[0] = thrust_x;

	} else {
		_vehicle_thrust_setpoint.xyz[0] = 0.0f;
	}

	if (PX4_ISFINITE(thrust_y)) {
		thrust_y = math::constrain(thrust_y, -1.0f, 1.0f);
		_vehicle_thrust_setpoint.xyz[1] = thrust_y;

	} else {
		_vehicle_thrust_setpoint.xyz[1] = 0.0f;
	}

	if (PX4_ISFINITE(thrust_z)) {
		thrust_z = math::constrain(thrust_z, -1.0f, 1.0f);
		_vehicle_thrust_setpoint.xyz[2] = thrust_z;

	} else {
		_vehicle_thrust_setpoint.xyz[2] = 0.0f;
	}
}

void UUVAttitudeControl::control_attitude_geo(const vehicle_attitude_s &attitude,
		const vehicle_attitude_setpoint_s &attitude_setpoint, const vehicle_angular_velocity_s &angular_velocity,
		const vehicle_rates_setpoint_s &rates_setpoint)
{
	/** Geometric Controller
	 *
	 * based on
	 * D. Mellinger, V. Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors", IEEE ICRA 2011, pp. 2520-2525.
	 * D. A. Duecker, A. Hackbarth, T. Johannink, E. Kreuzer, and E. Solowjow, “Micro Underwater Vehicle Hydrobatics: A SubmergedFuruta Pendulum,” IEEE ICRA 2018, pp. 7498–7503.
	 */
	Eulerf euler_angles(matrix::Quatf(attitude.q));

	const Eulerf setpoint_euler_angles(matrix::Quatf(attitude_setpoint.q_d));
	const float roll_body = setpoint_euler_angles(0);
	const float pitch_body = setpoint_euler_angles(1);
	const float yaw_body = setpoint_euler_angles(2);

	float roll_rate_desired = rates_setpoint.roll;
	float pitch_rate_desired = rates_setpoint.pitch;
	float yaw_rate_desired = rates_setpoint.yaw;

	/* get attitude setpoint rotational matrix */
	Dcmf rot_des = Eulerf(roll_body, pitch_body, yaw_body);

	/* get current rotation matrix from control state quaternions */
	Quatf q_att(attitude.q);
	Matrix3f rot_att =  matrix::Dcm<float>(q_att);

	Vector3f e_R_vec;
	Vector3f torques;

	/* Compute matrix: attitude error */
	Matrix3f e_R = (rot_des.transpose() * rot_att - rot_att.transpose() * rot_des) * 0.5;

	/* vee-map the error to get a vector instead of matrix e_R */
	e_R_vec(0) = e_R(2, 1);  /**< Roll  */
	e_R_vec(1) = e_R(0, 2);  /**< Pitch */
	e_R_vec(2) = e_R(1, 0);  /**< Yaw   */

	Vector3f omega{angular_velocity.xyz};
	omega(0) -= roll_rate_desired;
	omega(1) -= pitch_rate_desired;
	omega(2) -= yaw_rate_desired;

	/**< P-Control */
	torques(0) = - e_R_vec(0) * _param_roll_p.get();	/**< Roll  */
	torques(1) = - e_R_vec(1) * _param_pitch_p.get();	/**< Pitch */
	torques(2) = - e_R_vec(2) * _param_yaw_p.get();		/**< Yaw   */

	/**< PD-Control */
	torques(0) = torques(0) - omega(0) * _param_roll_d.get();  /**< Roll  */
	torques(1) = torques(1) - omega(1) * _param_pitch_d.get(); /**< Pitch */
	torques(2) = torques(2) - omega(2) * _param_yaw_d.get();   /**< Yaw   */

	float roll_u = torques(0);
	float pitch_u = torques(1);
	float yaw_u = torques(2);

	// take thrust as
	float thrust_x = attitude_setpoint.thrust_body[0];
	float thrust_y = attitude_setpoint.thrust_body[1];
	float thrust_z = attitude_setpoint.thrust_body[2];


	constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_x, thrust_y, thrust_z);
	/* Geometric Controller END*/
}

void UUVAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	/* update parameters from storage */
	parameters_update();

	vehicle_attitude_s attitude;

	/* only run controller if attitude changed */
	if (_vehicle_attitude_sub.update(&attitude)) {
		vehicle_angular_velocity_s angular_velocity {};
		_angular_velocity_sub.copy(&angular_velocity);

		/* Run geometric attitude controllers if NOT manual mode*/
		if (!_vcontrol_mode.flag_control_manual_enabled
		    && _vcontrol_mode.flag_control_attitude_enabled
		    && _vcontrol_mode.flag_control_rates_enabled) {

			int input_mode = _param_input_mode.get();

			_vehicle_attitude_setpoint_sub.update(&_attitude_setpoint);
			_vehicle_rates_setpoint_sub.update(&_rates_setpoint);

			if (input_mode == 1) { // process manual data
				Quatf attitude_setpoint(Eulerf(_param_direct_roll.get(), _param_direct_pitch.get(), _param_direct_yaw.get()));
				attitude_setpoint.copyTo(_attitude_setpoint.q_d);
				_attitude_setpoint.thrust_body[0] = _param_direct_thrust_x.get();
				_attitude_setpoint.thrust_body[1] = _param_direct_thrust_y.get();
				_attitude_setpoint.thrust_body[2] = _param_direct_thrust_z.get();
			}

			/* Geometric Control*/
			int skip_controller = _param_skip_ctrl.get();

			if (skip_controller) {
				Eulerf euler_angles(matrix::Quatf(_attitude_setpoint.q_d));
				constrain_actuator_commands(euler_angles(0), euler_angles(1), euler_angles(2),
							    _attitude_setpoint.thrust_body[0], _attitude_setpoint.thrust_body[1], _attitude_setpoint.thrust_body[2]);

			} else {
				control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint);
			}
		}
	}

	/* Manual Control mode (e.g. gamepad,...) - raw feedthrough no assistance */
	if (_manual_control_setpoint_sub.update(&_manual_control_setpoint)) {
		// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
		// returning immediately and this loop will eat up resources.
		if (_vcontrol_mode.flag_control_manual_enabled && !_vcontrol_mode.flag_control_rates_enabled) {
			/* manual/direct control */
			constrain_actuator_commands(_manual_control_setpoint.roll, -_manual_control_setpoint.pitch,
						    _manual_control_setpoint.yaw,
						    _manual_control_setpoint.throttle, 0.f, 0.f);
		}
	}

	/* Only publish if any of the proper modes are enabled */
	if (_vcontrol_mode.flag_control_manual_enabled ||
	    _vcontrol_mode.flag_control_attitude_enabled) {

		_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
		_vehicle_thrust_setpoint.timestamp_sample = 0.f;
		_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

		_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
		_vehicle_torque_setpoint.timestamp_sample = 0.f;
		_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
	}

	perf_end(_loop_perf);
}

int UUVAttitudeControl::task_spawn(int argc, char *argv[])
{
	UUVAttitudeControl *instance = new UUVAttitudeControl();

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

int UUVAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UUVAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The uuv runs missions

### Examples
CLI usage example:
$ uuv_att_control start
$ uuv_att_control status
$ uuv_att_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uuv_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uuv_att_control_main(int argc, char *argv[])
{
	return UUVAttitudeControl::main(argc, argv);
}
