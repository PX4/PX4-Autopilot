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
 * This module is a modification of the uuv_ hippocampus control module and is designed for the
 * HU Robosub.
 *
 * @author Daan Smienk <daansmienk10@gmail.com>
 * @author Jannick Bloemendal <jannick.bloemendal@student.hu.nl.
 */

#include "rs_pos_control.hpp"



/**
 * Robosub pos_controller app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rs_pos_control_main(int argc, char *argv[]);

RobosubPosControl::RobosubPosControl()
    : ModuleParams(nullptr), WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
      /* performance counters */
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle"))
{
}

RobosubPosControl::~RobosubPosControl()
{
	perf_free(_loop_perf);
}

bool RobosubPosControl::init()
{
	// attitude
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	// vlocal pos
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

void RobosubPosControl::parameters_update(bool force)
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
/**
 * @brief Publishes attitude setpoint
 */
void RobosubPosControl::publish_attitude_setpoint(const float thrust_x, const float thrust_y, const float thrust_z,
	const float roll_des, const float pitch_des, const float yaw_des)
{
	// watch if inputs are not to high
	vehicle_attitude_setpoint_s vehicle_attitude_setpoint = {};
	vehicle_attitude_setpoint.timestamp = hrt_absolute_time();

	const Quatf attitude_setpoint(Eulerf(roll_des, pitch_des, yaw_des));
	attitude_setpoint.copyTo(vehicle_attitude_setpoint.q_d);

	vehicle_attitude_setpoint.thrust_body[0] = thrust_x;
	vehicle_attitude_setpoint.thrust_body[1] = thrust_y;
	vehicle_attitude_setpoint.thrust_body[2] = thrust_z;


	_att_sp_pub.publish(vehicle_attitude_setpoint);
}

/* TODO_RS 6DOF controller*/
void RobosubPosControl::pos_controller_6dof(const Vector3f &pos_des,
	const float roll_des, const float pitch_des, const float yaw_des,
	vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
{
	//get current rotation of vehicle
	Quatf q_att(vehicle_attitude.q);

	Vector3f p_control_output = Vector3f(_param_pose_gain_x.get() * (pos_des(0) - vlocal_pos.x) - _param_pose_gain_d_x.get()
					     * vlocal_pos.vx,
					     _param_pose_gain_y.get() * (pos_des(1) - vlocal_pos.y) - _param_pose_gain_d_y.get() * vlocal_pos.vy,
					     _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z) - _param_pose_gain_d_z.get() * vlocal_pos.vz);

	Vector3f rotated_input = q_att.rotateVectorInverse(p_control_output);//rotate the coord.sys (from global to body)

	publish_attitude_setpoint(rotated_input(0),
				  rotated_input(1),
				  rotated_input(2),
				  roll_des, pitch_des, yaw_des);

}

void RobosubPosControl::stabilization_controller_6dof(const Vector3f &pos_des,
	const float roll_des, const float pitch_des, const float yaw_des,
	vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
{
	//get current rotation of vehicle
	Quatf q_att(vehicle_attitude.q);

	Vector3f p_control_output = Vector3f(0,
					     0,
					     _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z));
	//potential d controller missing
	Vector3f rotated_input = q_att.rotateVectorInverse(p_control_output);//rotate the coord.sys (from global to body)

	publish_attitude_setpoint(rotated_input(0) + pos_des(0), rotated_input(1) + pos_des(1), rotated_input(2),
				  roll_des, pitch_des, yaw_des);
}

/**
 * @brief constrains values and setpoint
 *
 * Borrow from UUVAttitudeControl::constrain_actuator_commands
 * @param pitch_u float
 */
void RobosubPosControl::constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u,
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
}

void RobosubPosControl::control_attitude_geo(const vehicle_attitude_s &attitude,
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

void RobosubPosControl::Run()
{
	PX4_INFO("RobosubPosControl::Run()");

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
	vehicle_local_position_s vlocal_pos;

	// TODO_RS IS THIS CORRECT OR ONLY ON OF THESE? only run controller if changed in vlocal_pos OR changed in _vehicle_attitude?
	/* only run position controller if attitude changed */
	/* only run pos controller if local_pos changed */
	if (_vehicle_local_position_sub.update(&vlocal_pos)){

		/* Run geometric attitude controllers if NOT manual mode*/
		if (!_vcontrol_mode.flag_control_manual_enabled
		    && _vcontrol_mode.flag_control_attitude_enabled
		    && _vcontrol_mode.flag_control_rates_enabled){

			// setpoints
			_vehicle_attitude_sub.update(&_vehicle_attitude); // get current vehicle attitude
			_trajectory_setpoint_sub.update(&_trajectory_setpoint);

			float roll_des = 0;
			float pitch_des = 0;
			float yaw_des = _trajectory_setpoint.yaw;

			/* Stabilization Controller keep pos and hold depth + angle) vs position controller(global + yaw) */
			int enable_stabilization = _param_stabilization.get();
			if (enable_stabilization == 0) {
				pos_controller_6dof(Vector3f(_trajectory_setpoint.position),
						     roll_des, pitch_des, yaw_des, _vehicle_attitude, vlocal_pos);

			} else {
				stabilization_controller_6dof(Vector3f(_trajectory_setpoint.position),
							      roll_des, pitch_des, yaw_des, _vehicle_attitude, vlocal_pos);
			}
		}
	}

	/* Attitude controller */
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
				_attitude_setpoint.thrust_body[0] = _param_direct_thrust.get();
				_attitude_setpoint.thrust_body[1] = 0.f;
				_attitude_setpoint.thrust_body[2] = 0.f;
			}

			/* Geometric Control */
			int skip_controller = _param_skip_ctrl.get();

			if (skip_controller == 0) { // Control using geo controller
				control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint);

			} else { // Skip geometric controller
				constrain_actuator_commands(_rates_setpoint.roll, _rates_setpoint.pitch, _rates_setpoint.yaw,
							    _rates_setpoint.thrust_body[0], _rates_setpoint.thrust_body[1], _rates_setpoint.thrust_body[2]);
			}

		}
	}

	/* Manual Control mode - raw feedthrough no assistance */
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

int RobosubPosControl::task_spawn(int argc, char *argv[])
{
	RobosubPosControl *instance = new RobosubPosControl();

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

int RobosubPosControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int RobosubPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the posistion and attitude of a Hu Robosub unmanned underwater vehicle (UUV).

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

* Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
* Auto mission: The uuv runs missions

### Examples
CLI usage example:
$ rs_pos_control start
$ rs_pos_control status
$ rs_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("_robosub_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rs_pos_control_main(int argc, char *argv[])
{
	return RobosubPosControl::main(argc, argv);
}
