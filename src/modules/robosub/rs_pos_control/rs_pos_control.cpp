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
 * This module is a modification of the hippocampus control module and is designed for the
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

RobosubPOSControl::RobosubPOSControl()
    : ModuleParams(nullptr), WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
      /* performance counters */
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle"))
{
}

RobosubPOSControl::~RobosubPOSControl()
{
	perf_free(_loop_perf);
}

bool RobosubPOSControl::init()
{
	// Execute the Run() function everytime an input_rc is publiced
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// PX4_DEBUG("RobosubPOSControl::init()");
	return true;
}

void RobosubPOSControl::parameters_update(bool force)
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
 * @brief from UUVPOSControl
 * TODO_RS should also publish
 */
// void RobosubPOSControl::publish_attitude_setpoint(const float thrust_x, const float thrust_y, const float thrust_z,
// 	const float roll_des, const float pitch_des, const float yaw_des)
// {
// // watch if inputs are not to high
// vehicle_attitude_setpoint_s vehicle_attitude_setpoint = {};
// vehicle_attitude_setpoint.timestamp = hrt_absolute_time();

// const Quatf attitude_setpoint(Eulerf(roll_des, pitch_des, yaw_des));
// attitude_setpoint.copyTo(vehicle_attitude_setpoint.q_d);

// vehicle_attitude_setpoint.thrust_body[0] = thrust_x;
// vehicle_attitude_setpoint.thrust_body[1] = thrust_y;
// vehicle_attitude_setpoint.thrust_body[2] = thrust_z;


// _att_sp_pub.publish(vehicle_attitude_setpoint);
// }

/**
 * @brief from UUVPOSControl

 */
// void RobosubPOSControl::pose_controller_6dof(const Vector3f &pos_des,
// 	const float roll_des, const float pitch_des, const float yaw_des,
// 	vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
// {
// //get current rotation of vehicle
// Quatf q_att(vehicle_attitude.q);

// Vector3f p_control_output = Vector3f(_param_pose_gain_x.get() * (pos_des(0) - vlocal_pos.x) - _param_pose_gain_d_x.get()
// 				     * vlocal_pos.vx,
// 				     _param_pose_gain_y.get() * (pos_des(1) - vlocal_pos.y) - _param_pose_gain_d_y.get() * vlocal_pos.vy,
// 				     _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z) - _param_pose_gain_d_z.get() * vlocal_pos.vz);

// Vector3f rotated_input = q_att.rotateVectorInverse(p_control_output);//rotate the coord.sys (from global to body)

// publish_attitude_setpoint(rotated_input(0),
// 			  rotated_input(1),
// 			  rotated_input(2),
// 			  roll_des, pitch_des, yaw_des);

// }

// void RobosubPOSControl::stabilization_controller_6dof(const Vector3f &pos_des,
// 	const float roll_des, const float pitch_des, const float yaw_des,
// 	vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
// {
// //get current rotation of vehicle
// Quatf q_att(vehicle_attitude.q);

// Vector3f p_control_output = Vector3f(0,
// 				     0,
// 				     _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z));
// //potential d controller missing
// Vector3f rotated_input = q_att.rotateVectorInverse(p_control_output);//rotate the coord.sys (from global to body)

// publish_attitude_setpoint(rotated_input(0) + pos_des(0), rotated_input(1) + pos_des(1), rotated_input(2),
// 			  roll_des, pitch_des, yaw_des);

// }

/**
 * @brief constrains values and runs actuator_test.
 *
 * Borrow from UUVAttitudeControl::constrain_actuator_commands
 * @param pitch_u float
 */
void RobosubPOSControl::constrain_actuator_commands(float pitch_u)
{
	// if (PX4_ISFINITE(roll_u)) {
	// 	roll_u = math::constrain(roll_u, -1.0f, 1.0f);
	// 	_vehicle_torque_setpoint.xyz[0] = roll_u;

	// } else {
	// 	_vehicle_torque_setpoint.xyz[0] = 0.0f;
	// }

	if (PX4_ISFINITE(pitch_u)) {
		pitch_u = math::constrain(pitch_u, -1.0f, 1.0f);
		// _vehicle_torque_setpoint.xyz[1] = pitch_u;
		actuator_test(101, pitch_u, 0, false);

	} else {
		// _vehicle_torque_setpoint.xyz[1] = 0.0f;
	}

	// if (PX4_ISFINITE(yaw_u)) {
	// 	yaw_u = math::constrain(yaw_u, -1.0f, 1.0f);
	// 	_vehicle_torque_setpoint.xyz[2] = yaw_u;

	// } else {
	// 	_vehicle_torque_setpoint.xyz[2] = 0.0f;
	// }
}


void RobosubPOSControl::Run()
{
	PX4_INFO("RobosubPOSControl::Run()");

	if (should_exit()) {
		//  _vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	//  _vcontrol_mode_sub.update(&_vcontrol_mode);
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	/* update parameters from storage */
	parameters_update();

	vehicle_attitude_s attitude;

	/* only run  if attitude changed */
	if (_vehicle_attitude_sub.update(&attitude))
	{
		// get angular velocity
		// vehicle_angular_velocity_s angular_velocity {};
		// _angular_velocity_sub.copy(&angular_velocity);
		control_gyro(attitude);
	}


	/* Only publish if any of the proper modes are enabled */
	if (_vcontrol_mode.flag_control_manual_enabled ||
		_vcontrol_mode.flag_control_attitude_enabled)
		{

		}

	perf_end(_loop_perf);
}

/**
 * @brief constrains values and runs actuator_test.
 *
 * Borrow from UUVAttitudeControl::constrain_actuator_commands
 * @param pitch_u float
 */
void RobosubPOSControl::actuator_test(int function, float value, int timeout_ms, bool release_control)
{
	PX4_DEBUG("actuator_test value: %.2f", (double) value);

	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	actuator_test_pub.publish(actuator_test);
}

/**
 * @brief Reads the gyroscope data and extracts the roll of the PX4 device.
 *
 * This function processes the vehicle attitude quaternion to compute the roll,
 * pitch, and yaw angles. It specifically logs the gyroscope value for debugging purposes.
 *
 * @param attitude The vehicle attitude structure containing quaternion data.
 */
void RobosubPOSControl::control_gyro(const vehicle_attitude_s &attitude)
{
	/* get attitude setpoint rotational matrix */
	// Dcmf rot_des = Eulerf(roll_body, pitch_body, yaw_body);

	Eulerf euler_angles(matrix::Quatf(attitude.q));

        // Extract roll, pitch, and yaw
        float roll = euler_angles.phi();    // Roll angle in radians
        float pitch = euler_angles.theta(); // Pitch angle in radians
        float yaw = euler_angles.psi();     // Yaw angle in radians

        // Log the gyro data
        PX4_INFO("roll: %f", (double) roll);
        PX4_INFO("pitch: %f",(double) pitch);
        PX4_INFO("yaw: %f", (double) yaw);

	// /* get current rotation matrix from control state quaternions */
	// Quatf q_att(attitude.q);
	// Matrix3f rot_att =  matrix::Dcm<float>(q_att);

	// Vector3f e_R_vec;
	// Vector3f torques;

	// Map pitch from rads [-π/2, π/2] to [-1, 1]
	// float pitch_u = pitch / (float) M_PI_2);
	float pitch_u = pitch / (float) M_PI_4;

	constrain_actuator_commands(pitch_u);
	/* gyro controller End*/
}

int RobosubPOSControl::task_spawn(int argc, char *argv[])
{
	RobosubPOSControl *instance = new RobosubPOSControl();

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

int RobosubPOSControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int RobosubPOSControl::print_usage(const char *reason)
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
	return RobosubPOSControl::main(argc, argv);
}
