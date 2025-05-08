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
 * This module controls brightness of an led connected to THE pwm HEADER
 *
 * @author Daan Smienk <daansmienk10@gmail.com>
 * @author Jannick Bloemendal <jannick.bloemendal@student.hu.nl.
 */

#include "rs_pwm_led.hpp"

/**
 * Robosub pwm led app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rs_pwm_led_main(int argc, char *argv[]);

RobosubPwmLed::RobosubPwmLed()
    : ModuleParams(nullptr), WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
      /* performance counters */
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle"))
{
}

RobosubPwmLed::~RobosubPwmLed()
{
	perf_free(_loop_perf);
}

bool RobosubPwmLed::init()
{
	// Execute the Run() function everytime an input_rc is publiced
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_DEBUG("RobosubPwmLed::init()");
	return true;
}

void RobosubPwmLed::parameters_update(bool force)
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

void RobosubPwmLed::Run()
{
	PX4_INFO("RobosubPwmLed::Run()");

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

		read_gyro(attitude);
	}

	PX4_INFO("roll data");


	perf_end(_loop_perf);
}

void RobosubPwmLed::led_test(int function, float us, int timeout_ms, bool release_control)
{
	// led_control_s
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = us;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

  	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
  	actuator_test_pub.publish(actuator_test);
}

/**
 * @brief Reads the gyroscope data and extracts the roll of the PX4 device.
 *
 * This function processes the vehicle attitude quaternion to compute the roll,
 * pitch, and yaw angles. It specifically logs the roll value for debugging purposes.
 *
 * @param attitude The vehicle attitude structure containing quaternion data.
 */
void RobosubPwmLed::read_gyro(const vehicle_attitude_s &attitude)
{
	/* get attitude setpoint rotational matrix */
	// Dcmf rot_des = Eulerf(roll_body, pitch_body, yaw_body);

	Eulerf euler_angles(matrix::Quatf(attitude.q));
        // Extract roll, pitch, and yaw
        float roll = euler_angles.phi();   // Roll angle in radians
        float pitch = euler_angles.theta(); // Pitch angle in radians
        float yaw = euler_angles.psi();    // Yaw angle in radians

        // Log the roll value
        PX4_INFO("Roll: %d ", (int) roll);
        PX4_INFO("pitch: %d ", (int) pitch);
        PX4_INFO("yaw: %d ", (int) yaw);

	// /* get current rotation matrix from control state quaternions */
	// Quatf q_att(attitude.q);
	// Matrix3f rot_att =  matrix::Dcm<float>(q_att);

	// Vector3f e_R_vec;
	// Vector3f torques;
}


int RobosubPwmLed::task_spawn(int argc, char *argv[])
{
	RobosubPwmLed *instance = new RobosubPwmLed();

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

int RobosubPwmLed::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int RobosubPwmLed::print_usage(const char *reason)
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
$ rs_motor_control start
$ rs_motor_control status
$ rs_motor_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("_robosub_motor_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rs_pwm_led_main(int argc, char *argv[])
{
	return RobosubPwmLed::main(argc, argv);
}
