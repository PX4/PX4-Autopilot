/*
 * mc_att_control_base.h
 *
 *  Created on: Sep 25, 2014
 *      Author: roman
 */

#ifndef MC_ATT_CONTROL_BASE_H_
#define MC_ATT_CONTROL_BASE_H_

/* Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
//#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>
//#include <Eigen/Eigen>



/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f

class MulticopterAttitudeControlBase {
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControlBase();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~MulticopterAttitudeControlBase();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	void control_attitude(float dt);
	void control_attitude_rates(float dt);
	 // setters and getters for interface with euroc-gazebo simulator
	void set_attitude(const Eigen::Quaternion<double> attitude);
	void set_attitude_rates(const Eigen::Vector3d& angular_rate);
	void set_attitude_reference(const Eigen::Vector4d& control_attitude_thrust_reference);
	void get_mixer_input(Eigen::Vector4d& motor_inputs);
	void set_actuator_controls();

protected:

	bool _task_should_exit; /**< if true, sensor task should exit */
	int _control_task; /**< task handle for sensor task */




	bool _actuators_0_circuit_breaker_enabled; /**< circuit breaker to suppress output */

	struct vehicle_attitude_s _v_att; /**< vehicle attitude */
	struct vehicle_attitude_setpoint_s _v_att_sp; /**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s _v_rates_sp; /**< vehicle rates setpoint */
	struct manual_control_setpoint_s _manual_control_sp; /**< manual control setpoint */
	struct vehicle_control_mode_s _v_control_mode; /**< vehicle control mode */
	struct actuator_controls_s _actuators; /**< actuator controls */
	struct actuator_armed_s _armed; /**< actuator arming status */

	perf_counter_t _loop_perf; /**< loop performance counter */

	math::Vector<3> _rates_prev; /**< angular rates on previous step */
	math::Vector<3> _rates_sp; /**< angular rates setpoint */
	math::Vector<3> _rates_int; /**< angular rates integral error */
	float _thrust_sp; /**< thrust setpoint */
	math::Vector<3> _att_control; /**< attitude control vector */

	math::Matrix<3, 3> _I; /**< identity matrix */

	bool _reset_yaw_sp; /**< reset yaw setpoint flag */



	struct {
		math::Vector<3> att_p; /**< P gain for angular error */
		math::Vector<3> rate_p; /**< P gain for angular rate error */
		math::Vector<3> rate_i; /**< I gain for angular rate error */
		math::Vector<3> rate_d; /**< D gain for angular rate error */
		float yaw_ff; /**< yaw control feed-forward */
		float yaw_rate_max; /**< max yaw rate */

		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		math::Vector<3> acro_rate_max; /**< max attitude rates in acro mode */
	} _params;


	/**
	 * Attitude controller.
	 */
	//void control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	
	void vehicle_attitude_setpoint_poll();	//provisional

	
	



};

#endif /* MC_ATT_CONTROL_BASE_H_ */
