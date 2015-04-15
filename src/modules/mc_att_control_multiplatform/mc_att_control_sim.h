#ifndef MC_ATT_CONTROL_BASE_H_
#define MC_ATT_CONTROL_BASE_H_

/* Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_sim.h
 *
 * MC Attitude Controller Interface for usage in a simulator
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 *
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
#include <lib/mathlib/mathlib.h>
#inlcude "mc_att_control_base.h"



#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f

class MulticopterAttitudeControlSim :
	public MulticopterAttitudeControlBase

{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControlSim();

	/**
	 * Destructor
	 */
	~MulticopterAttitudeControlSim();

	/* setters and getters for interface with rotors-gazebo simulator */
	void set_attitude(const Eigen::Quaternion<double> attitude);
	void set_attitude_rates(const Eigen::Vector3d &angular_rate);
	void set_attitude_reference(const Eigen::Vector4d &control_attitude_thrust_reference);
	void get_mixer_input(Eigen::Vector4d &motor_inputs);

protected:
	void vehicle_attitude_setpoint_poll() {};


};

#endif /* MC_ATT_CONTROL_BASE_H_ */
