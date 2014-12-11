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
 * @file mc_att_control_base.h
 *
 * MC Attitude Controller : Control and math code
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Roman Bapst <bapstr@ethz.ch>
 *
 */
#include <px4.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include <systemlib/perf_counter.h>
#include <lib/mathlib/mathlib.h>



#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f

class MulticopterAttitudeControlBase
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControlBase();

	/**
	 * Destructor
	 */
	~MulticopterAttitudeControlBase();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	void control_attitude(float dt);
	void control_attitude_rates(float dt);

	void set_actuator_controls();

protected:
	struct vehicle_attitude_s _v_att; /**< vehicle attitude */
	struct vehicle_attitude_setpoint_s _v_att_sp; /**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s _v_rates_sp; /**< vehicle rates setpoint */
	struct manual_control_setpoint_s _manual_control_sp; /**< manual control setpoint */
	struct vehicle_control_mode_s _v_control_mode; /**< vehicle control mode */
	struct actuator_controls_s _actuators; /**< actuator controls */
	struct actuator_armed_s _armed; /**< actuator arming status */

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

	bool _publish_att_sp;

	virtual void vehicle_attitude_setpoint_poll() = 0;

};

#endif /* MC_ATT_CONTROL_BASE_H_ */
