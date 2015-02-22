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

using namespace px4;

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
	px4::Subscriber<px4_vehicle_attitude> *_v_att;			    /**< vehicle attitude */
	px4::Subscriber<px4_vehicle_attitude_setpoint> *_v_att_sp;	    /**< vehicle attitude setpoint */
	px4::Subscriber<px4_vehicle_rates_setpoint> *_v_rates_sp;	    /**< vehicle rates setpoint */
	px4::Subscriber<px4_vehicle_control_mode> *_v_control_mode;	    /**< vehicle control mode */
	px4::Subscriber<px4_parameter_update> *_parameter_update;	    /**< parameter update */
	px4::Subscriber<px4_manual_control_setpoint> *_manual_control_sp;   /**< manual control setpoint */
	px4::Subscriber<px4_actuator_armed> *_armed;			    /**< actuator arming status */
	px4::Subscriber<px4_vehicle_status> *_v_status;			    /**< vehicle status */

	px4_vehicle_rates_setpoint	_v_rates_sp_mod;	/**< vehicle rates setpoint
								  that gets published eventually*/
	px4_actuator_controls_0		_actuators;	/**< actuator controls */

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

		math::Vector<3> acro_rate_max; /**< max attitude rates in acro mode */

		int32_t autostart_id;
	} _params;

	bool _publish_att_sp;

};

#endif /* MC_ATT_CONTROL_BASE_H_ */
