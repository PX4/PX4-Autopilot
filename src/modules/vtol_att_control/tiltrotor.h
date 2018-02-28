/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file tiltrotor.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
*
*/

#ifndef TILTROTOR_H
#define TILTROTOR_H

#include "vtol_type.h"
#include <drivers/drv_hrt.h>

class Tiltrotor final : public VtolType
{
public:

	Tiltrotor(VtolAttitudeControl *_att_controller);
	virtual ~Tiltrotor() = default;

	virtual void update_vtol_state();
	virtual void update_transition_state();
	virtual void fill_actuator_outputs();
	virtual void update_mc_state();
	virtual void update_fw_state();
	virtual void waiting_on_tecs();

private:

	BlockParamFloat	_param_tilt_mc;					/**< actuator value corresponding to mc tilt */
	BlockParamFloat	_param_tilt_transition;			/**< actuator value corresponding to transition tilt (e.g 45 degrees) */
	BlockParamFloat	_param_tilt_fw;					/**< actuator value corresponding to fw tilt */
	BlockParamFloat	_param_front_trans_time_openloop;
	BlockParamFloat	_param_front_trans_dur_p2;

	BlockParamInt	_param_fw_motors_off;			/**< bitmask of all motors that should be off in fixed wing mode */
	BlockParamInt	_param_diff_thrust;
	BlockParamFloat	_param_diff_thrust_scale;

	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	/**
	 * Specific to tiltrotor with vertical aligned rear engine/s.
	 * These engines need to be shut down in fw mode. During the back-transition
	 * they need to idle otherwise they need too much time to spin up for mc mode.
	 */
	enum rear_motor_state {
		ENABLED = 0,
		DISABLED,
		IDLE,
		VALUE
	} _rear_motors{ENABLED};

	struct {
		vtol_mode flight_mode;			/**< vtol flight mode, defined by enum vtol_mode */
		hrt_abstime transition_start;	/**< absoulte time at which front transition started */
	} _vtol_schedule;

	float _tilt_control{0.0f};		/**< actuator value for the tilt servo */

	uint32_t _fw_motors_off{0};

	/**
	 * Return a bitmap of channels that should be turned off in fixed wing mode.
	 */
	uint32_t get_motor_off_channels(const int channels);

	/**
	 * Return true if the motor channel is off in fixed wing mode.
	 */
	bool is_motor_off_channel(const int channel);

	/**
	 * Adjust the state of the rear motors. In fw mode they shouldn't spin.
	 */
	void set_rear_motor_state(rear_motor_state state, int value = 0);

	/**
	 * Update parameters.
	 */
	virtual void parameters_update();

};
#endif
