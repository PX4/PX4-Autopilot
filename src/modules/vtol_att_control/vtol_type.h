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
* @file vtol_type.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Sander Smeets		<sander@droneslab.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/

#ifndef VTOL_TYPE_H
#define VTOL_TYPE_H

#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <controllib/block/Block.hpp>

// Has to match 1:1 msg/vtol_vehicle_status.msg
enum mode {
	TRANSITION_TO_FW = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW,
	TRANSITION_TO_MC = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_MC,
	ROTARY_WING = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC,
	FIXED_WING = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW
};

enum vtol_type {
	TAILSITTER = 0,
	TILTROTOR,
	STANDARD
};

class VtolAttitudeControl;

using control::BlockParamFloat;
using control::BlockParamInt;

using math::constrain;
using math::max;

class VtolType : public control::SuperBlock
{
public:

	VtolType(VtolAttitudeControl *att_controller);
	VtolType(const VtolType &) = delete;
	VtolType &operator=(const VtolType &) = delete;

	virtual ~VtolType() = default;

	virtual void update_vtol_state() = 0;
	virtual void update_transition_state() = 0;
	virtual void update_mc_state();
	virtual void update_fw_state();

	/**
	 * Write control values to actuator output topics.
	 */
	virtual void fill_actuator_outputs() = 0;

	/**
	 * Special handling opportunity for the time right after transition to FW
	 * before TECS is running.
	 */
	virtual void waiting_on_tecs() {}

	/**
	 * Checks for fixed-wing failsafe condition and issues abort request if needed.
	 */
	void check_quadchute_condition();

	/**
	 * Returns true if we're allowed to do a mode transition on the ground.
	 */
	bool can_transition_on_ground();

	mode get_mode() const { return _vtol_mode; }

	virtual void parameters_update() { updateParams(); }

protected:
	VtolAttitudeControl *_attc;
	mode _vtol_mode{ROTARY_WING};

	struct vehicle_attitude_s		*_v_att;				//vehicle attitude
	struct vehicle_attitude_setpoint_s	*_v_att_sp;			//vehicle attitude setpoint
	struct vehicle_attitude_setpoint_s *_mc_virtual_att_sp;	// virtual mc attitude setpoint
	struct vehicle_attitude_setpoint_s *_fw_virtual_att_sp;	// virtual fw attitude setpoint
	struct vehicle_control_mode_s		*_v_control_mode;	//vehicle control mode
	struct actuator_controls_s			*_actuators_out_0;			//actuator controls going to the mc mixer
	struct actuator_controls_s			*_actuators_out_1;			//actuator controls going to the fw mixer (used for elevons)
	struct actuator_controls_s			*_actuators_mc_in;			//actuator controls from mc_att_control
	struct actuator_controls_s			*_actuators_fw_in;			//actuator controls from fw_att_control
	struct vehicle_local_position_s			*_local_pos;
	struct vehicle_local_position_setpoint_s	*_local_pos_sp;
	struct airspeed_s 				*_airspeed;					// airspeed
	struct tecs_status_s				*_tecs_status;
	struct vehicle_land_detected_s			*_land_detected;

	// VTOL parameters
	BlockParamFloat	_param_idle_pwm_mc; // TODO: consider removing VT_IDLE_PWM_MC
	BlockParamInt	_param_vtol_motor_count;
	BlockParamFloat	_param_fw_pitch_trim; // TODO: consider removing VT_FW_PITCH_TRIM
	BlockParamInt	_param_elevons_mc_lock;

	// VTOL front transition parameters
	BlockParamFloat	_param_front_trans_dur;
	BlockParamFloat	_param_front_trans_time_min;
	BlockParamFloat	_param_airspeed_blend_start;
	BlockParamFloat	_param_airspeed_trans;

	// VTOL back transition parameters
	BlockParamFloat	_param_back_trans_dur;

	// VTOL quadchute parameters
	BlockParamFloat	_param_qc_fw_min_alt;	// minimum relative altitude for FW mode (QuadChute)
	BlockParamFloat	_param_qc_fw_alt_err;	// altitude error for FW mode (QuadChute)
	BlockParamInt	_param_qc_fw_max_pitch;		// maximum pitch angle FW mode (QuadChute)
	BlockParamInt	_param_qc_fw_max_roll;		// maximum roll angle FW mode (QuadChute)

	// VTOL weathervane parameters
	BlockParamInt	_param_wv_takeoff;
	BlockParamInt	_param_wv_loiter;
	BlockParamInt	_param_wv_land;

	// non VTOL params
	BlockParamInt	_param_airspeed_mode;

	bool flag_idle_mc = true;		//false = "idle is set for fixed wing mode"; true = "idle is set for multicopter mode"

	float _mc_roll_weight = 1.0f;	// weight for multicopter attitude controller roll output
	float _mc_pitch_weight = 1.0f;	// weight for multicopter attitude controller pitch output
	float _mc_yaw_weight = 1.0f;	// weight for multicopter attitude controller yaw output
	float _mc_throttle_weight = 1.0f;	// weight for multicopter throttle command. Used to avoid

	// motors spinning up or cutting too fast when doing transitions.
	float _thrust_transition = 0.0f;	// thrust value applied during a front transition (tailsitter & tiltrotor only)

	float _ra_hrate = 0.0f;			// rolling average on height rate for quadchute condition
	float _ra_hrate_sp = 0.0f;		// rolling average on height rate setpoint for quadchute condition

	bool _flag_was_in_trans_mode = false;	// true if mode has just switched to transition

	hrt_abstime _trans_finished_ts = 0;

	bool _tecs_running = false;
	hrt_abstime _tecs_running_ts = 0;

	struct pwm_output_values _max_mc_pwm_values {};

	bool enable_mc_motors();
	bool disable_mc_motors();

};

#endif
