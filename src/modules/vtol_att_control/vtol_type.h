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

struct Params {
	int idle_pwm_mc;			// pwm value for idle in mc mode
	int vtol_motor_count;		// number of motors
	int vtol_fw_permanent_stab;	// in fw mode stabilize attitude also in manual mode
	float mc_airspeed_min;		// min airspeed in multicoper mode (including prop-wash)
	float mc_airspeed_trim;		// trim airspeed in multicopter mode
	float mc_airspeed_max;		// max airpseed in multicopter mode
	float fw_pitch_trim;		// trim for neutral elevon position in fw mode
	float power_max;			// maximum power of one engine
	float prop_eff;				// factor to calculate prop efficiency
	float arsp_lp_gain;			// total airspeed estimate low pass gain
	int vtol_type;
	int elevons_mc_lock;		// lock elevons in multicopter mode
};

enum mode {
	ROTARY_WING = 0,
	FIXED_WING,
	TRANSITION,
	EXTERNAL
};

class VtolAttitudeControl;

class VtolType
{
public:

	VtolType(VtolAttitudeControl *att_controller);

	virtual ~VtolType();

	/**
	 * Update vtol state.
	 */
	virtual void update_vtol_state() = 0;

	/**
	 * Update transition state.
	 */
	virtual void update_transition_state() = 0;

	/**
	 * Update multicopter state.
	 */
	virtual void update_mc_state();

	/**
	 * Update fixed wing state.
	 */
	virtual void update_fw_state();

	/**
	 * Update external state.
	 */
	virtual void update_external_state() {};

	/**
	 * Write control values to actuator output topics.
	 */
	virtual void fill_actuator_outputs() = 0;

	/**
	 * Special handling opportunity for the time right after transition to FW
	 * before TECS is running.
	 */
	virtual void waiting_on_tecs() {};

	void set_idle_mc();
	void set_idle_fw();

	mode get_mode() {return _vtol_mode;};

protected:
	VtolAttitudeControl *_attc;
	mode _vtol_mode;

	struct vehicle_attitude_s		*_v_att;				//vehicle attitude
	struct vehicle_attitude_setpoint_s	*_v_att_sp;			//vehicle attitude setpoint
	struct mc_virtual_attitude_setpoint_s *_mc_virtual_att_sp;	// virtual mc attitude setpoint
	struct fw_virtual_attitude_setpoint_s *_fw_virtual_att_sp;	// virtual fw attitude setpoint
	struct vehicle_rates_setpoint_s 	*_v_rates_sp;		//vehicle rates setpoint
	struct mc_virtual_rates_setpoint_s 	*_mc_virtual_v_rates_sp;		// virtual mc vehicle rates setpoint
	struct fw_virtual_rates_setpoint_s 	*_fw_virtual_v_rates_sp;		// virtual fw vehicle rates setpoint
	struct manual_control_setpoint_s	*_manual_control_sp; //manual control setpoint
	struct vehicle_control_mode_s		*_v_control_mode;	//vehicle control mode
	struct vtol_vehicle_status_s 		*_vtol_vehicle_status;
	struct actuator_controls_s			*_actuators_out_0;			//actuator controls going to the mc mixer
	struct actuator_controls_s			*_actuators_out_1;			//actuator controls going to the fw mixer (used for elevons)
	struct actuator_controls_s			*_actuators_mc_in;			//actuator controls from mc_att_control
	struct actuator_controls_s			*_actuators_fw_in;			//actuator controls from fw_att_control
	struct actuator_armed_s				*_armed;					//actuator arming status
	struct vehicle_local_position_s			*_local_pos;
	struct airspeed_s 				*_airspeed;					// airspeed
	struct battery_status_s 			*_batt_status; 				// battery status
	struct vehicle_status_s 			*_vehicle_status;			// vehicle status from commander app
	struct tecs_status_s				*_tecs_status;

	struct Params 					*_params;

	bool flag_idle_mc = true;		//false = "idle is set for fixed wing mode"; true = "idle is set for multicopter mode"

	float _mc_roll_weight = 1.0f;	// weight for multicopter attitude controller roll output
	float _mc_pitch_weight = 1.0f;	// weight for multicopter attitude controller pitch output
	float _mc_yaw_weight = 1.0f;	// weight for multicopter attitude controller yaw output
	float _mc_throttle_weight = 1.0f;	// weight for multicopter throttle command. Used to avoid
	// motors spinning up or cutting too fast whend doing transitions.
	float _thrust_transition = 0.0f;	// thrust value applied during a front transition (tailsitter & tiltrotor only)

	bool _flag_was_in_trans_mode = false;	// true if mode has just switched to transition
	hrt_abstime _trans_finished_ts = 0;
	bool _tecs_running = false;
	hrt_abstime _tecs_running_ts = 0;

};

#endif
