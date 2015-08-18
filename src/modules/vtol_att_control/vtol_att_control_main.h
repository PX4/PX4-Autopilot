/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file VTOL_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which mode the vehicle is in (hover,forward-
 * flight or transition). It also publishes the resulting controls on the actuator controls topics.
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler	<thomasgubler@gmail.com>
 *
 */
#ifndef VTOL_ATT_CONTROL_MAIN_H
#define VTOL_ATT_CONTROL_MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_virtual_mc.h>
#include <uORB/topics/actuator_controls_virtual_fw.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_command.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <nuttx/fs/ioctl.h>
#include <fcntl.h>

#include "tiltrotor.h"
#include "tailsitter.h"
#include "standard.h"


extern "C" __EXPORT int vtol_att_control_main(int argc, char *argv[]);


class VtolAttitudeControl
{
public:

	VtolAttitudeControl();
	~VtolAttitudeControl();

	int start();	/* start the task and return OK on success */
	bool is_fixed_wing_requested();

	struct vehicle_attitude_s* 			get_att () {return &_v_att;}
	struct vehicle_attitude_setpoint_s* get_att_sp () {return &_v_att_sp;}
	struct vehicle_rates_setpoint_s* 	get_rates_sp () {return &_v_rates_sp;}
	struct vehicle_rates_setpoint_s* 	get_mc_virtual_rates_sp () {return &_mc_virtual_v_rates_sp;}
	struct vehicle_rates_setpoint_s* 	get_fw_virtual_rates_sp () {return &_fw_virtual_v_rates_sp;}
	struct manual_control_setpoint_s* 	get_manual_control_sp () {return &_manual_control_sp;}
	struct vehicle_control_mode_s* 		get_control_mode () {return &_v_control_mode;}
	struct vtol_vehicle_status_s*		get_vehicle_status () {return &_vtol_vehicle_status;}
	struct actuator_controls_s* 		get_actuators_out0 () {return &_actuators_out_0;}
	struct actuator_controls_s* 		get_actuators_out1 () {return &_actuators_out_1;}
	struct actuator_controls_s* 		get_actuators_mc_in () {return &_actuators_mc_in;}
	struct actuator_controls_s* 		get_actuators_fw_in () {return &_actuators_fw_in;}
	struct actuator_armed_s* 			get_armed () {return &_armed;}
	struct vehicle_local_position_s* 	get_local_pos () {return &_local_pos;}
	struct airspeed_s* 					get_airspeed () {return &_airspeed;}
	struct battery_status_s* 			get_batt_status () {return &_batt_status;}

	struct Params* 						get_params () {return &_params;}


private:
//******************flags & handlers******************************************************
	bool _task_should_exit;
	int _control_task;		//task handle for VTOL attitude controller

	/* handlers for subscriptions */
	int		_v_att_sub;				//vehicle attitude subscription
	int		_v_att_sp_sub;			//vehicle attitude setpoint subscription
	int		_mc_virtual_v_rates_sp_sub;		//vehicle rates setpoint subscription
	int		_fw_virtual_v_rates_sp_sub;		//vehicle rates setpoint subscription
	int		_v_control_mode_sub;	//vehicle control mode subscription
	int		_params_sub;			//parameter updates subscription
	int		_manual_control_sp_sub;	//manual control setpoint subscription
	int		_armed_sub;				//arming status subscription
	int 	_local_pos_sub;			// sensor subscription
	int 	_airspeed_sub;			// airspeed subscription
	int 	_battery_status_sub;	// battery status subscription
	int 	_vehicle_cmd_sub;

	int 	_actuator_inputs_mc;	//topic on which the mc_att_controller publishes actuator inputs
	int 	_actuator_inputs_fw;	//topic on which the fw_att_controller publishes actuator inputs

	//handlers for publishers
	orb_advert_t	_actuators_0_pub;		//input for the mixer (roll,pitch,yaw,thrust)
	orb_advert_t 	_actuators_1_pub;
	orb_advert_t	_vtol_vehicle_status_pub;
	orb_advert_t	_v_rates_sp_pub;
//*******************data containers***********************************************************
	struct vehicle_attitude_s			_v_att;				//vehicle attitude
	struct vehicle_attitude_setpoint_s	_v_att_sp;			//vehicle attitude setpoint
	struct vehicle_rates_setpoint_s		_v_rates_sp;		//vehicle rates setpoint
	struct vehicle_rates_setpoint_s		_mc_virtual_v_rates_sp;		// virtual mc vehicle rates setpoint
	struct vehicle_rates_setpoint_s		_fw_virtual_v_rates_sp;		// virtual fw vehicle rates setpoint
	struct manual_control_setpoint_s	_manual_control_sp; //manual control setpoint
	struct vehicle_control_mode_s		_v_control_mode;	//vehicle control mode
	struct vtol_vehicle_status_s 		_vtol_vehicle_status;
	struct actuator_controls_s			_actuators_out_0;	//actuator controls going to the mc mixer
	struct actuator_controls_s			_actuators_out_1;	//actuator controls going to the fw mixer (used for elevons)
	struct actuator_controls_s			_actuators_mc_in;	//actuator controls from mc_att_control
	struct actuator_controls_s			_actuators_fw_in;	//actuator controls from fw_att_control
	struct actuator_armed_s				_armed;				//actuator arming status
	struct vehicle_local_position_s		_local_pos;
	struct airspeed_s 					_airspeed;			// airspeed
	struct battery_status_s 			_batt_status; 		// battery status
	struct vehicle_command_s			_vehicle_cmd;

	Params _params;	// struct holding the parameters

	struct {
		param_t idle_pwm_mc;
		param_t vtol_motor_count;
		param_t vtol_fw_permanent_stab;
		param_t mc_airspeed_min;
		param_t mc_airspeed_trim;
		param_t mc_airspeed_max;
		param_t fw_pitch_trim;
		param_t power_max;
		param_t prop_eff;
		param_t arsp_lp_gain;
		param_t vtol_type;
		param_t elevons_mc_lock;
	} _params_handles;

	/* for multicopters it is usual to have a non-zero idle speed of the engines
	 * for fixed wings we want to have an idle speed of zero since we do not want
	 * to waste energy when gliding. */
	unsigned _motor_count;	// number of motors
	float _airspeed_tot;
	int _transition_command;

	VtolType * _vtol_type;	// base class for different vtol types
	Tiltrotor * _tiltrotor;	// tailsitter vtol type
	Tailsitter * _tailsitter;	// tiltrotor vtol type
	Standard * _standard;	// standard vtol type

//*****************Member functions***********************************************************************

	void 		task_main();	//main task
	static void	task_main_trampoline(int argc, char *argv[]);	//Shim for calling task_main from task_create.

	void		vehicle_control_mode_poll();	//Check for changes in vehicle control mode.
	void		vehicle_manual_poll();			//Check for changes in manual inputs.
	void		arming_status_poll();			//Check for arming status updates.
	void 		actuator_controls_mc_poll();	//Check for changes in mc_attitude_control output
	void 		actuator_controls_fw_poll();	//Check for changes in fw_attitude_control output
	void 		vehicle_rates_sp_mc_poll();
	void 		vehicle_rates_sp_fw_poll();
	void 		vehicle_local_pos_poll();		// Check for changes in sensor values
	void 		vehicle_airspeed_poll();		// Check for changes in airspeed
	void 		vehicle_battery_poll();			// Check for battery updates
	void		vehicle_cmd_poll();
	void 		parameters_update_poll();		//Check if parameters have changed
	int 		parameters_update();			//Update local paraemter cache
	void 		fill_mc_att_rates_sp();
	void 		fill_fw_att_rates_sp();
	void		handle_command();
};

#endif
