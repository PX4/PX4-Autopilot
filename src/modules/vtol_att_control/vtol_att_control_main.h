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
 * @author David Vorsin		<davidvorsin@gmail.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
 */
#ifndef VTOL_ATT_CONTROL_MAIN_H
#define VTOL_ATT_CONTROL_MAIN_H

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <parameters/param.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vtol_vehicle_status.h>

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
	void abort_front_transition(const char *reason);

	struct actuator_controls_s 			*get_actuators_fw_in() {return &_actuators_fw_in;}
	struct actuator_controls_s 			*get_actuators_mc_in() {return &_actuators_mc_in;}
	struct actuator_controls_s 			*get_actuators_out0() {return &_actuators_out_0;}
	struct actuator_controls_s 			*get_actuators_out1() {return &_actuators_out_1;}
	struct airspeed_s 				*get_airspeed() {return &_airspeed;}
	struct position_setpoint_triplet_s		*get_pos_sp_triplet() {return &_pos_sp_triplet;}
	struct tecs_status_s 				*get_tecs_status() {return &_tecs_status;}
	struct vehicle_attitude_s 			*get_att() {return &_v_att;}
	struct vehicle_attitude_setpoint_s		*get_att_sp() {return &_v_att_sp;}
	struct vehicle_attitude_setpoint_s 		*get_fw_virtual_att_sp() {return &_fw_virtual_att_sp;}
	struct vehicle_attitude_setpoint_s 		*get_mc_virtual_att_sp() {return &_mc_virtual_att_sp;}
	struct vehicle_control_mode_s 			*get_control_mode() {return &_v_control_mode;}
	struct vehicle_land_detected_s			*get_land_detected() {return &_land_detected;}
	struct vehicle_local_position_s 		*get_local_pos() {return &_local_pos;}
	struct vehicle_local_position_setpoint_s	*get_local_pos_sp() {return &_local_pos_sp;}
	struct vtol_vehicle_status_s			*get_vtol_vehicle_status() {return &_vtol_vehicle_status;}

	struct Params 					*get_params() {return &_params;}


private:
//******************flags & handlers******************************************************
	bool	_task_should_exit{false};
	int	_control_task{-1};		//task handle for VTOL attitude controller

	/* handlers for subscriptions */
	int	_actuator_inputs_fw{-1};	//topic on which the fw_att_controller publishes actuator inputs
	int	_actuator_inputs_mc{-1};	//topic on which the mc_att_controller publishes actuator inputs
	int	_airspeed_sub{-1};			// airspeed subscription
	int	_fw_virtual_att_sp_sub{-1};
	int	_land_detected_sub{-1};
	int	_local_pos_sp_sub{-1};			// setpoint subscription
	int	_local_pos_sub{-1};			// sensor subscription
	int	_manual_control_sp_sub{-1};	//manual control setpoint subscription
	int	_mc_virtual_att_sp_sub{-1};
	int	_params_sub{-1};			//parameter updates subscription
	int	_pos_sp_triplet_sub{-1};			// local position setpoint subscription
	int	_tecs_status_sub{-1};
	int	_v_att_sp_sub{-1};			//vehicle attitude setpoint subscription
	int	_v_att_sub{-1};				//vehicle attitude subscription
	int	_v_control_mode_sub{-1};	//vehicle control mode subscription
	int	_vehicle_cmd_sub{-1};

	//handlers for publishers
	orb_advert_t	_actuators_0_pub{nullptr};		//input for the mixer (roll,pitch,yaw,thrust)
	orb_advert_t	_mavlink_log_pub{nullptr};	// mavlink log uORB handle
	orb_advert_t	_v_att_sp_pub{nullptr};
	orb_advert_t	_v_cmd_ack_pub{nullptr};
	orb_advert_t	_vtol_vehicle_status_pub{nullptr};
	orb_advert_t 	_actuators_1_pub{nullptr};

//*******************data containers***********************************************************

	vehicle_attitude_setpoint_s		_v_att_sp{};			//vehicle attitude setpoint
	vehicle_attitude_setpoint_s 		_fw_virtual_att_sp{};	// virtual fw attitude setpoint
	vehicle_attitude_setpoint_s 		_mc_virtual_att_sp{};	// virtual mc attitude setpoint

	actuator_controls_s			_actuators_fw_in{};	//actuator controls from fw_att_control
	actuator_controls_s			_actuators_mc_in{};	//actuator controls from mc_att_control
	actuator_controls_s			_actuators_out_0{};	//actuator controls going to the mc mixer
	actuator_controls_s			_actuators_out_1{};	//actuator controls going to the fw mixer (used for elevons)

	airspeed_s 				_airspeed{};			// airspeed
	manual_control_setpoint_s		_manual_control_sp{}; //manual control setpoint
	position_setpoint_triplet_s		_pos_sp_triplet{};
	tecs_status_s				_tecs_status{};
	vehicle_attitude_s			_v_att{};				//vehicle attitude
	vehicle_command_s			_vehicle_cmd{};
	vehicle_control_mode_s			_v_control_mode{};	//vehicle control mode
	vehicle_land_detected_s			_land_detected{};
	vehicle_local_position_s			_local_pos{};
	vehicle_local_position_setpoint_s	_local_pos_sp{};
	vtol_vehicle_status_s 			_vtol_vehicle_status{};

	Params _params{};	// struct holding the parameters

	struct {
		param_t idle_pwm_mc;
		param_t vtol_motor_count;
		param_t vtol_fw_permanent_stab;
		param_t vtol_type;
		param_t elevons_mc_lock;
		param_t fw_min_alt;
		param_t fw_alt_err;
		param_t fw_qc_max_pitch;
		param_t fw_qc_max_roll;
		param_t front_trans_time_openloop;
		param_t front_trans_time_min;
		param_t front_trans_duration;
		param_t back_trans_duration;
		param_t transition_airspeed;
		param_t front_trans_throttle;
		param_t back_trans_throttle;
		param_t airspeed_blend;
		param_t airspeed_mode;
		param_t front_trans_timeout;
		param_t mpc_xy_cruise;
		param_t fw_motors_off;
		param_t diff_thrust;
		param_t diff_thrust_scale;
		param_t v19_vt_rolldir;
	} _params_handles{};

	/* for multicopters it is usual to have a non-zero idle speed of the engines
	 * for fixed wings we want to have an idle speed of zero since we do not want
	 * to waste energy when gliding. */
	int _transition_command{vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC};
	bool _abort_front_transition{false};

	VtolType *_vtol_type{nullptr};	// base class for different vtol types

//*****************Member functions***********************************************************************

	void 		task_main();	//main task
	static int	task_main_trampoline(int argc, char *argv[]);	//Shim for calling task_main from task_create.

	void		land_detected_poll();
	void		tecs_status_poll();
	void		vehicle_attitude_poll();  //Check for attitude updates.
	void		vehicle_cmd_poll();
	void		vehicle_control_mode_poll();	//Check for changes in vehicle control mode.
	void		vehicle_manual_poll();			//Check for changes in manual inputs.
	void 		actuator_controls_fw_poll();	//Check for changes in fw_attitude_control output
	void 		actuator_controls_mc_poll();	//Check for changes in mc_attitude_control output
	void 		fw_virtual_att_sp_poll();
	void 		mc_virtual_att_sp_poll();
	void 		pos_sp_triplet_poll();		// Check for changes in position setpoint values
	void 		vehicle_airspeed_poll();		// Check for changes in airspeed
	void 		vehicle_local_pos_poll();		// Check for changes in sensor values
	void 		vehicle_local_pos_sp_poll();		// Check for changes in setpoint values

	int 		parameters_update();			//Update local parameter cache

	void		handle_command();
};

#endif
