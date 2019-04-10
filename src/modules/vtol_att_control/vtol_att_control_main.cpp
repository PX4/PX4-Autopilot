/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
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
#include "vtol_att_control_main.h"
#include <systemlib/mavlink_log.h>

namespace VTOL_att_control
{
VtolAttitudeControl *g_control;
}

/**
* Constructor
*/
VtolAttitudeControl::VtolAttitudeControl()
{
	_vtol_vehicle_status.vtol_in_rw_mode = true;	/* start vtol in rotary wing mode*/

	_params.idle_pwm_mc = PWM_DEFAULT_MIN;
	_params.vtol_motor_count = 0;

	_params_handles.idle_pwm_mc = param_find("VT_IDLE_PWM_MC");
	_params_handles.vtol_motor_count = param_find("VT_MOT_COUNT");
	_params_handles.vtol_fw_permanent_stab = param_find("VT_FW_PERM_STAB");
	_params_handles.vtol_type = param_find("VT_TYPE");
	_params_handles.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
	_params_handles.fw_min_alt = param_find("VT_FW_MIN_ALT");
	_params_handles.fw_alt_err = param_find("VT_FW_ALT_ERR");
	_params_handles.fw_qc_max_pitch = param_find("VT_FW_QC_P");
	_params_handles.fw_qc_max_roll = param_find("VT_FW_QC_R");
	_params_handles.front_trans_time_openloop = param_find("VT_F_TR_OL_TM");
	_params_handles.front_trans_time_min = param_find("VT_TRANS_MIN_TM");

	_params_handles.front_trans_duration = param_find("VT_F_TRANS_DUR");
	_params_handles.back_trans_duration = param_find("VT_B_TRANS_DUR");
	_params_handles.transition_airspeed = param_find("VT_ARSP_TRANS");
	_params_handles.front_trans_throttle = param_find("VT_F_TRANS_THR");
	_params_handles.back_trans_throttle = param_find("VT_B_TRANS_THR");
	_params_handles.airspeed_blend = param_find("VT_ARSP_BLEND");
	_params_handles.airspeed_mode = param_find("FW_ARSP_MODE");
	_params_handles.front_trans_timeout = param_find("VT_TRANS_TIMEOUT");
	_params_handles.mpc_xy_cruise = param_find("MPC_XY_CRUISE");
	_params_handles.fw_motors_off = param_find("VT_FW_MOT_OFFID");
	_params_handles.diff_thrust = param_find("VT_FW_DIFTHR_EN");
	_params_handles.diff_thrust_scale = param_find("VT_FW_DIFTHR_SC");

	_params_handles.v19_vt_rolldir = param_find("V19_VT_ROLLDIR");

	/* fetch initial parameter values */
	parameters_update();

	if (_params.vtol_type == vtol_type::TAILSITTER) {
		_vtol_type = new Tailsitter(this);

	} else if (_params.vtol_type == vtol_type::TILTROTOR) {
		_vtol_type = new Tiltrotor(this);

	} else if (_params.vtol_type == vtol_type::STANDARD) {
		_vtol_type = new Standard(this);

	} else {
		_task_should_exit = true;
	}
}

/**
* Destructor
*/
VtolAttitudeControl::~VtolAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	// free memory used by instances of base class VtolType
	if (_vtol_type != nullptr) {
		delete _vtol_type;
	}

	VTOL_att_control::g_control = nullptr;
}

/**
* Check for changes in vehicle control mode.
*/
void VtolAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

/**
* Check for changes in manual inputs.
*/
void VtolAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

/**
* Check for inputs from mc attitude controller.
*/
void VtolAttitudeControl::actuator_controls_mc_poll()
{
	bool updated;
	orb_check(_actuator_inputs_mc, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc, &_actuators_mc_in);
	}
}

/**
* Check for inputs from fw attitude controller.
*/
void VtolAttitudeControl::actuator_controls_fw_poll()
{
	bool updated;
	orb_check(_actuator_inputs_fw, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw, &_actuators_fw_in);
	}
}

/**
* Check for airspeed updates.
*/
void
VtolAttitudeControl::vehicle_airspeed_poll()
{
	bool updated;
	orb_check(_airspeed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}

/**
* Check for attitude update.
*/
void
VtolAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

/**
* Check for sensor updates.
*/
void
VtolAttitudeControl::vehicle_local_pos_poll()
{
	bool updated;
	/* Check if parameters have changed */
	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

}

/**
* Check for setpoint updates.
*/
void
VtolAttitudeControl::vehicle_local_pos_sp_poll()
{
	bool updated;
	/* Check if parameters have changed */
	orb_check(_local_pos_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_sub, &_local_pos_sp);
	}

}

/**
* Check for position setpoint updates.
*/
void
VtolAttitudeControl::pos_sp_triplet_poll()
{
	bool updated;
	/* Check if parameters have changed */
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}

}

/**
* Check for mc virtual attitude setpoint updates.
*/
void
VtolAttitudeControl::mc_virtual_att_sp_poll()
{
	bool updated;

	orb_check(_mc_virtual_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mc_virtual_attitude_setpoint), _mc_virtual_att_sp_sub, &_mc_virtual_att_sp);
	}
}

/**
* Check for fw virtual attitude setpoint updates.
*/
void
VtolAttitudeControl::fw_virtual_att_sp_poll()
{
	bool updated;

	orb_check(_fw_virtual_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(fw_virtual_attitude_setpoint), _fw_virtual_att_sp_sub, &_fw_virtual_att_sp);
	}
}

/**
* Check for command updates.
*/
void
VtolAttitudeControl::vehicle_cmd_poll()
{
	bool updated;
	orb_check(_vehicle_cmd_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_command), _vehicle_cmd_sub, &_vehicle_cmd);
		handle_command();
	}
}

/**
* Check for TECS status updates.
*/
void
VtolAttitudeControl::tecs_status_poll()
{
	bool updated;

	orb_check(_tecs_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(tecs_status), _tecs_status_sub, &_tecs_status);
	}
}

/**
* Check for land detector updates.
*/
void
VtolAttitudeControl::land_detected_poll()
{
	bool updated;

	orb_check(_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _land_detected_sub, &_land_detected);
	}
}

/**
* Check received command
*/
void
VtolAttitudeControl::handle_command()
{
	// update transition command if necessary
	if (_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
		_transition_command = int(_vehicle_cmd.param1 + 0.5f);

		// Report that we have received the command no matter what we actually do with it.
		// This might not be optimal but is better than no response at all.

		if (_vehicle_cmd.from_external) {
			vehicle_command_ack_s command_ack = {};
			command_ack.timestamp = hrt_absolute_time();
			command_ack.command = _vehicle_cmd.command;
			command_ack.result = (uint8_t)vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;
			command_ack.target_system = _vehicle_cmd.source_system;
			command_ack.target_component = _vehicle_cmd.source_component;

			if (_v_cmd_ack_pub == nullptr) {
				_v_cmd_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
								     vehicle_command_ack_s::ORB_QUEUE_LENGTH);

			} else {
				orb_publish(ORB_ID(vehicle_command_ack), _v_cmd_ack_pub, &command_ack);
			}
		}
	}
}

/*
 * Returns true if fixed-wing mode is requested.
 * Changed either via switch or via command.
 */
bool
VtolAttitudeControl::is_fixed_wing_requested()
{
	bool to_fw = false;

	if (_manual_control_sp.transition_switch != manual_control_setpoint_s::SWITCH_POS_NONE &&
	    _v_control_mode.flag_control_manual_enabled) {
		to_fw = (_manual_control_sp.transition_switch == manual_control_setpoint_s::SWITCH_POS_ON);

	} else {
		// listen to transition commands if not in manual or mode switch is not mapped
		to_fw = (_transition_command == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	}

	// handle abort request
	if (_abort_front_transition) {
		if (to_fw) {
			to_fw = false;

		} else {
			// the state changed to mc mode, reset the abort request
			_abort_front_transition = false;
			_vtol_vehicle_status.vtol_transition_failsafe = false;
		}
	}

	return to_fw;
}

/*
 * Abort front transition
 */
void
VtolAttitudeControl::abort_front_transition(const char *reason)
{
	if (!_abort_front_transition) {
		mavlink_log_critical(&_mavlink_log_pub, "Abort: %s", reason);
		_abort_front_transition = true;
		_vtol_vehicle_status.vtol_transition_failsafe = true;
	}
}

/**
* Update parameters.
*/
int
VtolAttitudeControl::parameters_update()
{
	float v;
	int32_t l;
	/* idle pwm for mc mode */
	param_get(_params_handles.idle_pwm_mc, &_params.idle_pwm_mc);

	/* vtol motor count */
	param_get(_params_handles.vtol_motor_count, &_params.vtol_motor_count);

	/* vtol fw permanent stabilization */
	param_get(_params_handles.vtol_fw_permanent_stab, &l);
	_vtol_vehicle_status.fw_permanent_stab = (l == 1);

	param_get(_params_handles.vtol_type, &l);
	_params.vtol_type = l;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles.elevons_mc_lock, &l);
	_params.elevons_mc_lock = (l == 1);

	/* minimum relative altitude for FW mode (QuadChute) */
	param_get(_params_handles.fw_min_alt, &v);
	_params.fw_min_alt = v;

	/* maximum negative altitude error for FW mode (Adaptive QuadChute) */
	param_get(_params_handles.fw_alt_err, &v);
	_params.fw_alt_err = v;

	/* maximum pitch angle (QuadChute) */
	param_get(_params_handles.fw_qc_max_pitch, &l);
	_params.fw_qc_max_pitch = l;

	/* maximum roll angle (QuadChute) */
	param_get(_params_handles.fw_qc_max_roll, &l);
	_params.fw_qc_max_roll = l;

	param_get(_params_handles.front_trans_time_openloop, &_params.front_trans_time_openloop);

	param_get(_params_handles.front_trans_time_min, &_params.front_trans_time_min);

	/*
	 * Minimum transition time can be maximum 90 percent of the open loop transition time,
	 * anything else makes no sense and can potentially lead to numerical problems.
	 */
	_params.front_trans_time_min = math::min(_params.front_trans_time_openloop * 0.9f,
				       _params.front_trans_time_min);


	param_get(_params_handles.front_trans_duration, &_params.front_trans_duration);
	param_get(_params_handles.back_trans_duration, &_params.back_trans_duration);
	param_get(_params_handles.transition_airspeed, &_params.transition_airspeed);
	param_get(_params_handles.front_trans_throttle, &_params.front_trans_throttle);
	param_get(_params_handles.back_trans_throttle, &_params.back_trans_throttle);
	param_get(_params_handles.airspeed_blend, &_params.airspeed_blend);
	param_get(_params_handles.airspeed_mode, &l);
	_params.airspeed_disabled = l != 0;
	param_get(_params_handles.front_trans_timeout, &_params.front_trans_timeout);
	param_get(_params_handles.mpc_xy_cruise, &_params.mpc_xy_cruise);
	param_get(_params_handles.fw_motors_off, &_params.fw_motors_off);
	param_get(_params_handles.diff_thrust, &_params.diff_thrust);

	param_get(_params_handles.diff_thrust_scale, &v);
	_params.diff_thrust_scale = math::constrain(v, -1.0f, 1.0f);

	// standard vtol always needs to turn all mc motors off when going into fixed wing mode
	// normally the parameter fw_motors_off can be used to specify this, however, since historically standard vtol code
	// did not use the interface of the VtolType class to disable motors we will have users flying  around with a wrong
	// parameter value. Therefore, explicitly set it here such that all motors will be disabled as expected.
	if (_params.vtol_type == vtol_type::STANDARD) {
		_params.fw_motors_off = 12345678;
	}

	// make sure parameters are feasible, require at least 1 m/s difference between transition and blend airspeed
	_params.airspeed_blend = math::min(_params.airspeed_blend, _params.transition_airspeed - 1.0f);

	// Bugfix for v1.9, should be removed in 1.10
	param_get(_params_handles.v19_vt_rolldir, &_params.v19_vt_rolldir);

	// update the parameters of the instances of base VtolType
	if (_vtol_type != nullptr) {
		_vtol_type->parameters_update();
	}

	return OK;
}

int
VtolAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	VTOL_att_control::g_control->task_main();
	return 0;
}

void VtolAttitudeControl::task_main()
{
	fflush(stdout);

	/* do subscriptions */
	_mc_virtual_att_sp_sub = orb_subscribe(ORB_ID(mc_virtual_attitude_setpoint));
	_fw_virtual_att_sp_sub = orb_subscribe(ORB_ID(fw_virtual_attitude_setpoint));
	_v_att_sub             = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub    = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub            = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_local_pos_sub         = orb_subscribe(ORB_ID(vehicle_local_position));
	_local_pos_sp_sub         = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_pos_sp_triplet_sub    = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_airspeed_sub          = orb_subscribe(ORB_ID(airspeed));
	_vehicle_cmd_sub	   = orb_subscribe(ORB_ID(vehicle_command));
	_tecs_status_sub = orb_subscribe(ORB_ID(tecs_status));
	_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	_actuator_inputs_mc    = orb_subscribe(ORB_ID(actuator_controls_virtual_mc));
	_actuator_inputs_fw    = orb_subscribe(ORB_ID(actuator_controls_virtual_fw));

	parameters_update();  // initialize parameter cache

	_task_should_exit = !_vtol_type->init();

	/* wakeup source*/
	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd     = _actuator_inputs_mc;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* only update parameters if they changed */
		bool params_updated = false;
		orb_check(_params_sub, &params_updated);

		if (params_updated) {
			/* read from param to clear updated flag */
			parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		// run vtol_att on MC actuator publications, unless in full FW mode
		switch (_vtol_type->get_mode()) {
		case TRANSITION_TO_FW:
		case TRANSITION_TO_MC:
		case ROTARY_WING:
			fds[0].fd = _actuator_inputs_mc;
			break;

		case FIXED_WING:
			fds[0].fd = _actuator_inputs_fw;
			break;
		}

		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], sizeof(fds) / sizeof(fds[0]), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			px4_usleep(100000);
			continue;
		}

		vehicle_control_mode_poll();
		vehicle_manual_poll();
		vehicle_attitude_poll();
		vehicle_local_pos_poll();
		vehicle_local_pos_sp_poll();
		pos_sp_triplet_poll();
		vehicle_airspeed_poll();
		vehicle_cmd_poll();
		tecs_status_poll();
		land_detected_poll();
		actuator_controls_fw_poll();
		actuator_controls_mc_poll();

		// update the vtol state machine which decides which mode we are in
		_vtol_type->update_vtol_state();

		// reset transition command if not auto control
		if (_v_control_mode.flag_control_manual_enabled) {
			if (_vtol_type->get_mode() == ROTARY_WING) {
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

			} else if (_vtol_type->get_mode() == FIXED_WING) {
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

			} else if (_vtol_type->get_mode() == TRANSITION_TO_MC) {
				/* We want to make sure that a mode change (manual>auto) during the back transition
				 * doesn't result in an unsafe state. This prevents the instant fall back to
				 * fixed-wing on the switch from manual to auto */
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			}
		}

		// check in which mode we are in and call mode specific functions
		if (_vtol_type->get_mode() == ROTARY_WING) {

			mc_virtual_att_sp_poll();

			// vehicle is in rotary wing mode
			_vtol_vehicle_status.vtol_in_rw_mode = true;
			_vtol_vehicle_status.vtol_in_trans_mode = false;
			_vtol_vehicle_status.in_transition_to_fw = false;

			// got data from mc attitude controller
			_vtol_type->update_mc_state();

		} else if (_vtol_type->get_mode() == FIXED_WING) {

			fw_virtual_att_sp_poll();

			// vehicle is in fw mode
			_vtol_vehicle_status.vtol_in_rw_mode = false;
			_vtol_vehicle_status.vtol_in_trans_mode = false;
			_vtol_vehicle_status.in_transition_to_fw = false;

			_vtol_type->update_fw_state();

		} else if (_vtol_type->get_mode() == TRANSITION_TO_MC || _vtol_type->get_mode() == TRANSITION_TO_FW) {

			mc_virtual_att_sp_poll();
			fw_virtual_att_sp_poll();

			// vehicle is doing a transition
			_vtol_vehicle_status.vtol_in_trans_mode = true;
			_vtol_vehicle_status.vtol_in_rw_mode = true; //making mc attitude controller work during transition
			_vtol_vehicle_status.in_transition_to_fw = (_vtol_type->get_mode() == TRANSITION_TO_FW);

			_vtol_type->update_transition_state();
		}

		// Fill actuator output
		if (_params.v19_vt_rolldir) {

			// The mixer may not have been adapted to the roll inversion in v1.9
			// Display error message and do not fill actuator outputs
			// TODO: remove the parameter and this error message in v1.10
			const int v19_rolldir_warning_throttling = 5000;
			static int v19_rolldir_warning_counter = 0;
			v19_rolldir_warning_counter += 1;

			if ((v19_rolldir_warning_counter % v19_rolldir_warning_throttling) == 0) {
				mavlink_log_critical(&_mavlink_log_pub,
						     "The VTOL roll commands were inverted in v1.9!");
				mavlink_log_critical(&_mavlink_log_pub,
						     "Check roll mixing, then set V19_VT_ROLLDIR to 0");
			}

			// Do not fill actuator output
			_actuators_out_0.timestamp = hrt_absolute_time();
			_actuators_out_0.timestamp_sample = _actuators_mc_in.timestamp_sample;
			_actuators_out_1.timestamp = hrt_absolute_time();
			_actuators_out_1.timestamp_sample = _actuators_fw_in.timestamp_sample;

			for (size_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
				_actuators_out_0.control[i] = 0.0f;
				_actuators_out_1.control[i] = 0.0f;
			}

		} else {

			// normal operation
			_vtol_type->fill_actuator_outputs();
		}

		/* Only publish if the proper mode(s) are enabled */
		if (_v_control_mode.flag_control_attitude_enabled ||
		    _v_control_mode.flag_control_rates_enabled ||
		    _v_control_mode.flag_control_manual_enabled) {

			if (_v_att_sp_pub != nullptr) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);

			} else {
				/* advertise and publish */
				_v_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
			}

			if (_actuators_0_pub != nullptr) {
				orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators_out_0);

			} else {
				_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators_out_0);
			}

			if (_actuators_1_pub != nullptr) {
				orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_out_1);

			} else {
				_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_out_1);
			}
		}

		/*Advertise/Publish vtol vehicle status*/
		_vtol_vehicle_status.timestamp = hrt_absolute_time();

		if (_vtol_vehicle_status_pub != nullptr) {
			orb_publish(ORB_ID(vtol_vehicle_status), _vtol_vehicle_status_pub, &_vtol_vehicle_status);

		} else {
			_vtol_vehicle_status_pub = orb_advertise(ORB_ID(vtol_vehicle_status), &_vtol_vehicle_status);
		}
	}

	PX4_WARN("exit");
	_control_task = -1;
}

int
VtolAttitudeControl::start()
{
	/* start the task */
	_control_task = px4_task_spawn_cmd("vtol_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL + 1,
					   1230,
					   (px4_main_t)&VtolAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}


int vtol_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: vtol_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (VTOL_att_control::g_control != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		VTOL_att_control::g_control = new VtolAttitudeControl;

		if (VTOL_att_control::g_control == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != VTOL_att_control::g_control->start()) {
			delete VTOL_att_control::g_control;
			VTOL_att_control::g_control = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (VTOL_att_control::g_control == nullptr) {
			PX4_WARN("not running");
			return 0;
		}

		delete VTOL_att_control::g_control;
		VTOL_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (VTOL_att_control::g_control) {
			PX4_WARN("running");

		} else {
			PX4_WARN("not running");
		}

		return 0;
	}

	PX4_WARN("unrecognized command");
	return 1;
}
