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

	int32_t vtol_type_param = -1;
	param_get(param_find("VT_TYPE"), &vtol_type_param);

	if (vtol_type_param == vtol_type::TAILSITTER) {
		_vtol_type = new Tailsitter(this);

	} else if (vtol_type_param == vtol_type::TILTROTOR) {
		_vtol_type = new Tiltrotor(this);

	} else if (vtol_type_param == vtol_type::STANDARD) {
		_vtol_type = new Standard(this);

	} else {
		PX4_ERR("invalid vtol_type %d", vtol_type_param);
		_task_should_exit = true;
	}

	/* fetch initial parameter values */
	parameters_update();
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
			usleep(20000);

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
			vehicle_command_ack_s command_ack = {
				.timestamp = hrt_absolute_time(),
				.result_param2 = 0,
				.command = _vehicle_cmd.command,
				.result = (uint8_t)vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED,
				.from_external = false,
				.result_param1 = 0,
				.target_system = _vehicle_cmd.source_system,
				.target_component = _vehicle_cmd.source_component
			};

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
void
VtolAttitudeControl::parameters_update()
{
	/* vtol fw permanent stabilization */
	int32_t vtol_fw_permanent_stab = 0;
	param_get(_param_vtol_fw_permanent_stab, &vtol_fw_permanent_stab);
	_vtol_vehicle_status.fw_permanent_stab = (_param_vtol_fw_permanent_stab == 1);

	// update the parameters of the instances of base VtolType
	if (_vtol_type != nullptr) {
		_vtol_type->parameters_update();
	}
}

/**
* Prepare message for mc attitude rates setpoint topic
*/
void VtolAttitudeControl::fill_mc_att_rates_sp()
{
	bool updated;
	orb_check(_mc_virtual_v_rates_sp_sub, &updated);

	if (updated) {
		vehicle_rates_setpoint_s v_rates_sp;

		if (orb_copy(ORB_ID(mc_virtual_rates_setpoint), _mc_virtual_v_rates_sp_sub, &v_rates_sp) == PX4_OK) {
			// publish the attitude rates setpoint
			if (_v_rates_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &v_rates_sp);

			} else {
				_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &v_rates_sp);
			}
		}
	}
}

/**
* Prepare message for fw attitude rates setpoint topic
*/
void VtolAttitudeControl::fill_fw_att_rates_sp()
{
	bool updated;
	orb_check(_fw_virtual_v_rates_sp_sub, &updated);

	if (updated) {
		vehicle_rates_setpoint_s v_rates_sp;

		if (orb_copy(ORB_ID(fw_virtual_rates_setpoint), _fw_virtual_v_rates_sp_sub, &v_rates_sp) == PX4_OK) {
			// publish the attitude rates setpoint
			if (_v_rates_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &v_rates_sp);

			} else {
				_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &v_rates_sp);
			}
		}
	}
}

void
VtolAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	VTOL_att_control::g_control->task_main();
}

void VtolAttitudeControl::task_main()
{
	/* do subscriptions */
	_v_att_sp_sub          = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_mc_virtual_att_sp_sub = orb_subscribe(ORB_ID(mc_virtual_attitude_setpoint));
	_fw_virtual_att_sp_sub = orb_subscribe(ORB_ID(fw_virtual_attitude_setpoint));
	_mc_virtual_v_rates_sp_sub = orb_subscribe(ORB_ID(mc_virtual_rates_setpoint));
	_fw_virtual_v_rates_sp_sub = orb_subscribe(ORB_ID(fw_virtual_rates_setpoint));
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
			usleep(100000);
			continue;
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

			// got data from mc attitude controller
			_vtol_type->update_mc_state();
			fill_mc_att_rates_sp();

		} else if (_vtol_type->get_mode() == FIXED_WING) {

			fw_virtual_att_sp_poll();

			// vehicle is in fw mode
			_vtol_vehicle_status.vtol_in_rw_mode = false;
			_vtol_vehicle_status.vtol_in_trans_mode = false;

			_vtol_type->update_fw_state();
			fill_fw_att_rates_sp();

		} else if (_vtol_type->get_mode() == TRANSITION_TO_MC || _vtol_type->get_mode() == TRANSITION_TO_FW) {

			mc_virtual_att_sp_poll();
			fw_virtual_att_sp_poll();

			// vehicle is doing a transition
			_vtol_vehicle_status.vtol_in_trans_mode = true;
			_vtol_vehicle_status.vtol_in_rw_mode = true; //making mc attitude controller work during transition
			_vtol_vehicle_status.in_transition_to_fw = (_vtol_type->get_mode() == TRANSITION_TO_FW);

			_vtol_type->update_transition_state();
			fill_mc_att_rates_sp();
		}

		_vtol_type->fill_actuator_outputs();

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
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("vtol_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL + 1,
					   1200,
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
