/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file vtol_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which mode the vehicle is in (hover, forward-
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
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>

using namespace matrix;
using namespace time_literals;

VtolAttitudeControl::VtolAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "vtol_att_control: cycle"))
{
	// start vtol in rotary wing mode
	_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

	parameters_update();

	if (static_cast<vtol_type>(_param_vt_type.get()) == vtol_type::TAILSITTER) {
		_vtol_type = new Tailsitter(this);

	} else if (static_cast<vtol_type>(_param_vt_type.get()) == vtol_type::TILTROTOR) {
		_vtol_type = new Tiltrotor(this);

	} else if (static_cast<vtol_type>(_param_vt_type.get()) == vtol_type::STANDARD) {
		_vtol_type = new Standard(this);

	} else {
		exit_and_cleanup();
	}

	_flaps_setpoint_pub.advertise();
	_spoilers_setpoint_pub.advertise();
	_vtol_vehicle_status_pub.advertise();
	_vehicle_thrust_setpoint0_pub.advertise();
	_vehicle_torque_setpoint0_pub.advertise();
	_vehicle_thrust_setpoint1_pub.advertise();
	_vehicle_torque_setpoint1_pub.advertise();
}

VtolAttitudeControl::~VtolAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
VtolAttitudeControl::init()
{
	ScheduleNow();
	return true;
}

void VtolAttitudeControl::vehicle_status_poll()
{
	_vehicle_status_sub.copy(&_vehicle_status);

	// abort front transition when RTL is triggered
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
	    && _nav_state_prev != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL && _vtol_type->get_mode() == mode::TRANSITION_TO_FW) {
		_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
	}

	_nav_state_prev = _vehicle_status.nav_state;
}

void VtolAttitudeControl::action_request_poll()
{
	while (_action_request_sub.updated()) {
		action_request_s action_request;

		if (_action_request_sub.copy(&action_request)) {
			switch (action_request.action) {
			case action_request_s::ACTION_VTOL_TRANSITION_TO_MULTICOPTER:
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
				_immediate_transition = false;
				break;

			case action_request_s::ACTION_VTOL_TRANSITION_TO_FIXEDWING:
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
				_immediate_transition = false;

				// reset fixed_wing_system_failure flag when a new transition to FW is triggered
				if (_transition_command == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW) {
					_vtol_vehicle_status.fixed_wing_system_failure = false;
				}

				break;
			}
		}
	}
}

void VtolAttitudeControl::vehicle_cmd_poll()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_cmd_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {

			uint8_t result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			const int transition_command_param1 = int(vehicle_command.param1 + 0.5f);

			// deny transition from MC to FW in Takeoff, Land, RTL and Orbit
			if (transition_command_param1 == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW &&
			    (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
			     || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
			     || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
			     ||  _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT)) {

				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

			} else {
				_transition_command = transition_command_param1;
				_immediate_transition = (PX4_ISFINITE(vehicle_command.param2)) ? int(vehicle_command.param2 + 0.5f) : false;

				// reset fixed_wing_system_failure flag when a new transition to FW is triggered
				if (_transition_command == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW) {
					_vtol_vehicle_status.fixed_wing_system_failure = false;
				}
			}

			if (vehicle_command.from_external) {
				vehicle_command_ack_s command_ack{};
				command_ack.timestamp = hrt_absolute_time();
				command_ack.command = vehicle_command.command;
				command_ack.result = result;
				command_ack.target_system = vehicle_command.source_system;
				command_ack.target_component = vehicle_command.source_component;

				uORB::Publication<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};
				command_ack_pub.publish(command_ack);
			}
		}
	}
}

void
VtolAttitudeControl::quadchute(QuadchuteReason reason)
{
	if (!_vtol_vehicle_status.fixed_wing_system_failure) {
		// only publish generic warning through mavlink to safe flash
		mavlink_log_critical(&_mavlink_log_pub, "Quad-chute triggered\t");

		switch (reason) {
		case QuadchuteReason::TransitionTimeout:
			events::send(events::ID("vtol_att_ctrl_quadchute_tout"), events::Log::Critical,
				     "Quad-chute triggered due to transition timeout");
			break;

		case QuadchuteReason::ExternalCommand:
			events::send(events::ID("vtol_att_ctrl_quadchute_ext_cmd"), events::Log::Critical,
				     "Quad-chute triggered due to external command");
			break;

		case QuadchuteReason::MinimumAltBreached:
			events::send(events::ID("vtol_att_ctrl_quadchute_min_alt"), events::Log::Critical,
				     "Quad-chute triggered due to minimum altitude breach");
			break;

		case QuadchuteReason::UncommandedDescent:
			events::send(events::ID("vtol_att_ctrl_quadchute_alt_loss"), events::Log::Critical,
				     "Quad-chute triggered due to uncommanded descent detection");
			break;

		case QuadchuteReason::TransitionAltitudeLoss:
			events::send(events::ID("vtol_att_ctrl_quadchute_trans_alt_err"), events::Log::Critical,
				     "Quad-chute triggered due to loss of altitude during transition");
			break;

		case QuadchuteReason::MaximumPitchExceeded:
			events::send(events::ID("vtol_att_ctrl_quadchute_max_pitch"), events::Log::Critical,
				     "Quad-chute triggered due to maximum pitch angle exceeded");
			break;

		case QuadchuteReason::MaximumRollExceeded:
			events::send(events::ID("vtol_att_ctrl_quadchute_max_roll"), events::Log::Critical,
				     "Quad-chute triggered due to maximum roll angle exceeded");
			break;

		case QuadchuteReason::None:
			// should never get in here
			return;
		}

		_vtol_vehicle_status.fixed_wing_system_failure = true;
	}
}

void
VtolAttitudeControl::parameters_update()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		updateParams();

		if (_vtol_type != nullptr) {
			_vtol_type->parameters_update();
		}
	}
}

void
VtolAttitudeControl::update_callbacks()
{
	mode current_vtol_mode = _vtol_type->get_mode();

	switch (current_vtol_mode) {
	case mode::TRANSITION_TO_FW:
	case mode::TRANSITION_TO_MC:
	case mode::ROTARY_WING:
		if (_vehicle_torque_setpoint_virtual_mc_sub.registerCallback()) {
			_vehicle_torque_setpoint_virtual_fw_sub.unregisterCallback();
		}

		break;

	case mode::FIXED_WING:
		if (_vehicle_torque_setpoint_virtual_fw_sub.registerCallback()) {
			_vehicle_torque_setpoint_virtual_mc_sub.unregisterCallback();
		}

		break;
	}

	_previous_vtol_mode = current_vtol_mode;
}

void
VtolAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_virtual_fw_sub.unregisterCallback();
		_vehicle_torque_setpoint_virtual_mc_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

#if !defined(ENABLE_LOCKSTEP_SCHEDULER)

	const hrt_abstime now = hrt_absolute_time();

	// prevent excessive scheduling (> 500 Hz)
	if (now - _last_run_timestamp < 2_ms) {
		return;
	}

#endif // !ENABLE_LOCKSTEP_SCHEDULER

	if (!_initialized) {

		if (_vtol_type->init()) {
			update_callbacks();
			_initialized = true;

		} else {
			exit_and_cleanup();
			return;
		}
	}

	perf_begin(_loop_perf);

	bool updated_fw_in = _vehicle_torque_setpoint_virtual_fw_sub.update(&_vehicle_torque_setpoint_virtual_fw);
	updated_fw_in |= _vehicle_thrust_setpoint_virtual_fw_sub.update(&_vehicle_thrust_setpoint_virtual_fw);
	bool updated_mc_in = _vehicle_torque_setpoint_virtual_mc_sub.update(&_vehicle_torque_setpoint_virtual_mc);
	updated_mc_in |= _vehicle_thrust_setpoint_virtual_mc_sub.update(&_vehicle_thrust_setpoint_virtual_mc);

	// run on actuator publications corresponding to VTOL mode
	bool should_run = false;
	mode current_vtol_mode = _vtol_type->get_mode();

	if (current_vtol_mode != _previous_vtol_mode) {
		update_callbacks();
	}

	switch (current_vtol_mode) {
	case mode::TRANSITION_TO_FW:
	case mode::TRANSITION_TO_MC:
		should_run = updated_fw_in || updated_mc_in;
		break;

	case mode::ROTARY_WING:
		should_run = updated_mc_in;
		break;

	case mode::FIXED_WING:
		should_run = updated_fw_in;
		break;
	}

	if (should_run) {
		parameters_update();

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_attitude_sub.update(&_vehicle_attitude);
		_local_pos_sub.update(&_local_pos);
		_local_pos_sp_sub.update(&_local_pos_sp);
		_pos_sp_triplet_sub.update(&_pos_sp_triplet);
		_tecs_status_sub.update(&_tecs_status);
		_land_detected_sub.update(&_land_detected);

		if (_home_position_sub.updated()) {
			home_position_s home_position;

			if (_home_position_sub.copy(&home_position) && home_position.valid_alt) {
				_home_position_z = home_position.z;

			} else {
				_home_position_z = NAN;
			}
		}

		if (_airspeed_validated_sub.updated()) {
			airspeed_validated_s airspeed_validated;

			if (_airspeed_validated_sub.copy(&airspeed_validated)) {
				const bool airspeed_from_sensor = airspeed_validated.airspeed_source == airspeed_validated_s::SENSOR_1
								  || airspeed_validated.airspeed_source == airspeed_validated_s::SENSOR_2
								  || airspeed_validated.airspeed_source == airspeed_validated_s::SENSOR_3;
				const bool use_airspeed = _param_fw_use_airspd.get() && airspeed_from_sensor;

				_calibrated_airspeed = use_airspeed ? airspeed_validated.calibrated_airspeed_m_s : NAN;
				_time_last_airspeed_update = airspeed_validated.timestamp;

			} else if (hrt_elapsed_time(&_time_last_airspeed_update) > 1_s) {
				_calibrated_airspeed = NAN;
			}
		}

		vehicle_status_poll();
		action_request_poll();
		vehicle_cmd_poll();

		vehicle_air_data_s air_data;

		if (_vehicle_air_data_sub.update(&air_data)) {
			_air_density = air_data.rho;
		}

		_vtol_type->handleEkfResets();

		// check if mc and fw sp were updated
		const bool mc_att_sp_updated = _mc_virtual_att_sp_sub.update(&_mc_virtual_att_sp);
		const bool fw_att_sp_updated = _fw_virtual_att_sp_sub.update(&_fw_virtual_att_sp);

		// update the vtol state machine which decides which mode we are in
		_vtol_type->update_vtol_state();

		// check in which mode we are in and call mode specific functions
		switch (_vtol_type->get_mode()) {
		case mode::TRANSITION_TO_FW:
			// vehicle is doing a transition to FW
			_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW;

			if (mc_att_sp_updated || fw_att_sp_updated) {
				_vtol_type->update_transition_state();
				_vehicle_attitude_sp_pub.publish(_vehicle_attitude_sp);
			}

			break;

		case mode::TRANSITION_TO_MC:
			// vehicle is doing a transition to MC
			_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_MC;

			if (mc_att_sp_updated || fw_att_sp_updated) {
				_vtol_type->update_transition_state();
				_vehicle_attitude_sp_pub.publish(_vehicle_attitude_sp);
			}

			break;

		case mode::ROTARY_WING:
			// vehicle is in rotary wing mode
			_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

			if (mc_att_sp_updated) {
				_vtol_type->update_mc_state();
				_vehicle_attitude_sp_pub.publish(_vehicle_attitude_sp);
			}

			break;

		case mode::FIXED_WING:
			// vehicle is in fw mode
			_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

			if (fw_att_sp_updated) {
				_vtol_type->update_fw_state();
				_vehicle_attitude_sp_pub.publish(_vehicle_attitude_sp);
			}

			break;
		}

		_vtol_type->fill_actuator_outputs();

		_vehicle_torque_setpoint0_pub.publish(_torque_setpoint_0);
		_vehicle_torque_setpoint1_pub.publish(_torque_setpoint_1);
		_vehicle_thrust_setpoint0_pub.publish(_thrust_setpoint_0);
		_vehicle_thrust_setpoint1_pub.publish(_thrust_setpoint_1);

		// Advertise/publish vtol vehicle status -- immediately if changed, otherwise at 1 Hz
		const bool vtol_vehicle_status_changed =
			(_vtol_vehicle_status.vehicle_vtol_state != _prev_published_vtol_vehicle_status.vehicle_vtol_state) ||
			(_vtol_vehicle_status.fixed_wing_system_failure != _prev_published_vtol_vehicle_status.fixed_wing_system_failure);

		if (vtol_vehicle_status_changed || hrt_elapsed_time(&_prev_published_vtol_vehicle_status.timestamp) >= 1_s) {
			_vtol_vehicle_status.timestamp = hrt_absolute_time();
			_vtol_vehicle_status_pub.publish(_vtol_vehicle_status);
			_prev_published_vtol_vehicle_status = _vtol_vehicle_status;
		}

		// Publish flaps/spoiler setpoint with configured deflection in Hover if in Auto.
		// In Manual always published in FW rate controller, and in Auto FW in FW Position Controller.
		if (_vehicle_control_mode.flag_control_auto_enabled
		    && _vtol_vehicle_status.vehicle_vtol_state != vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW) {

			// flaps
			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.normalized_setpoint = 0.f; // for now always set flaps to 0 in transitions and hover
			flaps_setpoint.timestamp = hrt_absolute_time();
			_flaps_setpoint_pub.publish(flaps_setpoint);

			// spoilers
			float spoiler_control = 0.f;

			if ((_pos_sp_triplet.current.valid && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) ||
			    _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {
				spoiler_control = _param_vt_spoiler_mc_ld.get();
			}

			normalized_unsigned_setpoint_s spoiler_setpoint;
			spoiler_setpoint.normalized_setpoint = spoiler_control;
			spoiler_setpoint.timestamp = hrt_absolute_time();
			_spoilers_setpoint_pub.publish(spoiler_setpoint);
		}
	}

	perf_end(_loop_perf);
}

int
VtolAttitudeControl::task_spawn(int argc, char *argv[])
{
	VtolAttitudeControl *instance = new VtolAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
VtolAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
VtolAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_NAME("vtol_att_control", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int vtol_att_control_main(int argc, char *argv[])
{
	return VtolAttitudeControl::main(argc, argv);
}
