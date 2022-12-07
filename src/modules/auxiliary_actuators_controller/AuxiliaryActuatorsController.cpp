/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "AuxiliaryActuatorsController.hpp"

using namespace time_literals;
using matrix::Vector3f;

namespace auxiliary_actuators_controller
{

AuxiliaryActuatorsController::AuxiliaryActuatorsController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	spoilers_setpoint_with_slewrate_.setSlewRate(kSpoilerSlewRate);
	flaps_setpoint_with_slewrate_.setSlewRate(kFlapSlewRate);

	landing_gear_pub_.advertise();

	flaps_setpoint_pub_.advertise();
	spoilers_setpoint_pub_.advertise();
}

AuxiliaryActuatorsController::~AuxiliaryActuatorsController()
{
	perf_free(_cycle_perf);
}

int AuxiliaryActuatorsController::task_spawn(int argc, char *argv[])
{
	AuxiliaryActuatorsController *obj = new AuxiliaryActuatorsController();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void AuxiliaryActuatorsController::start()
{
	ScheduleOnInterval(20_ms); // 50 Hz
}

void AuxiliaryActuatorsController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_begin(_cycle_perf);

	float dt = 0.f;

	const hrt_abstime time_now_us = hrt_absolute_time();
	dt = math::constrain((time_now_us - _last_run) * 1e-6f, kDtMin, kDtMax);
	_last_run = time_now_us;

	manual_control_setpoint_sub_.update(&manual_control_setpoint_);
	vehicle_control_mode_sub_.update(&vehicle_control_mode_);
	vehicle_status_sub_.update(&vehicle_status_);
	position_setpoint_triplet_sub_.update(&position_setpoint_triplet_);

	// landing gear logic
	landing_gear_auto_setpoint_poll();
	action_request_poll();

	// flaps and spoilers
	controlFlaps(dt);
	controlSpoilers(dt);

	landing_gear_.timestamp = hrt_absolute_time();
	landing_gear_pub_.publish(landing_gear_);

	perf_end(_cycle_perf);
}

void AuxiliaryActuatorsController::landing_gear_auto_setpoint_poll()
{
	action_request_poll();

	landing_gear_auto_setpoint_s landing_gear_auto_setpoint;

	if (landing_gear_auto_setpoint_sub_.update(&landing_gear_auto_setpoint)) {
		if (landing_gear_auto_setpoint.setpoint != lading_gear_auto_setpoint_old_) {
			landing_gear_.setpoint = landing_gear_auto_setpoint.setpoint;
			lading_gear_auto_setpoint_old_ = landing_gear_auto_setpoint.setpoint;
		}
	}
}

void AuxiliaryActuatorsController::action_request_poll()
{
	while (action_request_sub_.updated()) {
		action_request_s action_request;

		if (action_request_sub_.copy(&action_request)) {
			switch (action_request.action) {
			case action_request_s::ACTION_LANDING_GEAR_DOWN:
				landing_gear_.setpoint = landing_gear_setpoint_s::GEAR_DOWN;
				break;

			case action_request_s::ACTION_LANDING_GEAR_UP:
				landing_gear_.setpoint = landing_gear_setpoint_s::GEAR_UP;
				break;
			}
		}
	}
}

void AuxiliaryActuatorsController::controlFlaps(const float dt)
{
	flaps_auto_setpoint_s flaps_auto_setpoint;
	flaps_auto_setpoint_sub_.update(&flaps_auto_setpoint);

	// default to no flaps
	float flaps_control = 0.f;

	/* map flaps by default to manual if valid */
	if (vehicle_control_mode_.flag_control_manual_enabled && PX4_ISFINITE(manual_control_setpoint_.flaps)) {
		flaps_control = (manual_control_setpoint_.flaps + 1.f) / 2.f ; // map from [-1, 1] to [0, 1]

	} else if (vehicle_control_mode_.flag_control_auto_enabled
		   && vehicle_status_.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		// TODO: can maybe be replaced with switch (_pos_sp_triplet.current.type), problem is early land config
		switch (flaps_auto_setpoint.flaps_configuration) {
		case flaps_auto_setpoint_s::FLAPS_OFF:
			flaps_control = 0.f;
			break;

		case flaps_auto_setpoint_s::FLAPS_LAND:
			flaps_control = _param_fw_flaps_lnd_scl.get();
			break;

		case flaps_auto_setpoint_s::FLAPS_TAKEOFF:
			flaps_control = _param_fw_flaps_to_scl.get();
			break;
		}

	} else {
		flaps_control = 0.f;
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	flaps_setpoint_with_slewrate_.update(math::constrain(flaps_control, 0.f, 1.f), dt);

	flaps_setpoint_s flaps_setpoint;
	flaps_setpoint.timestamp = hrt_absolute_time();
	flaps_setpoint.normalized_setpoint = flaps_setpoint_with_slewrate_.getState();
	flaps_setpoint_pub_.publish(flaps_setpoint);
}

void AuxiliaryActuatorsController::controlSpoilers(const float dt)
{
	spoilers_auto_setpoint_s spoilers_auto_setpoint;
	spoilers_auto_setpoint_sub_.update(&spoilers_auto_setpoint);

	float spoilers_control = 0.f;

	if (vehicle_control_mode_.flag_control_manual_enabled) {
		switch (_param_fw_spoilers_man.get()) {
		case 0:
			break;

		case 1:
			// map from [-1, 1] to [0, 1]
			spoilers_control = PX4_ISFINITE(manual_control_setpoint_.flaps) ? (manual_control_setpoint_.flaps + 1.f) / 2.f : 0.f;
			break;

		case 2:
			// map from [-1, 1] to [0, 1]
			spoilers_control = PX4_ISFINITE(manual_control_setpoint_.aux1) ? (manual_control_setpoint_.aux1 + 1.f) / 2.f : 0.f;
			break;
		}

	} else if (vehicle_control_mode_.flag_control_auto_enabled
		   && vehicle_status_.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		switch (spoilers_auto_setpoint.spoilers_configuration) {
		case spoilers_auto_setpoint_s::SPOILERS_OFF:
			spoilers_control = 0.f;
			break;

		case spoilers_auto_setpoint_s::SPOILERS_LAND:
			spoilers_control = _param_fw_spoilers_lnd.get();
			break;

		case spoilers_auto_setpoint_s::SPOILERS_DESCEND:
			spoilers_control = _param_fw_spoilers_desc.get();
			break;
		}

	} else if (vehicle_control_mode_.flag_control_auto_enabled
		   && vehicle_status_.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		   && position_setpoint_triplet_.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

		spoilers_control = _param_vt_spoiler_mc_ld.get();

	} else {
		spoilers_control = 0.f;
	}

	spoilers_setpoint_with_slewrate_.update(math::constrain(spoilers_control, 0.f, 1.f), dt);

	spoilers_setpoint_s spoilers_setpoint;
	spoilers_setpoint.timestamp = hrt_absolute_time();
	spoilers_setpoint.normalized_setpoint = spoilers_setpoint_with_slewrate_.getState();
	spoilers_setpoint_pub_.publish(spoilers_setpoint);
}

int AuxiliaryActuatorsController::print_status()
{
	return 0;
}

int AuxiliaryActuatorsController::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Auxiliary Flight Control Actuators Controller
Controls landing gear, flaps, spoilers, landing gear wheel. Not responsible for payload.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("auxiliary_actuators_controller", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int auxiliary_actuators_controller_main(int argc, char *argv[])
{
	return AuxiliaryActuatorsController::main(argc, argv);
}

} // namespace auxiliary_actuators_controller
