/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "InternalCombustionEngineControl.hpp"

using namespace time_literals;

namespace internal_combustion_engine_control
{

InternalCombustionEngineControl::InternalCombustionEngineControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_internal_combustion_engine_control_pub.advertise();
	_internal_combustion_engine_status_pub.advertise();
}

InternalCombustionEngineControl::~InternalCombustionEngineControl()
{

}

int InternalCombustionEngineControl::task_spawn(int argc, char *argv[])
{
	InternalCombustionEngineControl *obj = new InternalCombustionEngineControl();

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

void InternalCombustionEngineControl::start()
{
	ScheduleOnInterval(20_ms); // 50 Hz
}

void InternalCombustionEngineControl::Run()
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
		_throttle_control_slew_rate.setSlewRate(_param_ice_thr_slew.get());
	}

	UserOnOffRequest user_request = UserOnOffRequest::Keep; // todo: keep is not yet doing anything

	manual_control_setpoint_s manual_control_setpoint;
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	vehicle_status_s vehicle_status;
	_vehicle_status_sub.copy(&vehicle_status);

	actuator_motors_s actuator_motors;
	_actuator_motors.copy(&actuator_motors);

	const float throttle_in = actuator_motors.control[0];

	switch (_param_ice_on_source.get()) {
	case 0:
		user_request = UserOnOffRequest::Off;
		break;

	case 1:
		user_request = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? UserOnOffRequest::On :
			       UserOnOffRequest::Off;
		break;

	case 2:
		user_request = UserOnOffRequest::Off;
		break;

	case 3:
		user_request = manual_control_setpoint.aux1 > 0.5f ? UserOnOffRequest::On : UserOnOffRequest::Off;
		break;

	case 4:
		user_request = manual_control_setpoint.aux2 > 0.5f ? UserOnOffRequest::On : UserOnOffRequest::Off;
		break;

	case 5:
		user_request = manual_control_setpoint.aux3 > 0.5f ? UserOnOffRequest::On : UserOnOffRequest::Off;
		break;

	case 6:
		// TODO: remove hack to have it testable with yaw stick
		user_request = manual_control_setpoint.yaw > 0.5f ? UserOnOffRequest::On : UserOnOffRequest::Off;
		break;
	}

	internal_combustion_engine_control_s ice_control{};

	switch (_state) {
	case State::Stopped:

		controlEngineStop(ice_control);

		if (user_request == UserOnOffRequest::On && !engine_tried_to_restart) {
			_state = State::Starting;
			instantiateEngineStart();
			controlEngineStartup(ice_control);
			PX4_INFO("ICE: Starting");
		}

		break;

	case State::Starting:

		if (user_request == UserOnOffRequest::Off) {
			_state = State::Stopped;
			controlEngineStop(ice_control);
			PX4_INFO("ICE: Abort");

		} else {
			if (isEngineRunning()) {
				_state = State::Running;
				controlEngineRunning(ice_control, throttle_in);
				PX4_INFO("ICE: Starting finished");

			} else {
				controlEngineStartup(ice_control);

				if (_starting_retry_cycle >= _param_ice_strt_retry.get()) {
					_state = State::Fault;
					PX4_WARN("ICE: Fault");

					engine_tried_to_restart = true;
				}
			}
		}

		break;

	case State::Running:

		controlEngineRunning(ice_control, throttle_in);

		if (user_request == UserOnOffRequest::Off) {
			_state = State::Stopped;
			PX4_INFO("ICE: Stop");

		} else {
			if (!isEngineRunning()) {
				_state = State::Fault;
				PX4_WARN("ICE: Fault detected");
			}
		}

		break;

	case State::Fault:

		// do nothing
		if (user_request == UserOnOffRequest::Off) {
			_state = State::Stopped;
			controlEngineStop(ice_control);
			PX4_INFO("ICE: Stop");

		} else {
			if (_param_ice_retry_fault.get() && !engine_tried_to_restart) {
				_state = State::Starting;
				instantiateEngineStart();
				controlEngineStartup(ice_control);
				PX4_INFO("ICE: Restarting");

			} else {
				controlEngineFault(ice_control);
			}
		}

		break;
	}

	const hrt_abstime now = hrt_absolute_time();

	const float control_interval = math::constrain(static_cast<float>((now - _last_time_run) * 1e-6f), 0.01f, 0.1f);

	_last_time_run = now;

	// slew rate limit throttle control if it's finite, otherwise just pass it through (0 throttle = NAN = disarmed)
	if (PX4_ISFINITE(ice_control.throttle_control)) {
		ice_control.throttle_control  = _throttle_control_slew_rate.update(ice_control.throttle_control, control_interval);

	} else {
		_throttle_control_slew_rate.setForcedValue(0.f);
	}

	ice_control.timestamp = now;
	ice_control.user_request = static_cast<uint8_t>(user_request);
	_internal_combustion_engine_control_pub.publish(ice_control);

	internal_combustion_engine_status_s ice_status;
	ice_status.state = static_cast<uint8_t>(_state);
	ice_status.timestamp = now;
	_internal_combustion_engine_status_pub.publish(ice_status);
}

bool InternalCombustionEngineControl::isEngineRunning()
{
	rpm_s rpm;
	_rpm_sub.copy(&rpm);

	const bool rpm_is_recent = hrt_elapsed_time(&rpm.timestamp) < 2_s;

	const bool use_rpm_feedback_for_running = _param_ice_min_run_rpm.get() > FLT_EPSILON && rpm_is_recent;
	bool engine_running = false;

	if (use_rpm_feedback_for_running) {

		if (rpm.rpm_estimate > _param_ice_min_run_rpm.get()) {
			engine_running = true;
		}

	} else {
		// without RPM feedback we assume the engine is running after the starting procedure
		engine_running = _starting_retry_cycle > 0;
	}

	return engine_running;
}

void InternalCombustionEngineControl::instantiateEngineStart()
{
	_state_start_time = hrt_absolute_time();
}

void InternalCombustionEngineControl::controlEngineRunning(internal_combustion_engine_control_s &ice_control,
		float throttle_in)
{
	ice_control.ignition_on = true;
	ice_control.choke_control = 0.f;
	ice_control.starter_engine_control = 0.f;
	ice_control.throttle_control = throttle_in;

	engine_tried_to_restart = false;
}

void InternalCombustionEngineControl::controlEngineStop(internal_combustion_engine_control_s &ice_control)
{
	ice_control.ignition_on = false;
	ice_control.choke_control = _param_ice_stop_choke.get() ? 1.f : 0.f;
	ice_control.starter_engine_control = 0.f;
	ice_control.throttle_control = 0.f;

	_starting_retry_cycle = 0;
	engine_tried_to_restart = false;
}

void InternalCombustionEngineControl::controlEngineFault(internal_combustion_engine_control_s &ice_control)
{
	ice_control.ignition_on = false;
	ice_control.choke_control = 0.f;
	ice_control.starter_engine_control = 0.f;
	ice_control.throttle_control = 0.f;
}

void InternalCombustionEngineControl::controlEngineStartup(internal_combustion_engine_control_s &ice_control)
{
	const float choke_duration = _param_ice_choke_st_dur.get() * 1_s; // todo make cold/hot start dependent
	const float starter_duration = _param_ice_starting_dur.get() * 1_s;

	if (hrt_elapsed_time(&_state_start_time) < choke_duration) {
		// choking engine to pump fuel
		ice_control.ignition_on = true;
		ice_control.choke_control = 1.f;
		ice_control.starter_engine_control = 1.f;
		ice_control.throttle_control = _param_ice_strt_thr.get();

	} else if (hrt_elapsed_time(&_state_start_time) < choke_duration + starter_duration) {
		// starting engine
		ice_control.ignition_on = true;
		ice_control.choke_control = 0.f;
		ice_control.starter_engine_control = 1.f;
		ice_control.throttle_control = _param_ice_strt_thr.get();

	} else {
		// reset timer to restart procedure if engine is not running
		_state_start_time = hrt_absolute_time();
		_starting_retry_cycle++;
		PX4_INFO("ICE: Retry %i finished", _starting_retry_cycle);
	}
}

int InternalCombustionEngineControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ICE controls.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("internal_combustion_engine_control", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int internal_combustion_engine_control_main(int argc, char *argv[])
{
	return InternalCombustionEngineControl::main(argc, argv);
}

} // namespace internal_combustion_engine_control
