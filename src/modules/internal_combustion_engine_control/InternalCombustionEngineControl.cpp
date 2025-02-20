/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/events.h>

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


	manual_control_setpoint_s manual_control_setpoint;
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	vehicle_status_s vehicle_status;
	_vehicle_status_sub.copy(&vehicle_status);

	actuator_motors_s actuator_motors;
	_actuator_motors.copy(&actuator_motors);

	const float throttle_in = actuator_motors.control[0];

	const hrt_abstime now = hrt_absolute_time();

	UserOnOffRequest user_request = UserOnOffRequest::Off;

	switch (static_cast<ICESource>(_param_ice_on_source.get())) {
	case ICESource::ArmingState: {
			if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				user_request = UserOnOffRequest::On;
			}
		}
		break;

	case ICESource::Aux1: {
			if (manual_control_setpoint.aux1 > 0.5f) {
				user_request = UserOnOffRequest::On;
			}
		}
		break;

	case ICESource::Aux2: {
			if (manual_control_setpoint.aux2 > 0.5f) {
				user_request = UserOnOffRequest::On;
			}
		}
		break;
	}

	switch (_state) {
	case State::Stopped: {
			controlEngineStop();

			if (user_request == UserOnOffRequest::On && !maximumAttemptsReached()) {

				_state = State::Starting;
				_state_start_time = now;
				PX4_INFO("ICE: Starting");
			}
		}
		break;

	case State::Starting: {

			if (user_request == UserOnOffRequest::Off) {
				_state = State::Stopped;
				_starting_retry_cycle = 0;
				PX4_INFO("ICE: Stopped");

			} else {

				switch (_starting_sub_state) {
				case SubState::Rest: {
						if (isStartingPermitted(now)) {
							_state_start_time = now;
							_starting_sub_state = SubState::Run;
						}
					}
					break;

				case SubState::Run:
				default: {
						controlEngineStartup(now);

						if (isEngineRunning(now)) {
							_state = State::Running;
							PX4_INFO("ICE: Starting finished");

						} else {

							if (maximumAttemptsReached()) {
								_state = State::Fault;
								PX4_WARN("ICE: Fault");

							} else if (!isStartingPermitted(now)) {
								controlEngineStop();
								_starting_sub_state = SubState::Rest;
							}
						}

						break;
					}

				}
			}

		}
		break;

	case State::Running: {
			controlEngineRunning(throttle_in);

			if (user_request == UserOnOffRequest::Off) {
				_state = State::Stopped;
				_starting_retry_cycle = 0;
				PX4_INFO("ICE: Stopped");

			} else if (!isEngineRunning(now) && _param_ice_running_fault_detection.get()) {
				// without RPM feedback we assume the engine is running after the
				// starting procedure but only switch state if fault detection is enabled
				_state = State::Starting;
				_state_start_time = now;
				_starting_retry_cycle = 0;
				PX4_WARN("ICE: Running Fault detected");
				events::send(events::ID("internal_combustion_engine_control_fault"), events::Log::Critical,
					     "IC engine fault detected");
			}
		}

		break;

	case State::Fault: {

			// do nothing
			if (user_request == UserOnOffRequest::Off) {
				_state = State::Stopped;
				_starting_retry_cycle = 0;
				PX4_INFO("ICE: Stopped");

			} else {
				controlEngineFault();
			}
		}


		break;
	}

	const float control_interval = math::constrain(static_cast<float>((now - _last_time_run) * 1e-6f), 0.01f, 0.1f);

	_last_time_run = now;

	// slew rate limit throttle control if it's finite, otherwise just pass it through (0 throttle = NAN = disarmed)
	if (PX4_ISFINITE(_throttle_control)) {
		_throttle_control  = _throttle_control_slew_rate.update(_throttle_control, control_interval);

	} else {
		_throttle_control_slew_rate.setForcedValue(0.f);
	}

	publishControl(now, user_request);
}

void InternalCombustionEngineControl::publishControl(const hrt_abstime now, const UserOnOffRequest user_request)
{
	internal_combustion_engine_control_s ice_control{};
	ice_control.timestamp = now;
	ice_control.choke_control = _choke_control;
	ice_control.ignition_on = _ignition_on;
	ice_control.starter_engine_control = _starter_engine_control;
	ice_control.throttle_control = _throttle_control;
	ice_control.user_request = static_cast<uint8_t>(user_request);
	_internal_combustion_engine_control_pub.publish(ice_control);

	internal_combustion_engine_status_s ice_status;
	ice_status.state = static_cast<uint8_t>(_state);
	ice_status.timestamp = now;
	_internal_combustion_engine_status_pub.publish(ice_status);
}

bool InternalCombustionEngineControl::isEngineRunning(const hrt_abstime now)
{
	rpm_s rpm;

	if (_rpm_sub.copy(&rpm)) {
		const hrt_abstime rpm_timestamp = rpm.timestamp;

		return (_param_ice_min_run_rpm.get() > FLT_EPSILON && (now < rpm_timestamp + 2_s)
			&& rpm.rpm_estimate > _param_ice_min_run_rpm.get());
	}

	return false;
}

void InternalCombustionEngineControl::controlEngineRunning(float throttle_in)
{
	_ignition_on = true;
	_choke_control = 0.f;
	_starter_engine_control = 0.f;
	_throttle_control = throttle_in;

}

void InternalCombustionEngineControl::controlEngineStop()
{
	_ignition_on = false;
	_choke_control = _param_ice_stop_choke.get() ? 1.f : 0.f;
	_starter_engine_control = 0.f;
	_throttle_control = 0.f;
}

void InternalCombustionEngineControl::controlEngineFault()
{
	_ignition_on = false;
	_choke_control = _param_ice_stop_choke.get() ? 1.f : 0.f;
	_starter_engine_control = 0.f;
	_throttle_control = 0.f;
}

void InternalCombustionEngineControl::controlEngineStartup(const hrt_abstime now)
{
	float ignition_delay = 0.f;
	float choke_duration = 0.f;
	const float starter_duration = _param_ice_strt_dur.get();

	if (_starting_retry_cycle == 0) {
		ignition_delay = math::max(_param_ice_ign_delay.get(), 0.f);

		if (_param_ice_choke_st_dur.get() > FLT_EPSILON) {
			choke_duration = _param_ice_choke_st_dur.get();
		}
	}

	_ignition_on = true;
	_throttle_control = _param_ice_strt_thr.get();
	_choke_control = now < _state_start_time + (choke_duration + ignition_delay) * 1_s ? 1.f : 0.f;
	_starter_engine_control = now > _state_start_time + (ignition_delay * 1_s) ? 1.f : 0.f;
	const hrt_abstime cycle_timeout_duration = (ignition_delay + choke_duration + starter_duration) * 1_s;

	if (now > _state_start_time + cycle_timeout_duration) {
		// start resting timer if engine is not running
		_starting_rest_time = now;
		_starting_retry_cycle++;
		PX4_INFO("ICE: starting attempt %i finished", _starting_retry_cycle);
	}
}

bool InternalCombustionEngineControl::isStartingPermitted(const hrt_abstime now)
{
	return now > _starting_rest_time + DELAY_BEFORE_RESTARTING * 1_s;
}

bool InternalCombustionEngineControl::maximumAttemptsReached()
{
	// First and only attempt
	if (_param_ice_strt_attempts.get() == 0) {
		return _starting_retry_cycle > 0;
	}

	return _starting_retry_cycle >= _param_ice_strt_attempts.get();
}

int InternalCombustionEngineControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
		
The module controls internal combustion engine (ICE) features including:
ignition (on/off), throttle and choke level, starter engine delay, and user request.

### Enabling

This feature is not enabled by default needs to be configured in the
build target for your board together with the rpm capture driver:

```
CONFIG_MODULES_INTERNAL_COMBUSTION_ENGINE_CONTROL=y
CONFIG_DRIVERS_RPM_CAPTURE=y
```

Additionally, to enable the module:

- Set [ICE_EN](../advanced_config/parameter_reference.md#ICE_EN)
to true and adjust the other `ICE_` module parameters according to your needs.
- Set [RPM_CAP_ENABLE](../advanced_config/parameter_reference.md#RPM_CAP_ENABLE) to true.

The module outputs control signals for ignition, throttle, and choke,
and takes inputs from an RPM sensor.
These must be mapped to AUX outputs/inputs in the [Actuator configuration](../config/actuators.md),
similar to the setup shown below.

![Actuator setup for ICE](../../assets/hardware/ice/ice_actuator_setup.png)

### Implementation

The ICE is implemented with a (4) state machine:

![Architecture](../../assets/hardware/ice/ice_control_state_machine.png)

The state machine:
		
- Checks if [Rpm.msg](../msg_docs/Rpm.md) is updated to know if the engine is running
- Allows for user inputs from:
  - AUX{N}
  - Arming state in [VehicleStatus.msg](../msg_docs/VehicleStatus.md)

The module publishes [InternalCombustionEngineControl.msg](../msg_docs/InternalCombustionEngineControl.md).
		
The architecture is as shown below:

![Architecture](../../assets/hardware/ice/ice_control_diagram.png)
		
<a id="internal_combustion_engine_control_usage"></a>
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("internal_combustion_engine_control", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int internal_combustion_engine_control_main(int argc, char *argv[])
{
	return InternalCombustionEngineControl::main(argc, argv);
}

} // namespace internal_combustion_engine_control
