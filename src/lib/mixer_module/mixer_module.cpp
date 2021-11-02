/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "mixer_module.hpp"

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>

#include <uORB/Publication.hpp>
#include <px4_platform_common/log.h>

using namespace time_literals;


struct FunctionProvider {
	using Constructor = FunctionProviderBase * (*)(const FunctionProviderBase::Context &context);
	FunctionProvider(OutputFunction min_func_, OutputFunction max_func_, Constructor constructor_)
		: min_func(min_func_), max_func(max_func_), constructor(constructor_) {}
	FunctionProvider(OutputFunction func, Constructor constructor_)
		: min_func(func), max_func(func), constructor(constructor_) {}

	OutputFunction min_func;
	OutputFunction max_func;
	Constructor constructor;
};

static const FunctionProvider all_function_providers[] = {
	// Providers higher up take precedence for subscription callback in case there are multiple
	{OutputFunction::Constant_Min, &FunctionConstantMin::allocate},
	{OutputFunction::Constant_Max, &FunctionConstantMax::allocate},
	{OutputFunction::Motor1, OutputFunction::MotorMax, &FunctionMotors::allocate},
	{OutputFunction::Servo1, OutputFunction::ServoMax, &FunctionServos::allocate},
	{OutputFunction::Offboard_Actuator_Set1, OutputFunction::Offboard_Actuator_Set6, &FunctionActuatorSet::allocate},
	{OutputFunction::Landing_Gear, &FunctionLandingGear::allocate},
	{OutputFunction::Parachute, &FunctionParachute::allocate},
	{OutputFunction::RC_Roll, OutputFunction::RC_AUXMax, &FunctionManualRC::allocate},
	{OutputFunction::Gimbal_Roll, OutputFunction::Gimbal_Yaw, &FunctionGimbal::allocate},
};

MixingOutput::MixingOutput(const char *param_prefix, uint8_t max_num_outputs, OutputModuleInterface &interface,
			   SchedulingPolicy scheduling_policy,
			   bool support_esc_calibration, bool ramp_up)
	: ModuleParams(&interface),
	  _control_subs{
	{&interface, ORB_ID(actuator_controls_0)},
	{&interface, ORB_ID(actuator_controls_1)},
	{&interface, ORB_ID(actuator_controls_2)},
	{&interface, ORB_ID(actuator_controls_3)},
},
_scheduling_policy(scheduling_policy),
_support_esc_calibration(support_esc_calibration),
_max_num_outputs(max_num_outputs < MAX_ACTUATORS ? max_num_outputs : MAX_ACTUATORS),
_interface(interface),
_control_latency_perf(perf_alloc(PC_ELAPSED, "control latency")),
_param_prefix(param_prefix)
{
	output_limit_init(&_output_limit);
	_output_limit.ramp_up = ramp_up;

	/* Safely initialize armed flags */
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;

	px4_sem_init(&_lock, 0, 1);

	_use_dynamic_mixing = _param_sys_ctrl_alloc.get();

	if (_use_dynamic_mixing) {
		initParamHandles();

		for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
			_failsafe_value[i] = UINT16_MAX;
		}

		updateParams();
	}

	_outputs_pub.advertise();
}

MixingOutput::~MixingOutput()
{
	perf_free(_control_latency_perf);
	delete _mixers;
	px4_sem_destroy(&_lock);

	cleanupFunctions();

	_outputs_pub.unadvertise();
}

void MixingOutput::initParamHandles()
{
	char param_name[17];

	for (unsigned i = 0; i < _max_num_outputs; ++i) {
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FUNC", i + 1);
		_param_handles[i].function = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "DIS", i + 1);
		_param_handles[i].disarmed = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MIN", i + 1);
		_param_handles[i].min = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MAX", i + 1);
		_param_handles[i].max = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FAIL", i + 1);
		_param_handles[i].failsafe = param_find(param_name);
	}
}

void MixingOutput::printStatus() const
{
	PX4_INFO("Param prefix: %s", _param_prefix);
	perf_print_counter(_control_latency_perf);

	if (_wq_switched) {
		PX4_INFO("Switched to rate_ctrl work queue");
	}


	if (!_use_dynamic_mixing) {
		PX4_INFO("Mixer loaded: %s", _mixers ? "yes" : "no");
		PX4_INFO("Driver instance: %i", _driver_instance);
	}

	PX4_INFO_RAW("Channel Configuration:\n");

	if (_use_dynamic_mixing) {
		for (unsigned i = 0; i < _max_num_outputs; i++) {
			PX4_INFO_RAW("Channel %i: func: %i, value: %i, failsafe: %d, disarmed: %d, min: %d, max: %d\n", i,
				     (int)_function_assignment[i], _current_output_value[i],
				     _failsafe_value[i], _disarmed_value[i], _min_value[i], _max_value[i]);
		}

	} else {
		for (unsigned i = 0; i < _max_num_outputs; i++) {
			int reordered_i = reorderedMotorIndex(i);
			PX4_INFO_RAW("Channel %i: value: %i, failsafe: %d, disarmed: %d, min: %d, max: %d\n", reordered_i,
				     _current_output_value[i],
				     _failsafe_value[reordered_i], _disarmed_value[reordered_i], _min_value[reordered_i], _max_value[reordered_i]);
		}
	}
}

void MixingOutput::updateParams()
{
	ModuleParams::updateParams();

	// update mixer if we have one
	if (_mixers) {
		if (_param_mot_slew_max.get() <= FLT_EPSILON) {
			_mixers->set_max_delta_out_once(0.f);
		}

		_mixers->set_thrust_factor(_param_thr_mdl_fac.get());
		_mixers->set_airmode((Mixer::Airmode)_param_mc_airmode.get());
	}

	if (_use_dynamic_mixing) {
		_param_mot_ordering.set(0); // not used with dynamic mixing

		bool function_changed = false;

		_reverse_output_mask = 0;

		for (unsigned i = 0; i < _max_num_outputs; i++) {
			int32_t val;

			if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &val) == 0) {
				if (val != (int32_t)_function_assignment[i]) {
					function_changed = true;
				}

				// we set _function_assignment[i] later to ensure _functions[i] is updated at the same time
			}

			if (_param_handles[i].disarmed != PARAM_INVALID && param_get(_param_handles[i].disarmed, &val) == 0) {
				_disarmed_value[i] = val;
			}

			if (_param_handles[i].min != PARAM_INVALID && param_get(_param_handles[i].min, &val) == 0) {
				_min_value[i] = val;
			}

			if (_param_handles[i].max != PARAM_INVALID && param_get(_param_handles[i].max, &val) == 0) {
				_max_value[i] = val;
			}

			if (_min_value[i] > _max_value[i]) {
				_reverse_output_mask |= 1 << i;
				uint16_t tmp = _min_value[i];
				_min_value[i] = _max_value[i];
				_max_value[i] = tmp;
			}

			if (_param_handles[i].failsafe != PARAM_INVALID && param_get(_param_handles[i].failsafe, &val) == 0) {
				_failsafe_value[i] = val;
			}
		}

		if (function_changed) {
			_need_function_update = true;
		}
	}
}

bool MixingOutput::updateSubscriptions(bool allow_wq_switch, bool limit_callbacks_to_primary)
{
	if (_use_dynamic_mixing) {
		return updateSubscriptionsDynamicMixer(allow_wq_switch, limit_callbacks_to_primary);

	} else {
		return updateSubscriptionsStaticMixer(allow_wq_switch, limit_callbacks_to_primary);
	}
}

bool MixingOutput::updateSubscriptionsStaticMixer(bool allow_wq_switch, bool limit_callbacks_to_primary)
{
	if (_groups_subscribed == _groups_required) {
		return false;
	}

	// must be locked to potentially change WorkQueue
	lock();

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		// first clear everything
		unregister();
		_interface.ScheduleClear();

		// if subscribed to control group 0 or 1 then move to the rate_ctrl WQ
		const bool sub_group_0 = (_groups_required & (1 << 0));
		const bool sub_group_1 = (_groups_required & (1 << 1));

		if (allow_wq_switch && !_wq_switched && (sub_group_0 || sub_group_1)) {
			if (_interface.ChangeWorkQeue(px4::wq_configurations::rate_ctrl)) {
				// let the new WQ handle the subscribe update
				_wq_switched = true;
				_interface.ScheduleNow();
				unlock();
				return false;
			}
		}

		bool sub_group_0_callback_registered = false;
		bool sub_group_1_callback_registered = false;

		// register callback to all required actuator control groups
		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {

			if (limit_callbacks_to_primary) {
				// don't register additional callbacks if actuator_controls_0 or actuator_controls_1 are already registered
				if ((i > 1) && (sub_group_0_callback_registered || sub_group_1_callback_registered)) {
					break;
				}
			}

			if (_groups_required & (1 << i)) {
				if (_control_subs[i].registerCallback()) {
					PX4_DEBUG("subscribed to actuator_controls_%d", i);

					if (limit_callbacks_to_primary) {
						if (i == 0) {
							sub_group_0_callback_registered = true;

						} else if (i == 1) {
							sub_group_1_callback_registered = true;
						}
					}

				} else {
					PX4_ERR("actuator_controls_%d register callback failed!", i);
				}
			}
		}

		// if nothing required keep periodic schedule (so the module can update other things)
		if (_groups_required == 0) {
			// TODO: this might need to be configurable depending on the module
			_interface.ScheduleOnInterval(100_ms);
		}
	}

	_groups_subscribed = _groups_required;
	setMaxTopicUpdateRate(_max_topic_update_interval_us);

	PX4_DEBUG("_groups_required 0x%08" PRIx32, _groups_required);
	PX4_DEBUG("_groups_subscribed 0x%08" PRIx32, _groups_subscribed);

	unlock();

	return true;
}


void MixingOutput::cleanupFunctions()
{
	if (_subscription_callback) {
		_subscription_callback->unregisterCallback();
		_subscription_callback = nullptr;
	}

	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		delete _function_allocated[i];
		_function_allocated[i] = nullptr;
		_functions[i] = nullptr;
	}
}

bool MixingOutput::updateSubscriptionsDynamicMixer(bool allow_wq_switch, bool limit_callbacks_to_primary)
{
	if (!_need_function_update || _armed.armed) {
		return false;
	}

	// must be locked to potentially change WorkQueue
	lock();

	_has_backup_schedule = false;

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		// first clear everything
		unregister();
		_interface.ScheduleClear();

		bool switch_requested = false;

		// potentially switch work queue if we run motor outputs
		for (unsigned i = 0; i < _max_num_outputs; i++) {
			// read function directly from param, as _function_assignment[i] is updated later
			int32_t function;

			if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &function) == 0) {
				if (function >= (int32_t)OutputFunction::Motor1 && function <= (int32_t)OutputFunction::MotorMax) {
					switch_requested = true;
				}
			}
		}

		if (allow_wq_switch && !_wq_switched && switch_requested) {
			if (_interface.ChangeWorkQeue(px4::wq_configurations::rate_ctrl)) {
				// let the new WQ handle the subscribe update
				_wq_switched = true;
				_interface.ScheduleNow();
				unlock();
				return false;
			}
		}
	}

	// Now update the functions
	PX4_DEBUG("updating functions");

	cleanupFunctions();

	const FunctionProviderBase::Context context{_interface, _reversible_motors};
	int provider_indexes[MAX_ACTUATORS] {};
	int next_provider = 0;
	int subscription_callback_provider_index = INT_MAX;
	bool all_disabled = true;

	for (int i = 0; i < _max_num_outputs; ++i) {
		int32_t val;

		if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &val) == 0) {
			_function_assignment[i] = (OutputFunction)val;

		} else {
			_function_assignment[i] = OutputFunction::Disabled;
		}

		for (int p = 0; p < (int)(sizeof(all_function_providers) / sizeof(all_function_providers[0])); ++p) {
			if (_function_assignment[i] >= all_function_providers[p].min_func &&
			    _function_assignment[i] <= all_function_providers[p].max_func) {
				all_disabled = false;
				int found_index = -1;

				for (int existing = 0; existing < next_provider; ++existing) {
					if (provider_indexes[existing] == p) {
						found_index = existing;
						break;
					}
				}

				if (found_index >= 0) {
					_functions[i] = _function_allocated[found_index];

				} else {
					_function_allocated[next_provider] = all_function_providers[p].constructor(context);

					if (_function_allocated[next_provider]) {
						_functions[i] = _function_allocated[next_provider];
						provider_indexes[next_provider++] = p;

						// lowest provider takes precedence for scheduling
						if (p < subscription_callback_provider_index && _functions[i]->subscriptionCallback()) {
							subscription_callback_provider_index = p;
							_subscription_callback = _functions[i]->subscriptionCallback();
						}

					} else {
						PX4_ERR("function alloc failed");
					}
				}

				break;
			}
		}
	}

	hrt_abstime fixed_rate_scheduling_interval = 4_ms; // schedule at 250Hz

	if (_max_topic_update_interval_us > fixed_rate_scheduling_interval) {
		fixed_rate_scheduling_interval = _max_topic_update_interval_us;
	}

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		if (_subscription_callback) {
			if (_subscription_callback->registerCallback()) {
				PX4_DEBUG("Scheduling via callback");
				_has_backup_schedule = true;
				_interface.ScheduleDelayed(50_ms);

			} else {
				PX4_ERR("registerCallback failed, scheduling at fixed rate");
				_interface.ScheduleOnInterval(fixed_rate_scheduling_interval);
			}

		} else if (all_disabled) {
			_interface.ScheduleOnInterval(_lowrate_schedule_interval);
			PX4_DEBUG("Scheduling at low rate");

		} else {
			_interface.ScheduleOnInterval(fixed_rate_scheduling_interval);
			PX4_DEBUG("Scheduling at fixed rate");
		}
	}

	setMaxTopicUpdateRate(_max_topic_update_interval_us);
	_need_function_update = false;

	unlock();

	_interface.mixerChanged();

	return true;
}

void MixingOutput::setMaxTopicUpdateRate(unsigned max_topic_update_interval_us)
{
	_max_topic_update_interval_us = max_topic_update_interval_us;

	if (_use_dynamic_mixing) {
		if (_subscription_callback) {
			_subscription_callback->set_interval_us(_max_topic_update_interval_us);
		}

	} else {
		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_groups_subscribed & (1 << i)) {
				_control_subs[i].set_interval_us(_max_topic_update_interval_us);
			}
		}
	}
}

void MixingOutput::setAllMinValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].min = PARAM_INVALID;
		_min_value[i] = value;
	}
}

void MixingOutput::setAllMaxValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].max = PARAM_INVALID;
		_max_value[i] = value;
	}
}

void MixingOutput::setAllFailsafeValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].failsafe = PARAM_INVALID;
		_failsafe_value[i] = value;
	}
}

void MixingOutput::setAllDisarmedValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].disarmed = PARAM_INVALID;
		_disarmed_value[i] = value;
	}
}

void MixingOutput::unregister()
{
	for (auto &control_sub : _control_subs) {
		control_sub.unregisterCallback();
	}

	if (_subscription_callback) {
		_subscription_callback->unregisterCallback();
	}
}

void MixingOutput::updateOutputSlewrateMultirotorMixer()
{
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _time_last_dt_update_multicopter) / 1e6f, 0.0001f, 0.02f);
	_time_last_dt_update_multicopter = now;

	// maximum value the outputs of the multirotor mixer are allowed to change in this cycle
	// factor 2 is needed because actuator outputs are in the range [-1,1]
	const float delta_out_max = 2.0f * 1000.0f * dt / (_max_value[0] - _min_value[0]) / _param_mot_slew_max.get();
	_mixers->set_max_delta_out_once(delta_out_max);
}

void MixingOutput::updateOutputSlewrateSimplemixer()
{
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _time_last_dt_update_simple_mixer) / 1e6f, 0.0001f, 0.02f);
	_time_last_dt_update_simple_mixer = now;

	// set dt for slew rate limiter in SimpleMixer (is reset internally after usig it, so needs to be set on every update)
	_mixers->set_dt_once(dt);
}


unsigned MixingOutput::motorTest()
{
	test_motor_s test_motor;
	bool had_update = false;

	while (_motor_test.test_motor_sub.update(&test_motor)) {
		if (test_motor.driver_instance != _driver_instance ||
		    test_motor.timestamp == 0 ||
		    hrt_elapsed_time(&test_motor.timestamp) > 100_ms) {
			continue;
		}

		bool in_test_mode = test_motor.action == test_motor_s::ACTION_RUN;

		if (in_test_mode != _motor_test.in_test_mode) {
			// reset all outputs to disarmed on state change
			for (int i = 0; i < MAX_ACTUATORS; ++i) {
				_current_output_value[i] = _disarmed_value[i];
			}
		}

		if (in_test_mode) {
			int idx = test_motor.motor_number;

			if (idx < MAX_ACTUATORS) {
				if (test_motor.value < 0.f) {
					_current_output_value[reorderedMotorIndex(idx)] = _disarmed_value[idx];

				} else {
					_current_output_value[reorderedMotorIndex(idx)] =
						math::constrain<uint16_t>(_min_value[idx] + (uint16_t)((_max_value[idx] - _min_value[idx]) * test_motor.value),
									  _min_value[idx], _max_value[idx]);
				}
			}

			if (test_motor.timeout_ms > 0) {
				_motor_test.timeout = test_motor.timestamp + test_motor.timeout_ms * 1000;

			} else {
				_motor_test.timeout = 0;
			}
		}

		_motor_test.in_test_mode = in_test_mode;
		had_update = true;
	}

	// check for timeouts
	if (_motor_test.timeout != 0 && hrt_absolute_time() > _motor_test.timeout) {
		_motor_test.in_test_mode = false;
		_motor_test.timeout = 0;

		for (int i = 0; i < MAX_ACTUATORS; ++i) {
			_current_output_value[i] = _disarmed_value[i];
		}

		had_update = true;
	}

	return (_motor_test.in_test_mode || had_update) ? _max_num_outputs : 0;
}

bool MixingOutput::update()
{
	if (_use_dynamic_mixing) {
		return updateDynamicMixer();

	} else {
		return updateStaticMixer();
	}
}
bool MixingOutput::updateStaticMixer()
{
	if (!_mixers) {
		handleCommands();
		// do nothing until we have a valid mixer
		return false;
	}

	// check arming state
	if (_armed_sub.update(&_armed)) {
		_armed.in_esc_calibration_mode &= _support_esc_calibration;

		if (_ignore_lockdown) {
			_armed.lockdown = false;
		}

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		_throttle_armed = (_armed.armed && !_armed.lockdown) || _armed.in_esc_calibration_mode;

		if (_armed.armed) {
			_motor_test.in_test_mode = false;
		}
	}

	if (_param_mot_slew_max.get() > FLT_EPSILON) {
		updateOutputSlewrateMultirotorMixer();
	}

	updateOutputSlewrateSimplemixer(); // update dt for output slew rate in simple mixer

	unsigned n_updates = 0;

	/* get controls for required topics */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_groups_subscribed & (1 << i)) {
			if (_control_subs[i].copy(&_controls[i])) {
				n_updates++;
			}

			/* During ESC calibration, we overwrite the throttle value. */
			if (i == 0 && _armed.in_esc_calibration_mode) {

				/* Set all controls to 0 */
				memset(&_controls[i], 0, sizeof(_controls[i]));

				/* except thrust to maximum. */
				_controls[i].control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;

				/* Switch off the output limit ramp for the calibration. */
				_output_limit.state = OUTPUT_LIMIT_STATE_ON;
			}
		}
	}

	// check for motor test (after topic updates)
	if (!_armed.armed && !_armed.manual_lockdown) {
		unsigned num_motor_test = motorTest();

		if (num_motor_test > 0) {
			if (_interface.updateOutputs(false, _current_output_value, num_motor_test, 1)) {
				actuator_outputs_s actuator_outputs{};
				setAndPublishActuatorOutputs(num_motor_test, actuator_outputs);
			}

			handleCommands();
			return true;
		}
	}

	/* do mixing */
	float outputs[MAX_ACTUATORS] {};
	const unsigned mixed_num_outputs = _mixers->mix(outputs, _max_num_outputs);

	/* the output limit call takes care of out of band errors, NaN and constrains */
	output_limit_calc(_throttle_armed, armNoThrottle(), mixed_num_outputs, _reverse_output_mask,
			  _disarmed_value, _min_value, _max_value, outputs, _current_output_value, &_output_limit);

	/* overwrite outputs in case of force_failsafe with _failsafe_value values */
	if (_armed.force_failsafe) {
		for (size_t i = 0; i < mixed_num_outputs; i++) {
			_current_output_value[i] = _failsafe_value[i];
		}
	}

	bool stop_motors = mixed_num_outputs == 0 || !_throttle_armed;

	/* overwrite outputs in case of lockdown with disarmed values */
	if (_armed.lockdown || _armed.manual_lockdown) {
		for (size_t i = 0; i < mixed_num_outputs; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		stop_motors = true;
	}

	/* apply _param_mot_ordering */
	reorderOutputs(_current_output_value);

	/* now return the outputs to the driver */
	if (_interface.updateOutputs(stop_motors, _current_output_value, mixed_num_outputs, n_updates)) {
		actuator_outputs_s actuator_outputs{};
		setAndPublishActuatorOutputs(mixed_num_outputs, actuator_outputs);

		publishMixerStatus(actuator_outputs);
		updateLatencyPerfCounter(actuator_outputs);
	}

	handleCommands();

	return true;
}

bool MixingOutput::updateDynamicMixer()
{
	// check arming state
	if (_armed_sub.update(&_armed)) {
		_armed.in_esc_calibration_mode &= _support_esc_calibration;

		if (_ignore_lockdown) {
			_armed.lockdown = false;
		}

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		_throttle_armed = (_armed.armed && !_armed.lockdown) || _armed.in_esc_calibration_mode;
	}

	// only used for sitl with lockstep
	bool has_updates = _subscription_callback && _subscription_callback->updated();

	// update topics
	for (int i = 0; i < MAX_ACTUATORS && _function_allocated[i]; ++i) {
		_function_allocated[i]->update();
	}

	if (_has_backup_schedule) {
		_interface.ScheduleDelayed(50_ms);
	}

	// check for motor test (after topic updates)
	if (!_armed.armed && !_armed.manual_lockdown) {
		// TODO
	}

	// get output values
	float outputs[MAX_ACTUATORS];
	bool all_disabled = true;

	for (int i = 0; i < _max_num_outputs; ++i) {
		if (_functions[i]) {
			all_disabled = false;

			if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
				outputs[i] = _functions[i]->value(_function_assignment[i]);

			} else {
				outputs[i] = NAN;
			}

		} else {
			outputs[i] = NAN;
		}
	}

	if (!all_disabled) {
		limitAndUpdateOutputs(outputs, has_updates);
	}

	return true;
}

void
MixingOutput::limitAndUpdateOutputs(float outputs[MAX_ACTUATORS], bool has_updates)
{
	/* the output limit call takes care of out of band errors, NaN and constrains */
	output_limit_calc(_throttle_armed, armNoThrottle(), _max_num_outputs, _reverse_output_mask,
			  _disarmed_value, _min_value, _max_value, outputs, _current_output_value, &_output_limit);

	/* overwrite outputs in case of force_failsafe with _failsafe_value values */
	if (_armed.force_failsafe) {
		for (size_t i = 0; i < _max_num_outputs; i++) {
			_current_output_value[i] = actualFailsafeValue(i);
		}
	}

	bool stop_motors = !_throttle_armed;

	/* overwrite outputs in case of lockdown with disarmed values */
	if (_armed.lockdown || _armed.manual_lockdown) {
		for (size_t i = 0; i < _max_num_outputs; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		stop_motors = true;
	}

	/* now return the outputs to the driver */
	if (_interface.updateOutputs(stop_motors, _current_output_value, _max_num_outputs, has_updates)) {
		actuator_outputs_s actuator_outputs{};
		setAndPublishActuatorOutputs(_max_num_outputs, actuator_outputs);

		updateLatencyPerfCounter(actuator_outputs);
	}
}

void
MixingOutput::setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs)
{
	actuator_outputs.noutputs = num_outputs;

	for (size_t i = 0; i < num_outputs; ++i) {
		actuator_outputs.output[i] = _current_output_value[i];
	}

	actuator_outputs.timestamp = hrt_absolute_time();
	_outputs_pub.publish(actuator_outputs);
}

void
MixingOutput::publishMixerStatus(const actuator_outputs_s &actuator_outputs)
{
	MultirotorMixer::saturation_status saturation_status;
	saturation_status.value = _mixers->get_saturation_status();

	if (saturation_status.flags.valid) {
		control_allocator_status_s sat{};
		sat.timestamp = hrt_absolute_time();
		sat.torque_setpoint_achieved = true;
		sat.thrust_setpoint_achieved = true;

		// Note: the values '-1', '1' and '0' are just to indicate a negative,
		// positive or no saturation to the rate controller. The actual magnitude
		// is not used.
		if (saturation_status.flags.roll_pos) {
			sat.unallocated_torque[0] = 1.f;
			sat.torque_setpoint_achieved = false;

		} else if (saturation_status.flags.roll_neg) {
			sat.unallocated_torque[0] = -1.f;
			sat.torque_setpoint_achieved = false;
		}

		if (saturation_status.flags.pitch_pos) {
			sat.unallocated_torque[1] = 1.f;
			sat.torque_setpoint_achieved = false;

		} else if (saturation_status.flags.pitch_neg) {
			sat.unallocated_torque[1] = -1.f;
			sat.torque_setpoint_achieved = false;
		}

		if (saturation_status.flags.yaw_pos) {
			sat.unallocated_torque[2] = 1.f;
			sat.torque_setpoint_achieved = false;

		} else if (saturation_status.flags.yaw_neg) {
			sat.unallocated_torque[2] = -1.f;
			sat.torque_setpoint_achieved = false;
		}

		if (saturation_status.flags.thrust_pos) {
			sat.unallocated_thrust[2] = 1.f;
			sat.thrust_setpoint_achieved = false;

		} else if (saturation_status.flags.thrust_neg) {
			sat.unallocated_thrust[2] = -1.f;
			sat.thrust_setpoint_achieved = false;
		}

		_control_allocator_status_pub.publish(sat);
	}
}

void
MixingOutput::updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs)
{
	if (_use_dynamic_mixing) {
		// Just check the first function. It means we only get the latency if motors are assigned first, which is the default
		if (_function_allocated[0]) {
			hrt_abstime timestamp_sample;

			if (_function_allocated[0]->getLatestSampleTimestamp(timestamp_sample)) {
				perf_set_elapsed(_control_latency_perf, actuator_outputs.timestamp - timestamp_sample);
			}
		}

	} else {
		// use first valid timestamp_sample for latency tracking
		for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			const bool required = _groups_required & (1 << i);
			const hrt_abstime &timestamp_sample = _controls[i].timestamp_sample;

			if (required && (timestamp_sample > 0)) {
				perf_set_elapsed(_control_latency_perf, actuator_outputs.timestamp - timestamp_sample);
				break;
			}
		}
	}
}

uint16_t
MixingOutput::actualFailsafeValue(int index)
{
	if (!_use_dynamic_mixing) {
		return failsafeValue(index);
	}

	uint16_t value = 0;

	if (_failsafe_value[index] == UINT16_MAX) { // if set to default, use the one provided by the function
		float default_failsafe = NAN;

		if (_functions[index]) {
			default_failsafe = _functions[index]->defaultFailsafeValue(_function_assignment[index]);
		}

		value = output_limit_calc_single(_reverse_output_mask & (1 << index),
						 _disarmed_value[index], _min_value[index], _max_value[index], default_failsafe);

	} else {
		value = _failsafe_value[index];
	}

	return value;
}

void
MixingOutput::reorderOutputs(uint16_t values[MAX_ACTUATORS])
{
	if (MAX_ACTUATORS < 4) {
		return;
	}

	if ((MotorOrdering)_param_mot_ordering.get() == MotorOrdering::Betaflight) {
		/*
		 * Betaflight default motor ordering:
		 * 4     2
		 *    ^
		 * 3     1
		 */
		const uint16_t value_tmp[4] = {values[0], values[1], values[2], values[3] };
		values[0] = value_tmp[3];
		values[1] = value_tmp[0];
		values[2] = value_tmp[1];
		values[3] = value_tmp[2];
	}

	/* else: PX4, no need to reorder
	 * 3     1
	 *    ^
	 * 2     4
	 */
}

int MixingOutput::reorderedMotorIndex(int index) const
{
	if ((MotorOrdering)_param_mot_ordering.get() == MotorOrdering::Betaflight) {
		switch (index) {
		case 0: return 1;

		case 1: return 2;

		case 2: return 3;

		case 3: return 0;
		}
	}

	return index;
}

int MixingOutput::controlCallback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	const MixingOutput *output = (const MixingOutput *)handle;

	input = output->_controls[control_group].control[control_index];

	/* limit control input */
	input = math::constrain(input, -1.f, 1.f);

	/* motor spinup phase - lock throttle to zero */
	if (output->_output_limit.state == OUTPUT_LIMIT_STATE_RAMP) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (output->armNoThrottle() && !output->_armed.in_esc_calibration_mode) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN;
		}
	}

	return 0;
}

void MixingOutput::resetMixer()
{
	if (_mixers != nullptr) {
		delete _mixers;
		_mixers = nullptr;
		_groups_required = 0;
	}

	_interface.mixerChanged();
}

int MixingOutput::loadMixer(const char *buf, unsigned len)
{
	if (_mixers == nullptr) {
		_mixers = new MixerGroup();
	}

	if (_mixers == nullptr) {
		_groups_required = 0;
		return -ENOMEM;
	}

	int ret = _mixers->load_from_buf(controlCallback, (uintptr_t)this, buf, len);

	if (ret != 0) {
		PX4_ERR("mixer load failed with %d", ret);
		delete _mixers;
		_mixers = nullptr;
		_groups_required = 0;
		return ret;
	}

	_mixers->groups_required(_groups_required);
	PX4_DEBUG("loaded mixers \n%s\n", buf);

	updateParams();
	_interface.mixerChanged();
	return ret;
}

void MixingOutput::handleCommands()
{
	if ((Command::Type)_command.command.load() == Command::Type::None) {
		return;
	}

	switch ((Command::Type)_command.command.load()) {
	case Command::Type::loadMixer:
		_command.result = loadMixer(_command.mixer_buf, _command.mixer_buf_length);
		break;

	case Command::Type::resetMixer:
		resetMixer();
		_command.result = 0;
		break;

	default:
		break;
	}

	// mark as done
	_command.command.store((int)Command::Type::None);
}

void MixingOutput::resetMixerThreadSafe()
{
	if (_use_dynamic_mixing) {
		PX4_ERR("mixer reset unavailable, not using static mixers");
		return;
	}

	if ((Command::Type)_command.command.load() != Command::Type::None) {
		// Cannot happen, because we expect only one other thread to call this.
		// But as a safety precaution we return here.
		PX4_ERR("Command not None");
		return;
	}

	lock();

	_command.command.store((int)Command::Type::resetMixer);

	_interface.ScheduleNow();

	unlock();

	// wait until processed
	while ((Command::Type)_command.command.load() != Command::Type::None) {
		usleep(1000);
	}

}

int MixingOutput::loadMixerThreadSafe(const char *buf, unsigned len)
{
	if (_use_dynamic_mixing) {
		PX4_ERR("mixer load unavailable, not using static mixers");
		return -1;
	}

	if ((Command::Type)_command.command.load() != Command::Type::None) {
		// Cannot happen, because we expect only one other thread to call this.
		// But as a safety precaution we return here.
		PX4_ERR("Command not None");
		return -1;
	}

	lock();

	_command.mixer_buf = buf;
	_command.mixer_buf_length = len;
	_command.command.store((int)Command::Type::loadMixer);

	_interface.ScheduleNow();

	unlock();

	// wait until processed
	while ((Command::Type)_command.command.load() != Command::Type::None) {
		usleep(1000);
	}

	return _command.result;
}
