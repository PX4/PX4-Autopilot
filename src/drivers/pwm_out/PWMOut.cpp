/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include "PWMOut.hpp"

#include <px4_platform_common/sem.hpp>

PWMOut::PWMOut() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_pwm_mask = ((1u << DIRECT_PWM_OUTPUT_CHANNELS) - 1);
	_mixing_output.setMaxNumOutputs(DIRECT_PWM_OUTPUT_CHANNELS);

	// Getting initial parameter values
	update_params();
}

PWMOut::~PWMOut()
{
	/* make sure servos are off */
	up_pwm_servo_deinit(_pwm_mask);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

bool PWMOut::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {

		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			_timer_rates[timer] = -1;

			uint32_t channels = io_timer_get_group(timer);

			if (channels == 0) {
				continue;
			}

			char param_name[17];
			snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

			int32_t tim_config = 0;
			param_t handle = param_find(param_name);
			param_get(handle, &tim_config);

			if (tim_config > 0) {
				_timer_rates[timer] = tim_config;

			} else if (tim_config == -1) { // OneShot
				_timer_rates[timer] = 0;

			} else {
				_pwm_mask &= ~channels; // don't use for pwm
			}
		}

		int ret = up_pwm_servo_init(_pwm_mask);

		if (ret < 0) {
			PX4_ERR("up_pwm_servo_init failed (%i)", ret);
			return false;
		}

		_pwm_mask = ret;

		// set the timer rates
		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

			if (channels == 0) {
				continue;
			}

			ret = up_pwm_servo_set_rate_group_update(timer, _timer_rates[timer]);

			if (ret != 0) {
				PX4_ERR("up_pwm_servo_set_rate_group_update failed for timer %i, rate %i (%i)", timer, _timer_rates[timer], ret);
				_timer_rates[timer] = -1;
				_pwm_mask &= ~channels;
			}
		}

		_pwm_initialized = true;

		// disable unused functions
		for (unsigned i = 0; i < _num_outputs; ++i) {
			if (((1 << i) & _pwm_mask) == 0) {
				_mixing_output.disableFunction(i);
			}
		}
	}

	up_pwm_servo_arm(on, _pwm_mask);
	return true;
}

bool PWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	/* output to the servos */
	if (_pwm_initialized) {
		for (size_t i = 0; i < num_outputs; i++) {
			if (!_mixing_output.isFunctionSet(i)) {
				// do not run any signal on disabled channels
				outputs[i] = 0;
			}

			if (_pwm_mask & (1 << i)) {
				up_pwm_servo_set(i, outputs[i]);
			}
		}
	}

	/* Trigger all timer's channels in Oneshot mode to fire
	 * the oneshots with updated values.
	 */
	if (num_control_groups_updated > 0) {
		up_pwm_update(_pwm_mask);
	}

	return true;
}

void PWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	/* update PWM status if armed or if disarmed PWM values are set */
	bool pwm_on = true;

	if (_pwm_on != pwm_on) {
		if (update_pwm_out_state(pwm_on)) {
			_pwm_on = pwm_on;
		}
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
	_first_update_cycle = false;
}

int PWMOut::task_spawn(int argc, char *argv[])
{
	PWMOut *instance = new PWMOut();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

void PWMOut::update_params()
{
	uint32_t previously_set_functions = 0;

	for (size_t i = 0; i < _num_outputs; i++) {
		previously_set_functions |= (uint32_t)_mixing_output.isFunctionSet(i) << i;
	}

	updateParams();

	// Automatically set the PWM rate and disarmed value when a channel is first set to a servo
	if (!_first_update_cycle) {
		for (size_t i = 0; i < _num_outputs; i++) {
			if ((previously_set_functions & (1u << i)) == 0 && _mixing_output.functionParamHandle(i) != PARAM_INVALID) {
				int32_t output_function;

				if (param_get(_mixing_output.functionParamHandle(i), &output_function) == 0
				    && output_function >= (int)OutputFunction::Servo1
				    && output_function <= (int)OutputFunction::ServoMax) { // Function got set to a servo
					int32_t val = 1500;
					PX4_INFO("Setting disarmed to %i for channel %i", (int) val, i);
					param_set(_mixing_output.disarmedParamHandle(i), &val);

					// If the whole timer group was not set previously, then set the pwm rate to 50 Hz
					for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {

						uint32_t channels = io_timer_get_group(timer);

						if ((channels & (1u << i)) == 0) {
							continue;
						}

						if ((channels & previously_set_functions) == 0) { // None of the channels was set
							char param_name[17];
							snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

							int32_t tim_config = 0;
							param_t handle = param_find(param_name);

							if (param_get(handle, &tim_config) == 0 && tim_config == 400) {
								tim_config = 50;
								PX4_INFO("setting timer %i to %i Hz", timer, (int) tim_config);
								param_set(handle, &tim_config);
							}
						}
					}
				}
			}
		}
	}
}

int PWMOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PWMOut::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	if (_pwm_initialized) {
		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			if (_timer_rates[timer] >= 0) {
				PX4_INFO_RAW("Timer %i: rate: %3i", timer, _timer_rates[timer]);
				uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

				if (channels > 0) {
					PX4_INFO_RAW(" channels: ");

					for (uint32_t channel = 0; channel < _num_outputs; ++channel) {
						if ((1 << channel) & channels) {
							PX4_INFO_RAW("%" PRIu32 " ", channel);
						}
					}
				}

				PX4_INFO_RAW("\n");
			}
		}
	}

	return 0;
}

int PWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving the output pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_main(int argc, char *argv[])
{
	return PWMOut::main(argc, argv);
}
