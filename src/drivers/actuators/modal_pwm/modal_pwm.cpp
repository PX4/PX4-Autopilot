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

#include "modal_pwm.hpp"

#include <px4_platform_common/sem.hpp>

ModalPWM::ModalPWM() :
	// OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
	OutputModuleInterface(MODULE_NAME, px4::serial_port_to_wq(MODAL_PWM_DEFAULT_PORT)),
	_mixing_output{"Modal PWM", MODAL_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false},
	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")},
	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")}
{
	_device = MODAL_PWM_DEFAULT_PORT;

	_pwm_mask = ((1u << MODAL_PWM_OUTPUT_CHANNELS) - 1);
	_mixing_output.setMaxNumOutputs(MODAL_PWM_OUTPUT_CHANNELS);

	// Getting initial parameter values
	update_params();
	_uart_port = new ModalIoSerial();

}

ModalPWM::~ModalPWM()
{
	/* make sure servos are off */
	// up_pwm_servo_deinit(_pwm_mask);
	stop_pwms(_pwm_mask);

	if (_uart_port) {
		_uart_port->uart_close();
		_uart_port = nullptr;
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}


void ModalPWM::update_params()
{
	int ret = PX4_ERROR;

	updateParams();
	ret = load_params(&_parameters, (ch_assign_t *)&_output_map);

	if (ret == PX4_OK) {
		_mixing_output.setAllDisarmedValues(0);
		_mixing_output.setAllFailsafeValues(0);
		_mixing_output.setAllMinValues(MODAL_PWM_DEFAULT_PWM_MIN);
		_mixing_output.setAllMaxValues(MODAL_PWM_DEFAULT_PWM_MAX);

		_pwm_fullscale = _parameters.pwm_max - _parameters.pwm_min;
	}
}
	
int ModalPWM::load_params(modal_pwm_params_t *params, ch_assign_t *map)
{
	int ret = PX4_OK;

	// initialize out
	for (int i = 0; i < MODAL_PWM_OUTPUT_CHANNELS; i++) {
		params->function_map[i] = (int)OutputFunction::Disabled;
		params->direction_map[i] = 0;
		params->motor_map[i] = 0;
	}

	// Set PWM max and min 
	params->pwm_min = MODAL_PWM_DEFAULT_PWM_MIN;
	params->pwm_max = MODAL_PWM_DEFAULT_PWM_MAX;

	return ret;
}


bool ModalPWM::update_pwm_out_state(bool on)
{
	return true;
	// if (on && !_pwm_initialized && _pwm_mask != 0) {

	// 	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
	// 		_timer_rates[timer] = -1;

	// 		uint32_t channels = io_timer_get_group(timer);

	// 		if (channels == 0) {
	// 			continue;
	// 		}

	// 		char param_name[17];
	// 		snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

	// 		int32_t tim_config = 0;
	// 		param_t handle = param_find(param_name);
	// 		param_get(handle, &tim_config);

	// 		if (tim_config > 0) {
	// 			_timer_rates[timer] = tim_config;

	// 		} else if (tim_config == -1) { // OneShot
	// 			_timer_rates[timer] = 0;

	// 		} else {
	// 			_pwm_mask &= ~channels; // don't use for pwm
	// 		}
	// 	}

	// 	int ret = up_pwm_servo_init(_pwm_mask);

	// 	if (ret < 0) {
	// 		PX4_ERR("up_pwm_servo_init failed (%i)", ret);
	// 		return false;
	// 	}

	// 	_pwm_mask = ret;

	// 	// set the timer rates
	// 	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
	// 		uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

	// 		if (channels == 0) {
	// 			continue;
	// 		}

	// 		ret = up_pwm_servo_set_rate_group_update(timer, _timer_rates[timer]);

	// 		if (ret != 0) {
	// 			PX4_ERR("up_pwm_servo_set_rate_group_update failed for timer %i, rate %i (%i)", timer, _timer_rates[timer], ret);
	// 			_timer_rates[timer] = -1;
	// 			_pwm_mask &= ~channels;
	// 		}
	// 	}

	// 	_pwm_initialized = true;

	// 	// disable unused functions
	// 	for (unsigned i = 0; i < _num_outputs; ++i) {
	// 		if (((1 << i) & _pwm_mask) == 0) {
	// 			_mixing_output.disableFunction(i);
	// 		}
	// 	}
	// }

	// up_pwm_servo_arm(on, _pwm_mask);
	// return true;
}

bool ModalPWM::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	/* output to the servos */
	// if (_pwm_initialized) {
	// 	for (size_t i = 0; i < num_outputs; i++) {
	// 		if (!_mixing_output.isFunctionSet(i)) {
	// 			// do not run any signal on disabled channels
	// 			outputs[i] = 0;
	// 		}

	// 		if (_pwm_mask & (1 << i)) {
	// 			up_pwm_servo_set(i, outputs[i]);
	// 		}
	// 	}
	// }

	/* Trigger all timer's channels in Oneshot mode to fire
	 * the oneshots with updated values.
	 */
	// if (num_control_groups_updated > 0) {
	// 	up_pwm_update(_pwm_mask);
	// }

	return true;
}

void ModalPWM::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// _mixing_output.update();

	/* update PWM status if armed or if disarmed PWM values are set */
	_pwm_on = true;

	// if (_pwm_on != pwm_on) {
	// 	if (update_pwm_out_state(pwm_on)) {
	// 		_pwm_on = pwm_on;
	// 	}
	// }

	// check for parameter updates
	// if (_parameter_update_sub.updated()) {
	// 	// clear update
	// 	parameter_update_s pupdate;
	// 	_parameter_update_sub.copy(&pupdate);

	// 	// update parameters from storage
	// 	update_params();
	// }

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	// _mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
	_first_update_cycle = false;
}

int ModalPWM::task_spawn(int argc, char *argv[])
{
	ModalPWM *instance = new ModalPWM();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}


bool stop_pwms(uint32_t pwm_mask){
	return true;
}

int ModalPWM::send_cmd_thread_safe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}


int ModalPWM::custom_command(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	Command  cmd;
	uint8_t  output_channel   = 0xF;
	int16_t  rate     = 0;

	uint32_t repeat_count    = 100;
	uint32_t repeat_delay_us = 10000;

	if (argc < 3) {
		return print_usage("unknown command");
	}

	const char *verb = argv[argc - 1];

	/* start the FMU if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return ModalPWM::task_spawn(argc, argv);
		}
	}

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;

	}

	while ((ch = px4_getopt(argc, argv, "c:n:t:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'c':
			output_channel = atoi(myoptarg);
			break;

		case 'n':
			repeat_count = atoi(myoptarg);

			if (repeat_count < 1) {
				print_usage("bad repeat_count");
				return 0;
			}

			break;

		case 't':
			repeat_delay_us = atoi(myoptarg);

			if (repeat_delay_us < 1) {
				print_usage("bad repeat delay");
				return 0;
			}

			break;

		case 'r':
			rate = atoi(myoptarg);
			break;

		default:
			print_usage("Unknown command");
			return 0;
		}
	}

	if (!strcmp(verb, "pwm")) {
		if (output_channel < MODAL_PWM_OUTPUT_CHANNELS) {
			PX4_INFO("Request PWM for Output Channel: %i - PWM: %i", output_channel, rate);
			int16_t rate_req[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
			uint8_t id_fb = 0;

			if (output_channel == 0xFF) {  //WARNING: this condition is not possible due to check 'if (esc_id < MODAL_IO_OUTPUT_CHANNELS)'.
				rate_req[0] = rate;
				rate_req[1] = rate;
				rate_req[2] = rate;
				rate_req[3] = rate;

			} else {
				rate_req[output_channel] = rate;
				id_fb = output_channel;
			}

			cmd.len = qc_esc_create_pwm_packet4_fb(rate_req[0],
							       rate_req[1],
							       rate_req[2],
							       rate_req[3],
							       0,
							       0,
							       0,
							       0,
							       id_fb,  /* ESC ID .. need to fix for correct ID.. but what about multiple ESCs in bit mask.. */
							       cmd.buf,
							       sizeof(cmd.buf));

			cmd.response        = true;
			cmd.repeats         = repeat_count;
			cmd.resp_delay_us   = 0;
			cmd.repeat_delay_us = repeat_delay_us;
			cmd.print_feedback  = true;

			PX4_INFO("feedback id debug: %i", id_fb);
			PX4_INFO("Sending UART M0065 power command %i", rate);

			return get_instance()->send_cmd_thread_safe(&cmd);
		} else {
			print_usage("Invalid Output Channel, use 0-3");
			return 0;
		}
	}

	return print_usage("unknown command");
}

int ModalPWM::print_status()
{
	// perf_print_counter(_cycle_perf);
	// perf_print_counter(_interval_perf);
	// _mixing_output.printStatus();

	// if (_pwm_initialized) {
	// 	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
	// 		if (_timer_rates[timer] >= 0) {
	// 			PX4_INFO_RAW("Timer %i: rate: %3i", timer, _timer_rates[timer]);
	// 			uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

	// 			if (channels > 0) {
	// 				PX4_INFO_RAW(" channels: ");

	// 				for (uint32_t channel = 0; channel < _num_outputs; ++channel) {
	// 					if ((1 << channel) & channels) {
	// 						PX4_INFO_RAW("%" PRIu32 " ", channel);
	// 					}
	// 				}
	// 			}

	// 			PX4_INFO_RAW("\n");
	// 		}
	// 	}
	// }

	return 0;
}

int ModalPWM::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("modal_pwm", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "Open-Loop PWM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, 3, "PWM OUTPUT Channel, 0-3", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 800, "Duty Cycle value, 0 to 800", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 100, 0, 1<<31, "Command repeat count, 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 10000, 0, 1<<31, "Delay between repeated commands (microseconds), 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int modal_pwm_main(int argc, char *argv[]);

int modal_pwm_main(int argc, char *argv[])
{
	return ModalPWM::main(argc, argv);
}
