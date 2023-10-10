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
	OutputModuleInterface(MODULE_NAME, px4::serial_port_to_wq(MODAL_PWM_DEFAULT_PORT)),
	_mixing_output{"MODAL_PWM", MODAL_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false},
	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")},
	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")}
{
	_mixing_output.setMaxNumOutputs(MODAL_PWM_OUTPUT_CHANNELS);

	// Getting initial parameter values
	update_params();
	_uart_port = new ModalIoSerial();

}

ModalPWM::~ModalPWM()
{
	/* make sure servos are off */
	stop_all_pwms();

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
		_mixing_output.setAllFailsafeValues(MODAL_PWM_DEFAULT_PWM_FAILSAFE);
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
	
	param_get(param_find("MODAL_PWM_CONFIG"),  &params->config);
	param_get(param_find("MODAL_PWM_MODE"),    &params->mode);
	param_get(param_find("MODAL_PWM_BAUD"),    &params->baud_rate);

	param_get(param_find("MODAL_IO_FUNC1"),  &params->function_map[0]);
	param_get(param_find("MODAL_IO_FUNC2"),  &params->function_map[1]);
	param_get(param_find("MODAL_IO_FUNC3"),  &params->function_map[2]);
	param_get(param_find("MODAL_IO_FUNC4"),  &params->function_map[3]);

	param_get(param_find("MODAL_PWM_MIN"), &params->pwm_min);
	param_get(param_find("MODAL_PWM_MAX"), &params->pwm_max);
	param_get(param_find("MODAL_PWM_FS"),  &params->pwm_failsafe);

	if (params->pwm_min >= params->pwm_max) {
		PX4_ERR("Invalid parameter MODAL_PWM_MIN.  Please verify parameters.");
		params->pwm_min = 0;
		ret = PX4_ERROR;
	}

	if (params->pwm_failsafe >= params->pwm_max) {
		PX4_ERR("Invalid parameter MODAL_PWM_FS.  Please verify parameters.");
		params->pwm_failsafe = 0;
		ret = PX4_ERROR;
	}

	return ret;
}

bool ModalPWM::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	// PX4_INFO("Entering updateOuputs in ModalPWM!");
	//in Run() we call _mixing_output.update(), which calls MixingOutput::limitAndUpdateOutputs which calls _interface.updateOutputs (this function)
	//So, if Run() is blocked by a custom command, this function will not be called until Run is running again
	int16_t _rate_req[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	int16_t _led_req[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	uint8_t _fb_idx = 0;

	if (num_outputs != MODAL_PWM_OUTPUT_CHANNELS) {
		PX4_ERR("Num outputs != MODAL_PWM_OUTPUT_CHANNELS!");
		return false;
	}

	for (int i = 0; i < MODAL_PWM_OUTPUT_CHANNELS; i++) {
		if (outputs[i]){
			_pwm_on = true;
		}
		if (!_pwm_on || stop_motors) {
			_rate_req[i] = 0;
			_pwm_values[i] = _rate_req[i];
		} else {
			_rate_req[i] = outputs[i];
			_pwm_values[i] = _rate_req[i];
		}
	}

	Command cmd;
	cmd.len = qc_esc_create_pwm_packet4_fb(_rate_req[0],
					       _rate_req[1],
					       _rate_req[2],
					       _rate_req[3],
					       _led_req[0],
					       _led_req[1],
					       _led_req[2],
					       _led_req[3],
					       _fb_idx,
					       cmd.buf,
					       sizeof(cmd.buf));

	// if (_pwm_on){
		// PX4_INFO("\tPWM CH1: %hu", _rate_req[0]);
		// PX4_INFO("\tPWM CH2: %hu", _rate_req[1]);
		// PX4_INFO("\tPWM CH3: %hu", _rate_req[2]);
		// PX4_INFO("\tPWM CH4: %hu", _rate_req[3]);
		// PX4_INFO("");
	// }

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
		return false;
	} 

	perf_count(_interval_perf);

	return true;
}

int ModalPWM::flush_uart_rx()
{
	while (_uart_port->uart_read(_read_buf, sizeof(_read_buf)) > 0) {}

	return 0;
}

int ModalPWM::read_response(Command *out_cmd)
{
	px4_usleep(_current_cmd.resp_delay_us);

	int res = _uart_port->uart_read(_read_buf, sizeof(_read_buf));

	if (res > 0) {
		//PX4_INFO("read %i bytes",res);
		// if (parse_response(_read_buf, res, out_cmd->print_feedback) < 0) {
			//PX4_ERR("Error parsing response");
			return -1;
		// }

	} else {
		//PX4_ERR("Read error: %i", res);
		return -1;
	}

	//_current_cmd.response = false;

	return 0;
}


void ModalPWM::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	if (_first_update_cycle) PX4_INFO("Begin Modal_PWM on M0065 device");
	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	/* Open serial port in this thread */
	if (!_uart_port->is_open()) {
		if (_uart_port->uart_open(_device, _parameters.baud_rate) == PX4_OK) {
			PX4_INFO("Opened UART M0065 device");

		} else {
			PX4_ERR("Failed opening device");
			return;
		}
	}

	_mixing_output.update(); //calls MixingOutput::limitAndUpdateOutputs which calls updateOutputs in this module

	/* update PWM status if armed or if disarmed PWM values are set */
	_pwm_on = _mixing_output.armed().armed;

	/* check for parameter updates */
	if (!_pwm_on && _parameter_update_sub.updated()) {
		/* clear update */
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		/* update parameters from storage */
		update_params();
	}

	/* Don't process commands if pwm on */
	if (!_pwm_on) {
		if (_current_cmd.valid()) {
			PX4_INFO("sending %d commands with delay %dus",_current_cmd.repeats,_current_cmd.repeat_delay_us);
			flush_uart_rx();

			do {
				PX4_INFO("CMDs left %d",_current_cmd.repeats);
				if (_uart_port->uart_write(_current_cmd.buf, _current_cmd.len) == _current_cmd.len) {
					if (_current_cmd.repeats == 0) {
						_current_cmd.clear();
					}

					/* Not reading response right now */
					// if (_current_cmd.response) {
					// 	if (read_response(&_current_cmd) == 0) {
					// 		int i = 0;
					// 		// _esc_status_pub.publish(_esc_status);
					// 	}
					// }

				} else {
					if (_current_cmd.retries == 0) {
						_current_cmd.clear();
						PX4_ERR("Failed to send command, errno: %i", errno);

					} else {
						_current_cmd.retries--;
						PX4_ERR("Failed to send command, errno: %i", errno);
					}
				}

				px4_usleep(_current_cmd.repeat_delay_us);
			} while (_current_cmd.repeats-- > 0);

			// PX4_INFO("RX packet count: %d", (int)_rx_packet_count);
			// PX4_INFO("CRC error count: %d", (int)_rx_crc_error_count);

		} else {
			Command *new_cmd = _pending_cmd.load();

			if (new_cmd) {
				_current_cmd = *new_cmd;
				_pending_cmd.store(nullptr);
			}
		}
	}

	/* check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread) */
	_mixing_output.updateSubscriptions(true);

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


bool ModalPWM::stop_all_pwms()
{
	int16_t _rate_req[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	int16_t _led_req[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	uint8_t _fb_idx = 0;

	Command cmd;
	cmd.len = qc_esc_create_pwm_packet4_fb(_rate_req[0],
					       _rate_req[1],
					       _rate_req[2],
					       _rate_req[3],
					       _led_req[0],
					       _led_req[1],
					       _led_req[2],
					       _led_req[3],
					       _fb_idx,
					       cmd.buf,
					       sizeof(cmd.buf));

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
		return false;
	}

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
	PX4_INFO("Executing the following command: %s", verb);

	/* start the FMU if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return ModalPWM::task_spawn(argc, argv);
		}
	}

	if (!strcmp(verb, "status")) {
		if (!is_running()){
			PX4_INFO("Not running");
			return -1;
		}
		return get_instance()->print_status();
	}

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;

	}

	while ((ch = px4_getopt(argc, argv, "c:n:t:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'c':
			output_channel = atoi(myoptarg);
			if (output_channel > MODAL_PWM_OUTPUT_CHANNELS - 1){
				char reason[50];
				sprintf(reason, "Bad channel value: %d. Must be 0-%d.", output_channel, MODAL_PWM_OUTPUT_CHANNELS-1);
				print_usage(reason);
				return 0;
			}
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
		PX4_INFO("Output channel: %i", output_channel);
		PX4_INFO("Repeat count: %i", repeat_count);
		PX4_INFO("Repeat delay (us): %i", repeat_delay_us);
		PX4_INFO("Rate: %i", rate);
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

			cmd.response        = false;
			cmd.repeats         = repeat_count;
			cmd.resp_delay_us   = 0;
			cmd.repeat_delay_us = repeat_delay_us;
			cmd.print_feedback  = false;

			PX4_INFO("feedback id debug: %i", id_fb);
			PX4_INFO("Sending UART M0065 power command %i", rate);

			if (get_instance()->_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
				PX4_ERR("Failed to send packet");
				return -1;
			} 
		} else {
			print_usage("Invalid Output Channel, use 0-3");
			return 0;
		}
	}

	return print_usage("unknown custom command");
}

int ModalPWM::print_status()
{
	PX4_INFO("Max update rate: %i Hz", _current_update_rate);
	PX4_INFO("Outputs on: %s", _pwm_on ? "yes" : "no");
	PX4_INFO("UART port: %s", _device);
	PX4_INFO("UART open: %s", _uart_port->is_open() ? "yes" : "no");

	PX4_INFO("");

	PX4_INFO("Params: MODAL_PWM_CONFIG: %" PRId32, _parameters.config);
	PX4_INFO("Params: MODAL_PWM_BAUD: %" PRId32, _parameters.baud_rate);

	PX4_INFO("Params: MODAL_PWM_FUNC1: %" PRId32, _parameters.function_map[0]);
	PX4_INFO("Params: MODAL_PWM_FUNC2: %" PRId32, _parameters.function_map[1]);
	PX4_INFO("Params: MODAL_PWM_FUNC3: %" PRId32, _parameters.function_map[2]);
	PX4_INFO("Params: MODAL_PWM_FUNC4: %" PRId32, _parameters.function_map[3]);

	PX4_INFO("Params: MODAL_PWM_MIN: %" PRId32, _parameters.pwm_min);
	PX4_INFO("Params: MODAL_PWM_MAX: %" PRId32, _parameters.pwm_max);
	PX4_INFO("Params: MODAL_PWM_FS : %" PRId32, _parameters.pwm_failsafe);

	PX4_INFO("PWM CH1: %" PRId16, _pwm_values[0]);
	PX4_INFO("PWM CH2: %" PRId16, _pwm_values[1]);
	PX4_INFO("PWM CH3: %" PRId16, _pwm_values[2]);
	PX4_INFO("PWM CH4: %" PRId16, _pwm_values[3]);

	PX4_INFO("");

	_mixing_output.printStatus();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

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
