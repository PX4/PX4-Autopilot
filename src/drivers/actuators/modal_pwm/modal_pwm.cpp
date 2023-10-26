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
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
	_mixing_output.setMaxNumOutputs(MODAL_PWM_OUTPUT_CHANNELS);
	_uart_port = new ModalIoSerial();
	qc_esc_packet_init(&_sbus_packet);
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
	perf_free(_output_update_perf);
}


int ModalPWM::init()
{

	/* Getting initial parameter values */
	int ret = update_params();

	if (ret != OK) {
		return ret;
	}

	// ScheduleOnInterval(_current_update_interval); 
	ScheduleNow();
	return ret;
}

int ModalPWM::update_params()
{
	int ret = PX4_ERROR;

	updateParams();
	ret = load_params(&_parameters);

	if (ret == PX4_OK) {
		_mixing_output.setAllDisarmedValues(MODAL_PWM_MIXER_DISARMED);
		_mixing_output.setAllFailsafeValues(MODAL_PWM_MIXER_FAILSAFE);
		_mixing_output.setAllMinValues(MODAL_PWM_MIXER_MIN);
		_mixing_output.setAllMaxValues(MODAL_PWM_MIXER_MAX);
		_pwm_fullscale = _parameters.pwm_max - _parameters.pwm_min;
	}

	return ret;
}
	
int ModalPWM::load_params(modal_pwm_params_t *params)
{
	int ret = PX4_OK;
	int32_t max = params->pwm_max;
	int32_t min = params->pwm_min;

	// initialize out
	for (int i = 0; i < MODAL_PWM_OUTPUT_CHANNELS; i++) {
		params->function_map[i] = (int)OutputFunction::Disabled;
	}
	
	/* UART config, PWM mode, and RC protocol*/
	param_get(param_find("MODAL_PWM_CONFIG"),  &params->config);
	param_get(param_find("MODAL_PWM_BAUD"),    &params->baud_rate);
	param_get(param_find("RC_INPUT_PROTO"),    &params->param_rc_input_proto);

	/* PWM min, max, and failsafe values*/
	param_get(param_find("MODAL_PWM_MIN"),  &params->pwm_min);
	param_get(param_find("MODAL_PWM_MAX"),  &params->pwm_max);

	/* PWM output functions */
	param_get(param_find("MODAL_PWM_FUNC1"),  &params->function_map[0]);
	param_get(param_find("MODAL_PWM_FUNC2"),  &params->function_map[1]);
	param_get(param_find("MODAL_PWM_FUNC3"),  &params->function_map[2]);
	param_get(param_find("MODAL_PWM_FUNC4"),  &params->function_map[3]);
	
	/* PWM rate */
	if( params->config == 1){
		_current_update_rate = 400;
	}

	/* Validate PWM min and max values */
	if (params->pwm_min > params->pwm_max){
		PX4_ERR("Invalid parameter MODAL_PWM_MIN.  Please verify parameters.");
		params->pwm_min = 0;
		ret = PX4_ERROR;
	}

	if(ret == PX4_OK && _uart_port->is_open() && (max != params->pwm_max || min != params->pwm_min)){
		PX4_INFO("Updating PWM params load_params");
		update_pwm_config();
	}

	return ret;
}

void ModalPWM::update_pwm_config()
{
	Command cmd;
	uint8_t data[MODAL_PWM_CONFIG_SIZE] = {static_cast<uint8_t>((_parameters.pwm_min & 0xFF00)>>8), static_cast<uint8_t>(_parameters.pwm_min & 0xFF),
										   static_cast<uint8_t>((_parameters.pwm_max & 0xFF00)>>8), static_cast<uint8_t>(_parameters.pwm_max & 0xFF)};
	cmd.len = qc_esc_create_packet(ESC_PACKET_TYPE_CONFIG_BOARD_REQUEST,
						data,
						MODAL_PWM_CONFIG_SIZE,	// 4 bytes, pwm min and pwm max 
						cmd.buf,
						sizeof(cmd.buf));

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
	} else {
		_bytes_sent+=cmd.len;
		_packets_sent++;
	}
}

bool ModalPWM::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	//in Run() we call _mixing_output.update(), which calls MixingOutput::limitAndUpdateOutputs which calls _interface.updateOutputs (this function)
	//So, if Run() is blocked by a custom command, this function will not be called until Run is running again
	int16_t _rate_req[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	uint8_t _led_req[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	int32_t _fb_idx = -1;

	if (num_outputs != MODAL_PWM_OUTPUT_CHANNELS) {
		PX4_ERR("Num outputs != MODAL_PWM_OUTPUT_CHANNELS!");
		return false;
	}

	for (int i = 0; i < MODAL_PWM_OUTPUT_CHANNELS; i++) {
		if (!_mixing_output.isFunctionSet(i)) {
			// do not run any signal on disabled channels
			outputs[i] = 0;
		}

		if (outputs[i]){
			_pwm_on = true;
		}
		if (!_pwm_on || stop_motors) {
			_rate_req[i] = 0;
		} else {
			_rate_req[i] = outputs[i];
		}
		_pwm_values[i] = _rate_req[i];
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
	/*
	if (_pwm_on){
	// Debug messages for PWM 400Hz values sent to M0065  
		uint16_t tics_1 = (_parameters.pwm_min +  (_pwm_fullscale * ((double)outputs[0]/MODAL_PWM_MIXER_MAX))) * 24;
		PX4_INFO("\tPWM CH1: %hu::%uus::%u tics", outputs[0], tics_1/24, tics_1);
		uint16_t tics_2 = (_parameters.pwm_min +  (_pwm_fullscale *((double)outputs[1]/MODAL_PWM_MIXER_MAX))) * 24;
		PX4_INFO("\tPWM CH2: %u::%uus::%u tics", outputs[1], tics_2/24, tics_2);
		uint16_t tics_3 = (_parameters.pwm_min +  (_pwm_fullscale *((double)outputs[2]/MODAL_PWM_MIXER_MAX))) * 24;
		PX4_INFO("\tPWM CH3: %u::%uus::%u tics", outputs[2], tics_3/24, tics_3);
		uint16_t tics_4 = (_parameters.pwm_min +  (_pwm_fullscale *((double)outputs[3]/MODAL_PWM_MIXER_MAX))) * 24;
		PX4_INFO("\tPWM CH4: %u::%uus::%u tics", outputs[3], tics_4/24, tics_4);
		PX4_INFO("");
	}
	*/
	
	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
		return false;
	} else {
		_bytes_sent+=cmd.len;
		_packets_sent++;
	}

	perf_count(_output_update_perf);

	return true;
}

int ModalPWM::flush_uart_rx()
{
	while (_uart_port->uart_read(_read_buf, sizeof(_read_buf)) > 0) {}

	return 0;
}

int ModalPWM::parse_response(uint8_t *buf, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		int16_t ret = qc_esc_packet_process_char(buf[i], &_sbus_packet);

		if (ret > 0) {
			//PX4_INFO("got packet of length %i",ret);
			// _rx_packet_count++;
			uint8_t packet_type = qc_esc_packet_get_type(&_sbus_packet);
			uint8_t packet_size = qc_esc_packet_get_size(&_sbus_packet);

			if (packet_type == ESC_PACKET_TYPE_RC_DATA_RAW && packet_size == QC_SBUS_FRAME_SIZE) 
			{
				return 0;
			}

		} else { //parser error
			switch (ret) {
			case ESC_ERROR_BAD_CHECKSUM:
				if(_pwm_on) PX4_WARN("BAD SBUS packet checksum");
				break;

			case ESC_ERROR_BAD_LENGTH:
				if(_pwm_on) PX4_WARN("BAD SBUS packet length");
				break;
			}
			return ret;
		}
	}

	return 0;
}

int ModalPWM::read_response(Command *out_cmd)
{
	px4_usleep(_current_cmd.resp_delay_us);

	int res = _uart_port->uart_read(_read_buf, sizeof(_read_buf));

	if (res > 0) {
		_bytes_received+=res;
		_packets_received++;
		//PX4_INFO("read %i bytes",res);
		// if (parse_response(_read_buf, res, out_cmd->print_feedback) < 0) {
			//PX4_ERR("Error parsing response");
			// return -1;
		// }

	} else {
		//PX4_ERR("Read error: %i", res);
		return -1;
	}

	//_current_cmd.response = false;

	return 0;
}

void ModalPWM::fill_rc_in(uint16_t raw_rc_count_local,
		    uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
		    hrt_abstime now, bool frame_drop, bool failsafe,
		    unsigned frame_drops, int rssi, input_rc_s &input_rc)
{
	// fill rc_in struct for publishing
	input_rc.channel_count = raw_rc_count_local;

	if (input_rc.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		input_rc.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < input_rc.channel_count; i++) {
		input_rc.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}

		// once filled, reset values back to default
		_raw_rc_values[i] = UINT16_MAX;
	}

	input_rc.timestamp = now;
	input_rc.timestamp_last_signal = input_rc.timestamp;
	input_rc.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {
		input_rc.rssi = 255;
	} else {
		input_rc.rssi = rssi;
	}

	if (valid_chans == 0) {
		input_rc.rssi = 0;
	}

	if (frame_drops){
		_sbus_frame_drops++;
	}

	input_rc.rc_failsafe = failsafe;
	input_rc.rc_lost = input_rc.rc_failsafe;
	input_rc.rc_lost_frame_count = _sbus_frame_drops;
	input_rc.rc_total_frame_count = ++_sbus_total_frames;
}

int ModalPWM::receive_sbus()
{
	int res = 0;
	int read_retries = 3;
	int read_succeeded = 0;
	while (read_retries) {
    	res = _uart_port->uart_read(_read_buf, QC_SBUS_FRAME_SIZE);
		if (res) {
			/* Check if we got a valid packet...*/
			if (parse_response(_read_buf, (uint8_t)QC_SBUS_FRAME_SIZE)){
				if(_pwm_on) PX4_ERR("Error parsing QC SBUS packet");
				read_retries--;
				break;
			}

			input_rc_s input_rc;
			uint16_t num_values;
			bool sbus_failsafe;
			bool sbus_frame_drop;
			uint16_t max_channels = sizeof(_raw_rc_values) / sizeof(_raw_rc_values[0]);
			hrt_abstime now = hrt_absolute_time();
			bool rc_updated = sbus_parse(now, &_read_buf[SBUS_PAYLOAD], SBUS_FRAME_SIZE, _raw_rc_values, &num_values,
						&sbus_failsafe, &sbus_frame_drop, &_sbus_frame_drops, max_channels);
	
			if (rc_updated) {
				/*
				PX4_INFO("decoded packet");
				PX4_INFO("[%0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x %0x]",
					_raw_rc_values[0], _raw_rc_values[1], _raw_rc_values[2], 
					_raw_rc_values[3], _raw_rc_values[4], _raw_rc_values[5], 
					_raw_rc_values[6], _raw_rc_values[7], _raw_rc_values[8],
					_raw_rc_values[9], _raw_rc_values[10], _raw_rc_values[11], 
					_raw_rc_values[12], _raw_rc_values[13], _raw_rc_values[14], 
					_raw_rc_values[15], _raw_rc_values[16], _raw_rc_values[17]
					);
				*/
				input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;
				fill_rc_in(num_values, _raw_rc_values, now, sbus_frame_drop, sbus_failsafe, _sbus_frame_drops, -1, input_rc);
				if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
					_rc_last_valid = input_rc.timestamp;
				}

				input_rc.timestamp_last_signal =_rc_last_valid;
				_rc_pub.publish(input_rc);

				_bytes_received+=res;
				_packets_received++;
				read_succeeded = 1;
				break;
			} else {
				read_retries--;
				if (_pwm_on){
					PX4_ERR("Failed to decode SBUS packet");
					if (sbus_frame_drop) {
						PX4_WARN("SBUS frame dropped");
					}
					// PX4_ERR("[%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x]",
					// 	_read_buf[0], _read_buf[1], _read_buf[2], _read_buf[3], _read_buf[4], _read_buf[5], 
					// 	_read_buf[6], _read_buf[7], _read_buf[8], _read_buf[9], _read_buf[10], _read_buf[11], 
					// 	_read_buf[12], _read_buf[13], _read_buf[14], _read_buf[15], _read_buf[16], _read_buf[17], 
					// 	_read_buf[18], _read_buf[19], _read_buf[20], _read_buf[21], _read_buf[22], _read_buf[23], 
					// 	_read_buf[24], _read_buf[25], _read_buf[26], _read_buf[27], _read_buf[28], _read_buf[29]
					// 	);
				}
			}
		}
		read_retries--;
	}

	if ( ! read_succeeded) {
		return -EIO;
	}

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

	if (_first_update_cycle){
		PX4_INFO("Begin Modal_PWM for M0065 device");
		_first_update_cycle = false;
	}
	perf_begin(_cycle_perf);

	/* Open serial port in this thread */
	if (!_uart_port->is_open()) {
		if (_uart_port->uart_open(_device, _parameters.baud_rate) == PX4_OK) {			
			/* Send PWM config to M0065... pwm_min and pwm_max */
			PX4_INFO("Opened UART connection to M0065 device");
			update_pwm_config();
		} else {
			PX4_ERR("Failed opening device");
			return;
		}
	}

	// Check for SBUS packets
	receive_sbus();

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


				} else {
					_bytes_sent+=_current_cmd.len;
					_packets_sent++;
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
}

int ModalPWM::task_spawn(int argc, char *argv[])
{
	ModalPWM *instance = new ModalPWM();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		
		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}
		
	} else {
		PX4_ERR("alloc failed");
	}

	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
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
	} else {
		_bytes_sent+=cmd.len;
		_packets_sent++;
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
			cmd.resp_delay_us   = 1000;
			cmd.repeat_delay_us = repeat_delay_us;
			cmd.print_feedback  = false;

			PX4_INFO("feedback id debug: %i", id_fb);
			PX4_INFO("Sending UART M0065 power command %i", rate);

			if (get_instance()->_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
				PX4_ERR("Failed to send packet");
				return -1;
			} else {
				get_instance()->_bytes_sent+=cmd.len;
				get_instance()->_packets_sent++;
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
	PX4_INFO("Max update rate: %u Hz", 1000000/_current_update_interval);
	PX4_INFO("PWM Rate: %i Hz", _current_update_rate);
	PX4_INFO("Outputs on: %s", _pwm_on ? "yes" : "no");
	PX4_INFO("RC Type: SBUS");
	PX4_INFO("RC Connected: %s", hrt_absolute_time() - _rc_last_valid > 500000 ? "no" : "yes");
	PX4_INFO("RC Packets Received: %" PRIu16, _sbus_total_frames);
	PX4_INFO("UART port: %s", _device);
	PX4_INFO("UART open: %s", _uart_port->is_open() ? "yes" : "no");
	PX4_INFO("Packets sent: %" PRIu32, _packets_sent);
	PX4_INFO("");
	PX4_INFO("Params: MODAL_PWM_CONFIG: %" PRId32, _parameters.config);
	PX4_INFO("Params: MODAL_PWM_BAUD: %" PRId32, _parameters.baud_rate);
	PX4_INFO("Params: MODAL_PWM_FUNC1: %" PRId32, _parameters.function_map[0]);
	PX4_INFO("Params: MODAL_PWM_FUNC2: %" PRId32, _parameters.function_map[1]);
	PX4_INFO("Params: MODAL_PWM_FUNC3: %" PRId32, _parameters.function_map[2]);
	PX4_INFO("Params: MODAL_PWM_FUNC4: %" PRId32, _parameters.function_map[3]);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_output_update_perf);
	PX4_INFO("");
	_mixing_output.printStatus();
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
