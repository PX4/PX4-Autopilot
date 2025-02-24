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

#include "voxl2_io.hpp"


Voxl2IO::Voxl2IO() :
	OutputModuleInterface(MODULE_NAME, px4::serial_port_to_wq(VOXL2_IO_DEFAULT_PORT)),
	_mixing_output{"VOXL2_IO", VOXL2_IO_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false},
	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": module cycle")},
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval"))
{
	_mixing_output.setMaxNumOutputs(VOXL2_IO_OUTPUT_CHANNELS);
	voxl2_io_packet_init(&_voxl2_io_packet);

	//set low rate scheduling interval to 200hz so that RC can be updated even if all actuators are disabled
	//otherwise, the default low rate scheduling interval is 300ms and RC packets are delayed and lost
	_mixing_output.setLowrateSchedulingInterval(5_ms);
}

Voxl2IO::~Voxl2IO()
{
	_uart_port.close();

	perf_free(_cycle_perf);
	perf_free(_output_update_perf);
}

int Voxl2IO::init()
{
	PX4_INFO("VOXL2_IO: Driver starting");

	int ret = PX4_OK;

	// Getting initial parameter values
	ret = update_params();

	if (ret != OK) {
		PX4_ERR("VOXL2_IO: Failed to update params during init");
		return ret;
	}

	// Print initial param values
	print_params();

	PX4_INFO("VOXL2_IO: ");

	// Open serial port
	if (!_uart_port.isOpen()) {
		PX4_INFO("VOXL2_IO: Opening UART ESC device %s, baud rate %" PRIi32, _device, _parameters.baud_rate);

		// Configure UART port
		if (! _uart_port.setPort(_device)) {
			PX4_ERR("Error configuring serial device on port %s", _device);
			return -1;
		}

		if (! _uart_port.setBaudrate(_parameters.baud_rate)) {
			PX4_ERR("Error setting baudrate to %d on %s", (int) _parameters.baud_rate, _device);
			return -1;
		}

		// Open the UART. If this is successful then the UART is ready to use.
		if (! _uart_port.open()) {
			PX4_ERR("Error opening serial device  %s", _device);
			return -1;
		}
	}

	// Detect M0065 board
	ret = get_version_info();

	if (ret != 0) {
		PX4_ERR("VOXL2_IO: Could not detect the board");
		PX4_ERR("VOXL2_IO: Driver initialization failed. Exiting");

		if (_uart_port.open()) {
			PX4_INFO("VOXL2_IO: Closing uart port");
			_uart_port.close();
		}

		return -1;
	}

	//ScheduleOnInterval(_current_update_interval);
	ScheduleNow();

	PX4_INFO("VOXL2_IO: Driver initialization succeeded");
	return ret;
}

int Voxl2IO::update_params()
{
	int ret = PX4_ERROR;

	updateParams();
	ret = load_params(&_parameters);

	if (ret == PX4_OK) {
		_mixing_output.setAllDisarmedValues(_parameters.pwm_dis);
		_mixing_output.setAllFailsafeValues(VOXL2_IO_MIXER_FAILSAFE);
		_mixing_output.setAllMinValues(_parameters.pwm_min);
		_mixing_output.setAllMaxValues(_parameters.pwm_max);
	}

	return ret;
}

int Voxl2IO::load_params(voxl2_io_params_t *params)
{
	int ret = PX4_OK;

	// initialize out
	for (int i = 0; i < VOXL2_IO_OUTPUT_CHANNELS; i++) {
		params->function_map[i] = (int)OutputFunction::Disabled;
	}

	// UART config, PWM mode, and RC protocol
	param_get(param_find("VOXL2_IO_BAUD"),    &params->baud_rate);
	//param_get(param_find("RC_INPUT_PROTO"),    &params->param_rc_input_proto);

	// PWM min, max, and failsafe values
	param_get(param_find("VOXL2_IO_MIN"),  &params->pwm_min);
	param_get(param_find("VOXL2_IO_MAX"),  &params->pwm_max);
	param_get(param_find("VOXL2_IO_DIS"),  &params->pwm_dis);
	param_get(param_find("VOXL2_IO_CMIN"), &params->pwm_cal_min);
	param_get(param_find("VOXL2_IO_CMAX"), &params->pwm_cal_max);

	// PWM output functions
	//0: disabled, 1: constant min, 2: constant max
	//101-112: motors, 201-208: servos, 402: RC Roll, 403: RC Pitch, 404: RC Throttle,
	//405: RC Yaw, 406: RC Flaps, 407-412: RC AUX 1-6, 420-422: Gimbal RPY
	param_get(param_find("VOXL2_IO_FUNC1"),  &params->function_map[0]);
	param_get(param_find("VOXL2_IO_FUNC2"),  &params->function_map[1]);
	param_get(param_find("VOXL2_IO_FUNC3"),  &params->function_map[2]);
	param_get(param_find("VOXL2_IO_FUNC4"),  &params->function_map[3]);
	param_get(param_find("VOXL2_IO_FUNC5"),  &params->function_map[4]);
	param_get(param_find("VOXL2_IO_FUNC6"),  &params->function_map[5]);
	param_get(param_find("VOXL2_IO_FUNC7"),  &params->function_map[6]);
	param_get(param_find("VOXL2_IO_FUNC8"),  &params->function_map[7]);

	// Validate PWM min and max values
	if (params->pwm_min > params->pwm_max) {
		PX4_ERR("VOXL2_IO: Invalid parameter VOXL2_IO_MIN.  Please verify parameters.");
		params->pwm_min = 0;
		ret = PX4_ERROR;
	}

	return ret;
}

int Voxl2IO::get_version_info()
{
	PX4_INFO("VOXL2_IO: Detecting M0065 board...");
	voxl2_io_packet_init(&_voxl2_io_packet);

	Command cmd;
	cmd.len = voxl2_io_create_extended_version_request_packet(0, cmd.buf, sizeof(cmd.buf));

	int retries_left  = _board_detect_retries;
	bool got_response = false;

	while ((got_response == false) && (retries_left > 0)) {
		retries_left--;

		//send the version request command to the board
		if (_uart_port.write(cmd.buf, cmd.len) != cmd.len) {
			PX4_ERR("VOXL2_IO: Could not write version request packet to UART port");
			return -1;
		}

		_bytes_sent += cmd.len;
		_packets_sent++;

		hrt_abstime t_request = hrt_absolute_time();
		hrt_abstime t_timeout = 50000; //50ms timeout for version info response


		//wait for the response to come back
		while ((!got_response) && (hrt_elapsed_time(&t_request) < t_timeout)) {
			px4_usleep(500); //sleep a bit while waiting for the board to respond

			int nread = _uart_port.read(_read_buf, sizeof(_read_buf));

			for (int i = 0; i < nread; i++) {
				int16_t parse_ret = voxl2_io_packet_process_char(_read_buf[i], &_voxl2_io_packet);

				if (parse_ret > 0) {
					hrt_abstime response_time = hrt_elapsed_time(&t_request);
					//PX4_INFO("got packet of length %i",ret);
					_packets_received++;
					uint8_t packet_type = voxl2_io_packet_get_type(&_voxl2_io_packet);
					uint8_t packet_size = voxl2_io_packet_get_size(&_voxl2_io_packet);

					if (packet_type == VOXL2_IO_PACKET_TYPE_VERSION_EXT_RESPONSE && packet_size == sizeof(VOXL2_IO_EXTENDED_VERSION_INFO)) {
						VOXL2_IO_EXTENDED_VERSION_INFO ver;
						memcpy(&ver, _voxl2_io_packet.buffer, packet_size);

						PX4_INFO("VOXL2_IO: \tVOXL2_IO ID: %i", ver.id);
						PX4_INFO("VOXL2_IO: \tBoard Type : %i: %s", ver.hw_version, board_id_to_name(ver.hw_version).c_str());

						uint8_t *u = &ver.unique_id[0];
						PX4_INFO("VOXL2_IO: \tUnique ID  : 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
							 u[11], u[10], u[9], u[8], u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);

						PX4_INFO("VOXL2_IO: \tFirmware   : version %4d, hash %.12s", ver.sw_version, ver.firmware_git_version);
						PX4_INFO("VOXL2_IO: \tBootloader : version %4d, hash %.12s", ver.bootloader_version, ver.bootloader_git_version);
						PX4_INFO("VOXL2_IO: \tReply time : %uus", (uint32_t)response_time);

						//we requested response from ID 0, so it should match
						if (ver.id != 0) {
							PX4_ERR("VOXL2_IO: Invalid id: %d", ver.id);
						}

						//check HW (board version)
						else if (ver.hw_version != VOXL2_IO_HW_VERSION) {
							PX4_ERR("VOXL2_IO: Invalid HW version : %d (expected %d)", ver.hw_version, VOXL2_IO_HW_VERSION);
							return -1;
						}

						//check firmware version running on the board
						else if (ver.sw_version != VOXL2_IO_SW_VERSION) {
							PX4_ERR("VOXL2_IO: Invalid FW version : %d (expected %d)", ver.sw_version, VOXL2_IO_SW_VERSION);
							return -1;

						} else {
							got_response = true;
							memcpy(&_version_info, &ver, sizeof(_version_info)); //store the version info only if it is valid
						}
					}
				}
			}

			//break out of the loop waiting for a response
			if (got_response) {
				break;
			}
		}


		if (!got_response) {
			PX4_ERR("VOXL2_IO: Board version info response timeout (%d retries left)", retries_left);
		}
	}

	return (got_response == true ? 0 : -1);
}

bool Voxl2IO::updateOutputs(bool stop_motors, uint16_t outputs[input_rc_s::RC_INPUT_MAX_CHANNELS],
			    unsigned num_outputs, unsigned num_control_groups_updated)
{
	// Stop Mixer while ESCs are being calibrated
	if (_outputs_disabled) {
		return 0;
	}

	//PX4_INFO("VOXL2_IO: Mixer output: %u, %u, %u, %u", outputs[0], outputs[1], outputs[2], outputs[3]);

	//in Run() we call _mixing_output.update(), which calls MixingOutput::limitAndUpdateOutputs which calls _interface.updateOutputs (this function)
	//So, if Run() is blocked by a custom command, this function will not be called until Run is running again
	uint32_t output_cmds[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};

	if (num_outputs != VOXL2_IO_OUTPUT_CHANNELS) {
		PX4_ERR("VOXL2_IO: Num outputs != VOXL2_IO_OUTPUT_CHANNELS!");
		return false;
	}

	for (int i = 0; i < VOXL2_IO_OUTPUT_CHANNELS; i++) {
		// do not run any signal on disabled channels
		if (!_mixing_output.isFunctionSet(i)) {
			outputs[i] = 0;
		}

		if (outputs[i]) {
			_pwm_on = true;
		}

		//Do we even need this condition? mixer should handle stopping motors anyway by sending the disable command, right?
		if (0) { //(!_pwm_on || stop_motors) {
			output_cmds[i] = _parameters.pwm_dis * MIXER_OUTPUT_TO_CMD_SCALE; //0; //convert to ns

		} else {
			output_cmds[i] = ((uint32_t)outputs[i]) * MIXER_OUTPUT_TO_CMD_SCALE;  //convert to ns
		}
	}

	Command cmd;
	cmd.len = voxl2_io_create_hires_pwm_packet(output_cmds, VOXL2_IO_OUTPUT_CHANNELS, cmd.buf, sizeof(cmd.buf));

	if (_uart_port.write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("VOXL2_IO: Failed to send packet");
		return false;

	} else {
		_bytes_sent += cmd.len;
		_packets_sent++;
	}

	//if (_pwm_on && _debug){
	if (_debug) {
		PX4_INFO("VOXL2_IO: Mixer outputs: [%u, %u, %u, %u, %u, %u, %u, %u]",
			 outputs[0], outputs[1], outputs[2], outputs[3],
			 outputs[4], outputs[5], outputs[6], outputs[7]);
	}

	perf_count(_output_update_perf);

	return true;
}

static bool valid_port(int port)
{
	if (port == 2 || port == 6 || port == 7) {
		return true;
	}

	return false;
}

/*
int Voxl2IO::parse_response(uint8_t *buf, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		int16_t ret = voxl2_io_packet_process_char(buf[i], &_voxl2_io_packet);

		if (ret > 0) {
			uint8_t packet_type = voxl2_io_packet_get_type(&_voxl2_io_packet);
			uint8_t packet_size = voxl2_io_packet_get_size(&_voxl2_io_packet);

			if (packet_type == VOXL2_IO_PACKET_TYPE_RC_DATA_RAW && packet_size == VOXL2_IO_SBUS_FRAME_SIZE) {
				return 0;
			} else {
				return -1;
			}

		} else { //parser error
			switch (ret) {
			case VOXL2_IO_ERROR_BAD_CHECKSUM:
				if(_pwm_on && _debug) PX4_WARN("VOXL2_IO: BAD packet checksum");
				break;

			case VOXL2_IO_ERROR_BAD_LENGTH:
				if(_pwm_on && _debug) PX4_WARN("VOXL2_IO: BAD packet length");
				break;

			case VOXL2_IO_ERROR_BAD_HEADER:
				if(_pwm_on && _debug) PX4_WARN("VOXL2_IO: BAD packet header");
				break;

			case VOXL2_IO_NO_PACKET:
				// if(_pwm_on) PX4_WARN("VOXL2_IO: NO packet");
				break;

			default:
				if(_pwm_on && _debug) PX4_WARN("VOXL2_IO: Unknown error: %i", ret);
				break;
			}
		}
	}

	return 0;
}
*/

void Voxl2IO::fill_rc_in(uint16_t raw_rc_count_local,
			 uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			 hrt_abstime now, bool frame_drop, bool failsafe,
			 unsigned frame_drops, int rssi, input_rc_s &input_rc)
{
	// fill rc_in struct for publishing
	memset(&input_rc, 0, sizeof(input_rc));  //zero out the struct first

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

	input_rc.timestamp             = now;
	input_rc.timestamp_last_signal = input_rc.timestamp;
	input_rc.rc_ppm_frame_length   = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {
		input_rc.rssi = 255;

	} else {
		input_rc.rssi = rssi;
	}

	if (valid_chans == 0) {
		input_rc.rssi = 0;
	}

	if (frame_drops) {
		_sbus_frame_drops++;
	}

	input_rc.rssi_dbm             = 0.0f;
	input_rc.rc_failsafe          = failsafe;
	input_rc.rc_lost              = input_rc.rc_failsafe;
	input_rc.rc_lost_frame_count  = _sbus_frame_drops;
	input_rc.rc_total_frame_count = ++_sbus_total_frames;
}


int Voxl2IO::parse_sbus_packet(uint8_t *raw_data, uint32_t data_len)
{
	input_rc_s input_rc;
	uint16_t num_values;
	bool sbus_failsafe    = false;
	bool sbus_frame_drop  = false;
	uint16_t max_channels = sizeof(_raw_rc_values) / sizeof(_raw_rc_values[0]);
	hrt_abstime t_now     = hrt_absolute_time();


	bool rc_updated = sbus_parse(t_now, raw_data, data_len, _raw_rc_values, &num_values,
				     &sbus_failsafe, &sbus_frame_drop, &_sbus_frame_drops, max_channels);

	if (rc_updated) {
		//if (_pwm_on && _debug){
		if (_debug) {
			//PX4_INFO("VOXL2_IO: Decoded packet, header pos: %i", header);
			PX4_INFO("VOXL2_IO: [%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]",
				 _raw_rc_values[0], _raw_rc_values[1], _raw_rc_values[2],
				 _raw_rc_values[3], _raw_rc_values[4], _raw_rc_values[5],
				 _raw_rc_values[6], _raw_rc_values[7], _raw_rc_values[8],
				 _raw_rc_values[9], _raw_rc_values[10], _raw_rc_values[11],
				 _raw_rc_values[12], _raw_rc_values[13], _raw_rc_values[14],
				 _raw_rc_values[15], _raw_rc_values[16], _raw_rc_values[17]
				);

			if (sbus_frame_drop) {
				PX4_WARN("VOXL2_IO: SBUS frame dropped");
			}
		}

		input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;
		fill_rc_in(num_values, _raw_rc_values, t_now, sbus_frame_drop, sbus_failsafe, _sbus_frame_drops, -1, input_rc);

		if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
			_rc_last_valid_time = input_rc.timestamp;
		}

		input_rc.timestamp_last_signal = _rc_last_valid_time;
		_rc_pub.publish(input_rc);

		if (_rc_mode == RC_MODE::SCAN) {
			PX4_INFO("VOXL2_IO: Detected VOXL2 IO SBUS RC");
			_rc_mode = RC_MODE::SBUS;
		}
	}

	return 0;
}


int Voxl2IO::receive_uart_packets()
{
	int nread = _uart_port.read(_read_buf, READ_BUF_SIZE);

	if (nread > 0) {
		if (_debug) {
			PX4_INFO("VOXL2_IO: receive_uart_packets read %d bytes", nread);
		}

		_bytes_received += nread;

		for (int i = 0; i < nread; i++) {
			int16_t parse_ret = voxl2_io_packet_process_char(_read_buf[i], &_voxl2_io_packet);

			if (parse_ret > 0) {
				_packets_received++;

				uint8_t packet_type = voxl2_io_packet_get_type(&_voxl2_io_packet);
				uint8_t packet_size = voxl2_io_packet_get_size(&_voxl2_io_packet);

				if (packet_type == VOXL2_IO_PACKET_TYPE_RC_DATA_RAW && packet_size == VOXL2_IO_SBUS_FRAME_SIZE) {

					//parse SBUS packet only if configured to do so
					if ((_rc_mode == RC_MODE::SCAN) || (_rc_mode == RC_MODE::SBUS)) {
						parse_sbus_packet(&_voxl2_io_packet.buffer[SBUS_PAYLOAD], SBUS_FRAME_SIZE);
					}
				}

				//parse other packets (future use)
			}
		}
	}

	return 0;
}


void Voxl2IO::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update(); //calls MixingOutput::limitAndUpdateOutputs which calls updateOutputs in this module

	/* update PWM status if armed or if disarmed PWM values are set */
	_pwm_on = _mixing_output.armed().armed;

	//receive packets from voxl_io board
	receive_uart_packets();

	/* check for parameter updates */
	if (!_pwm_on && _parameter_update_sub.updated()) {
		/* clear update */
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		/* update parameters from storage */
		update_params();
	}

	/* check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread) */
	_mixing_output.updateSubscriptions(true);
	perf_end(_cycle_perf);
}

int Voxl2IO::task_spawn(int argc, char *argv[])
{
	Voxl2IO *instance = new Voxl2IO();

	if (instance) {
		int myoptind = 0;
		int ch;
		const char *myoptarg = nullptr;

		_object.store(instance);
		_task_id = task_id_is_work_queue;
		argv++;

		while ((ch = px4_getopt(argc - 1, argv, "vdep:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'v':
				PX4_INFO("VOXL2_IO: Verbose mode enabled");
				get_instance()->_debug = true;
				break;

			case 'd':
				PX4_INFO("VOXL2_IO: M0065 PWM outputs disabled");
				get_instance()->_outputs_disabled = true;
				break;

			case 'e':
				PX4_INFO("VOXL2_IO: M0065 using external RC");
				get_instance()->_rc_mode = RC_MODE::EXTERNAL;
				break;

			case 'p':
				if (valid_port(atoi(myoptarg))) {
					snprintf(get_instance()->_device, 2, "%s", myoptarg);

				} else {
					PX4_ERR("VOXL2_IO: Bad UART port number: %s (must be 2, 6, or 7).", myoptarg);
					_object.store(nullptr);
					_task_id = -1;
					return PX4_ERROR;
				}

				break;

			default:
				print_usage("VOXL2_IO: Unknown command, parsing flags");
				break;
			}
		}

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("VOXL2_IO: alloc failed");
	}

	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int Voxl2IO::calibrate_escs()
{

	// Disable outputs so Mixer isn't being a PITA while we calibrate
	_outputs_disabled = true;

	Command cmd;

	PX4_INFO("VOXL2_IO: PWM ESC calibration is starting!");

	// Give user 10 seconds to plug in PWM cable for ESCs
	PX4_INFO("VOXL2_IO: MIN PWM used for ESC Calibration : %" PRId32, _parameters.pwm_cal_min);
	PX4_INFO("VOXL2_IO: MAX PWM used for ESC Calibration : %" PRId32, _parameters.pwm_cal_max);
	PX4_INFO("VOXL2_IO:");
	PX4_INFO("VOXL2_IO: Connect your ESCs! (Calibration will start in ~10 seconds)");

	px4_usleep(10000000);

	uint32_t max_pwm_cmds[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint32_t min_pwm_cmds[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};

	for (int idx = 0; idx < VOXL2_IO_OUTPUT_CHANNELS; idx++) {
		//only send out calibration pulse if the actuator is a motor
		if ((_parameters.function_map[idx] >= 101) && (_parameters.function_map[idx] <= 112)) {
			max_pwm_cmds[idx] = _parameters.pwm_cal_max * MIXER_OUTPUT_TO_CMD_SCALE;
			min_pwm_cmds[idx] = _parameters.pwm_cal_min * MIXER_OUTPUT_TO_CMD_SCALE;
		}
	}

	if (_debug) {
		PX4_INFO("VOXL2_IO: Scaled max pwms: %u %u %u %u %u %u %u %u",
			 max_pwm_cmds[0], max_pwm_cmds[1], max_pwm_cmds[2], max_pwm_cmds[3],
			 max_pwm_cmds[4], max_pwm_cmds[5], max_pwm_cmds[6], max_pwm_cmds[7]);
	}

	hrt_abstime start;

	// Send PWM max every 2.5ms for 5 seconds
	PX4_INFO("VOXL2_IO: Sending PWM MAX (%dus) for 5 seconds!", _parameters.pwm_cal_max);
	cmd.len = voxl2_io_create_hires_pwm_packet(max_pwm_cmds, VOXL2_IO_OUTPUT_CHANNELS, cmd.buf, sizeof(cmd.buf));
	start = hrt_absolute_time();

	while (hrt_elapsed_time(&start) < 5000000) {
		if (_uart_port.write(cmd.buf, cmd.len) != cmd.len) {
			PX4_ERR("VOXL2_IO: ESC Calibration failed: Failed to send PWM MAX packet");
			_outputs_disabled = false;
			return -1;
		}

		px4_usleep(2500);
	}

	// Send PWM min every 2.5ms for 5 seconds
	PX4_INFO("VOXL2_IO: Sending PWM MIN (%dus) for 5 seconds!", _parameters.pwm_cal_min);

	if (_debug) {
		PX4_INFO("VOXL2_IO: Scaled min pwms: %u %u %u %u %u %u %u %u",
			 min_pwm_cmds[0], min_pwm_cmds[1], min_pwm_cmds[2], min_pwm_cmds[3],
			 min_pwm_cmds[4], min_pwm_cmds[5], min_pwm_cmds[6], min_pwm_cmds[7]);
	}

	cmd.len = voxl2_io_create_hires_pwm_packet(min_pwm_cmds, VOXL2_IO_OUTPUT_CHANNELS, cmd.buf, sizeof(cmd.buf));


	start = hrt_absolute_time();

	while (hrt_elapsed_time(&start) < 5000000) {
		if (_uart_port.write(cmd.buf, cmd.len) != cmd.len) {
			PX4_ERR("VOXL2_IO: ESC Calibration failed: Failed to send PWM MIN packet");
			_outputs_disabled = false;
			return -1;
		}

		px4_usleep(2500);
	}

	PX4_INFO("VOXL2_IO: Waiting 5 seconds to finish the calibration (no PWM output)");
	px4_usleep(5000000);

	PX4_INFO("VOXL2_IO: ESC Calibration complete");
	_outputs_disabled = false;
	return 0;
}

int Voxl2IO::custom_command(int argc, char *argv[])
{
	const char *verb = argv[argc - 1];

	if (argc < 1) {
		return print_usage("unknown command: missing args");
	}

	PX4_INFO("VOXL2_IO: Executing the following command: %s", verb);

	/* start if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return Voxl2IO::task_spawn(argc, argv);
		}

		PX4_INFO("VOXL2_IO: Already running");
		return 0;
	}

	if (!is_running()) {
		PX4_INFO("VOXL2_IO: Not running");
		return -1;
	}

	if (!strcmp(verb, "status")) {
		return get_instance()->print_status();
	}


	if (!strcmp(verb, "calibrate_escs")) {
		if (get_instance()->_outputs_disabled) {
			PX4_WARN("VOXL2_IO: Can't calibrate ESCs while outputs are disabled.");
			return -1;
		}

		return get_instance()->calibrate_escs();
	}

	if (!strcmp(verb, "enable_debug")) {
		get_instance()->_debug = true;
	}

	return print_usage("unknown custom command");
}

int Voxl2IO::print_status()
{
	//PX4_INFO("VOXL2_IO: Max update rate: %u Hz", 1000000/_current_update_interval);
	//PX4_INFO("VOXL2_IO: PWM Rate: 400 Hz");	// Only support 400 Hz for now
	PX4_INFO("VOXL2_IO: Outputs on     : %s", _pwm_on ? "yes" : "no");
	PX4_INFO("VOXL2_IO: SW version     : %u", _version_info.sw_version);
	PX4_INFO("VOXL2_IO: HW version     : %u: %s", _version_info.hw_version,
		 board_id_to_name(_version_info.hw_version).c_str());
	PX4_INFO("VOXL2_IO: RC Type        : SBUS");		// Only support SBUS through M0065 for now
	PX4_INFO("VOXL2_IO: RC Connected   : %s",  hrt_absolute_time() - _rc_last_valid_time > 500000 ? "no" : "yes");
	PX4_INFO("VOXL2_IO: RC Packets Rxd : %"    PRIu16, _sbus_total_frames);
	PX4_INFO("VOXL2_IO: UART port      : %s", _device);
	PX4_INFO("VOXL2_IO: UART open      : %s", _uart_port.open() ? "yes" : "no");
	PX4_INFO("VOXL2_IO: Packets sent   : %"    PRIu32, _packets_sent);
	PX4_INFO("VOXL2_IO: ");
	print_params();

	perf_print_counter(_cycle_perf);
	perf_print_counter(_output_update_perf);
	PX4_INFO("");
	_mixing_output.printStatus();
	return 0;
}

int Voxl2IO::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("voxl2_io", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "Verbose messages", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('d', "Disable PWM", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Disable RC", false);
	PRINT_MODULE_USAGE_PARAM_INT('p', 2, 2, 7, "UART port", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate_escs", "Calibrate ESCs min/max range");
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable_debug", "Enables driver debugging");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

std::string Voxl2IO::board_id_to_name(int board_id)
{
	switch(board_id){
		case 31: return std::string("ModalAi 4-in-1 ESC V2 RevB (M0049)");
		case 32: return std::string("Blheli32 4-in-1 ESC Type A (Tmotor F55A PRO F051)");
		case 33: return std::string("Blheli32 4-in-1 ESC Type B (Tmotor F55A PRO G071)");
		case 34: return std::string("ModalAi 4-in-1 ESC (M0117-1)");
		case 35: return std::string("ModalAi I/O Expander (M0065)");
		case 36: return std::string("ModalAi 4-in-1 ESC (M0117-3)");
		case 37: return std::string("ModalAi 4-in-1 ESC (M0134-1)");
		case 38: return std::string("ModalAi 4-in-1 ESC (M0134-3)");
		case 39: return std::string("ModalAi 4-in-1 ESC (M0129-1)");
		case 40: return std::string("ModalAi 4-in-1 ESC (M0129-3)");
		case 41: return std::string("ModalAi 4-in-1 ESC (M0134-6)");
		case 42: return std::string("ModalAi 4-in-1 ESC (M0138-1)");
		default: return std::string("Unknown Board");
	}
}

void Voxl2IO::print_params()
{
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_BAUD  : %" PRId32, _parameters.baud_rate);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC1 : %" PRId32, _parameters.function_map[0]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC2 : %" PRId32, _parameters.function_map[1]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC3 : %" PRId32, _parameters.function_map[2]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC4 : %" PRId32, _parameters.function_map[3]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC5 : %" PRId32, _parameters.function_map[4]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC6 : %" PRId32, _parameters.function_map[5]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC7 : %" PRId32, _parameters.function_map[6]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_FUNC8 : %" PRId32, _parameters.function_map[7]);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_DIS   : %" PRId32, _parameters.pwm_dis);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_MIN   : %" PRId32, _parameters.pwm_min);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_MAX   : %" PRId32, _parameters.pwm_max);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_CMIN  : %" PRId32, _parameters.pwm_cal_min);
	PX4_INFO("VOXL2_IO: Params: VOXL2_IO_CMAX  : %" PRId32, _parameters.pwm_cal_max);
}

extern "C" __EXPORT int voxl2_io_main(int argc, char *argv[]);

int voxl2_io_main(int argc, char *argv[])
{
	return Voxl2IO::main(argc, argv);
}
