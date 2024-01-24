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

#include <px4_platform_common/sem.hpp>


Voxl2IO::Voxl2IO() :
	OutputModuleInterface(MODULE_NAME, px4::serial_port_to_wq(VOXL2_IO_DEFAULT_PORT)),
	_mixing_output{"VOXL2_IO", VOXL2_IO_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false},
	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": module cycle")},
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval"))
{
	_mixing_output.setMaxNumOutputs(VOXL2_IO_OUTPUT_CHANNELS);
	_uart_port = new Voxl2IoSerial();
	voxl2_io_packet_init(&_sbus_packet);
}

Voxl2IO::~Voxl2IO()
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


int Voxl2IO::init()
{
	int ret = PX4_OK;

	/* Open serial port in this thread */
	if (!_uart_port->is_open()) {
		if (_uart_port->uart_open((const char *)_device, _parameters.baud_rate) == PX4_OK) {
			/* Send PWM config to M0065... pwm_min and pwm_max */
			PX4_INFO("Opened UART connection to M0065 device on port %s", _device);

		} else {
			PX4_ERR("Failed opening device");
			return PX4_ERROR;
		}
	}

	/* Verify connectivity and protocol version number */
	if (get_version_info() < 0) {
		PX4_ERR("Failed to detect voxl2_io protocol version.");
		return PX4_ERROR;

	} else {
		if (_version_info.sw_version == VOXL2_IO_SW_PROTOCOL_VERSION
		    && _version_info.hw_version == VOXL2_IO_HW_PROTOCOL_VERSION) {
			PX4_INFO("Detected M0065 protocol version. SW: %u HW: %u", _version_info.sw_version, _version_info.hw_version);

		} else {
			PX4_ERR("Detected incorrect M0065 protocol version. SW: %u HW: %u", _version_info.sw_version, _version_info.hw_version);
			return PX4_ERROR;
		}
	}

	/* Getting initial parameter values */
	ret = update_params();

	if (ret != OK) {
		return ret;
	}

	/* Send PWM MIN/MAX to M0065 */
	update_pwm_config();

	ScheduleOnInterval(_current_update_interval);
	// ScheduleNow();
	return ret;
}

int Voxl2IO::update_params()
{
	int ret = PX4_ERROR;

	updateParams();
	ret = load_params(&_parameters);

	if (ret == PX4_OK) {
		_mixing_output.setAllDisarmedValues(VOXL2_IO_MIXER_DISARMED);
		_mixing_output.setAllFailsafeValues(VOXL2_IO_MIXER_FAILSAFE);
		_mixing_output.setAllMinValues(VOXL2_IO_MIXER_MIN);
		_mixing_output.setAllMaxValues(VOXL2_IO_MIXER_MAX);
		_pwm_fullscale = _parameters.pwm_max - _parameters.pwm_min;
	}

	return ret;
}

int Voxl2IO::load_params(voxl2_io_params_t *params)
{
	int ret = PX4_OK;
	int32_t max = params->pwm_max;
	int32_t min = params->pwm_min;

	// initialize out
	for (int i = 0; i < VOXL2_IO_OUTPUT_CHANNELS; i++) {
		params->function_map[i] = (int)OutputFunction::Disabled;
	}

	/* UART config, PWM mode, and RC protocol*/
	param_get(param_find("VOXL2_IO_BAUD"),    &params->baud_rate);
	param_get(param_find("RC_INPUT_PROTO"),    &params->param_rc_input_proto);

	/* PWM min, max, and failsafe values*/
	param_get(param_find("VOXL2_IO_MIN"),  &params->pwm_min);
	param_get(param_find("VOXL2_IO_MAX"),  &params->pwm_max);

	/* PWM output functions */
	param_get(param_find("VOXL2_IO_FUNC1"),  &params->function_map[0]);
	param_get(param_find("VOXL2_IO_FUNC2"),  &params->function_map[1]);
	param_get(param_find("VOXL2_IO_FUNC3"),  &params->function_map[2]);
	param_get(param_find("VOXL2_IO_FUNC4"),  &params->function_map[3]);

	/* Validate PWM min and max values */
	if (params->pwm_min > params->pwm_max) {
		PX4_ERR("Invalid parameter VOXL2_IO_MIN.  Please verify parameters.");
		params->pwm_min = 0;
		ret = PX4_ERROR;
	}

	if (ret == PX4_OK && _uart_port->is_open() && (max != params->pwm_max || min != params->pwm_min)) {
		PX4_INFO("Updating PWM params load_params");
		update_pwm_config();
	}

	return ret;
}

void Voxl2IO::update_pwm_config()
{
	Command cmd;
	uint8_t data[VOXL2_IO_BOARD_CONFIG_SIZE] = {static_cast<uint8_t>((_parameters.pwm_min & 0xFF00) >> 8), static_cast<uint8_t>(_parameters.pwm_min & 0xFF),
						    static_cast<uint8_t>((_parameters.pwm_max & 0xFF00) >> 8), static_cast<uint8_t>(_parameters.pwm_max & 0xFF)
						   };
	cmd.len = voxl2_io_create_packet(VOXL2_IO_PACKET_TYPE_CONFIG_BOARD_REQUEST, data, VOXL2_IO_BOARD_CONFIG_SIZE,	cmd.buf,
					 sizeof(cmd.buf));

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send config packet");

	} else {
		_bytes_sent += cmd.len;
		_packets_sent++;
	}
}

int Voxl2IO::get_version_info()
{
	int res = 0 ;
	int header = -1 ;
	int info_packet = -1;
	int read_retries = 3;
	int read_succeeded = 0;
	Command cmd;

	/* Request protocol version info from M0065 */
	cmd.len = voxl2_io_create_version_request_packet(0, cmd.buf, VOXL2_IO_VERSION_INFO_SIZE);

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send version info packet");

	} else {
		_bytes_sent += cmd.len;
		_packets_sent++;
	}

	/* Read response */
	px4_usleep(10000);
	memset(&_read_buf, 0x00, READ_BUF_SIZE);
	res = _uart_port->uart_read(_read_buf, READ_BUF_SIZE);

	while (read_retries) {
		if (res) {
			/* Get index of packer header */
			for (int index = 0; index < READ_BUF_SIZE; ++index) {
				if (_read_buf[index] == VOXL2_IO_PACKET_TYPE_VERSION_RESPONSE) {
					info_packet = index;
					break;
				}

				if (_read_buf[index] == VOXL2_IO_PACKET_HEADER) {
					header = index;
				}
			}

			/* Try again in a bit if packet header not present yet... */
			if (header == -1 || info_packet == -1) {
				if (_debug && header == -1) { PX4_ERR("Failed to find voxl2_io packet header, trying again... retries left: %i", read_retries); }

				if (_debug && info_packet == -1) { PX4_ERR("Failed to find version info packet header, trying again... retries left: %i", read_retries); }

				read_retries--;
				flush_uart_rx();

				if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
					PX4_ERR("Failed to send version info packet");

				} else {
					_bytes_sent += cmd.len;
					_packets_sent++;
					px4_usleep(2000);
				}

				continue;
			}

			/* Check if we got a valid packet...*/
			if (parse_response(&_read_buf[header], (uint8_t)VOXL2_IO_VERSION_INFO_SIZE)) {
				if (_debug) {
					PX4_ERR("Error parsing version info packet");
					PX4_INFO_RAW("[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x]\n",
						     _read_buf[header + 0], _read_buf[header + 1], _read_buf[header + 2], _read_buf[header + 3], _read_buf[header + 4],
						     _read_buf[header + 5],
						     _read_buf[header + 6], _read_buf[header + 7], _read_buf[header + 8], _read_buf[header + 9], _read_buf[header + 10],
						     _read_buf[header + 11],
						     _read_buf[header + 12], _read_buf[header + 13], _read_buf[header + 14], _read_buf[header + 15], _read_buf[header + 16],
						     _read_buf[header + 17],
						     _read_buf[header + 18], _read_buf[header + 19], _read_buf[header + 20], _read_buf[header + 21], _read_buf[header + 22],
						     _read_buf[header + 23],
						     _read_buf[header + 24], _read_buf[header + 25], _read_buf[header + 26], _read_buf[header + 27], _read_buf[header + 28],
						     _read_buf[header + 29]
						    );
				}

				read_retries--;
				flush_uart_rx();

				if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
					PX4_ERR("Failed to send version info packet");

				} else {
					_bytes_sent += cmd.len;
					_packets_sent++;
					px4_usleep(2000);
				}

				break;

			}  else {
				memcpy(&_version_info, &_read_buf[header], sizeof(VOXL2_IO_VERSION_INFO));
				read_succeeded = 1;
				break;
			}

		} else {
			read_retries--;

			if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
				PX4_ERR("Failed to send version info packet");

			} else {
				_bytes_sent += cmd.len;
				_packets_sent++;
				px4_usleep(2000);
			}
		}
	}

	if (! read_succeeded) {
		return -EIO;
	}

	return 0;
}

bool Voxl2IO::updateOutputs(bool stop_motors, uint16_t outputs[input_rc_s::RC_INPUT_MAX_CHANNELS],
			    unsigned num_outputs, unsigned num_control_groups_updated)
{
	/* Stop Mixer while ESCs are being calibrated */
	if (_outputs_disabled) {
		return 0;
	}

	//in Run() we call _mixing_output.update(), which calls MixingOutput::limitAndUpdateOutputs which calls _interface.updateOutputs (this function)
	//So, if Run() is blocked by a custom command, this function will not be called until Run is running again
	int16_t _rate_req[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	uint8_t _led_req[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	int32_t _fb_idx = -1;

	if (num_outputs != VOXL2_IO_OUTPUT_CHANNELS) {
		PX4_ERR("Num outputs != VOXL2_IO_OUTPUT_CHANNELS!");
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

		if (!_pwm_on || stop_motors) {
			_rate_req[i] = 0;

		} else {
			_rate_req[i] = outputs[i];
		}

		_pwm_values[i] = _rate_req[i];
	}

	Command cmd;
	cmd.len = voxl2_io_create_pwm_packet4_fb(_rate_req[0], _rate_req[1], _rate_req[2], _rate_req[3],
			_led_req[0], _led_req[1], _led_req[2], _led_req[3],
			_fb_idx, cmd.buf, sizeof(cmd.buf));

	if (_pwm_on && _debug) {
		PX4_INFO("Mixer outputs");
		PX4_INFO("[%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]",
			 outputs[0], outputs[1], outputs[2], outputs[3], outputs[4], outputs[5],
			 outputs[6], outputs[7], outputs[8], outputs[9], outputs[10], outputs[11],
			 outputs[12], outputs[13], outputs[14], outputs[15], outputs[16], outputs[17]
			);

		// Debug messages for PWM 400Hz values sent to M0065
		uint16_t tics_1 = (_parameters.pwm_min + (_pwm_fullscale * ((double)outputs[0] / VOXL2_IO_MIXER_MAX))) * VOXL2_IO_TICS;
		PX4_INFO("\tPWM CH1: %hu::%uus::%u tics", outputs[0], tics_1 / 24, tics_1);
		uint16_t tics_2 = (_parameters.pwm_min + (_pwm_fullscale * ((double)outputs[1] / VOXL2_IO_MIXER_MAX))) * VOXL2_IO_TICS;
		PX4_INFO("\tPWM CH2: %u::%uus::%u tics", outputs[1], tics_2 / 24, tics_2);
		uint16_t tics_3 = (_parameters.pwm_min + (_pwm_fullscale * ((double)outputs[2] / VOXL2_IO_MIXER_MAX))) * VOXL2_IO_TICS;
		PX4_INFO("\tPWM CH3: %u::%uus::%u tics", outputs[2], tics_3 / 24, tics_3);
		uint16_t tics_4 = (_parameters.pwm_min + (_pwm_fullscale * ((double)outputs[3] / VOXL2_IO_MIXER_MAX))) * VOXL2_IO_TICS;
		PX4_INFO("\tPWM CH4: %u::%uus::%u tics", outputs[3], tics_4 / 24, tics_4);
		PX4_INFO("");
	}

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
		return false;

	} else {
		_bytes_sent += cmd.len;
		_packets_sent++;
	}

	perf_count(_output_update_perf);

	return true;
}

int Voxl2IO::flush_uart_rx()
{
	while (_uart_port->uart_read(_read_buf, sizeof(_read_buf)) > 0) {}

	return 0;
}

static bool valid_port(int port)
{
	if (port == 2 || port == 6 || port == 7) { return true; }

	return false;
}

int Voxl2IO::parse_response(uint8_t *buf, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		int16_t ret = voxl2_io_packet_process_char(buf[i], &_sbus_packet);

		if (ret > 0) {
			uint8_t packet_type = voxl2_io_packet_get_type(&_sbus_packet);
			uint8_t packet_size = voxl2_io_packet_get_size(&_sbus_packet);

			if (packet_type == VOXL2_IO_PACKET_TYPE_RC_DATA_RAW && packet_size == VOXL2_IO_SBUS_FRAME_SIZE) {
				return 0;

			} else if (packet_type == VOXL2_IO_PACKET_TYPE_VERSION_RESPONSE && packet_size == sizeof(VOXL2_IO_VERSION_INFO)) {
				return 0;

			} else {
				return -1;
			}

		} else { //parser error
			switch (ret) {
			case VOXL2_IO_ERROR_BAD_CHECKSUM:
				if (_pwm_on && _debug) { PX4_WARN("BAD packet checksum"); }

				break;

			case VOXL2_IO_ERROR_BAD_LENGTH:
				if (_pwm_on && _debug) { PX4_WARN("BAD packet length"); }

				break;

			case VOXL2_IO_ERROR_BAD_HEADER:
				if (_pwm_on && _debug) { PX4_WARN("BAD packet header"); }

				break;

			case VOXL2_IO_NO_PACKET:
				// if(_pwm_on) PX4_WARN("NO packet");
				break;

			default:
				if (_pwm_on && _debug) { PX4_WARN("Unknown error: %i", ret); }

				break;
			}

			return ret;
		}
	}

	return 0;
}

void Voxl2IO::fill_rc_in(uint16_t raw_rc_count_local,
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

	if (frame_drops) {
		_sbus_frame_drops++;
	}

	input_rc.rc_failsafe = failsafe;
	input_rc.rc_lost = input_rc.rc_failsafe;
	input_rc.rc_lost_frame_count = _sbus_frame_drops;
	input_rc.rc_total_frame_count = ++_sbus_total_frames;
}

int Voxl2IO::receive_sbus()
{
	int res = 0;
	int header = -1;
	int read_retries = 3;
	int read_succeeded = 0;
	voxl2_io_packet_init(&_sbus_packet);

	while (read_retries) {
		memset(&_read_buf, 0x00, READ_BUF_SIZE);
		res = _uart_port->uart_read(_read_buf, READ_BUF_SIZE);

		if (res) {
			/* Get index of packer header */
			for (int index = 0; index < READ_BUF_SIZE; ++index) {
				if (_read_buf[index] == VOXL2_IO_PACKET_HEADER) {
					header = index;
					break;
				}
			}

			/* Try again in a bit if packet header not present yet... */
			if (header == -1) {
				if (_debug) { PX4_ERR("Failed to find SBUS packet header, trying again... retries left: %i", read_retries); }

				read_retries--;
				continue;
			}

			/* Check if we got a valid packet...*/
			if (parse_response(&_read_buf[header], (uint8_t)VOXL2_IO_SBUS_FRAME_SIZE)) {
				if (_pwm_on && _debug) {
					PX4_ERR("Error parsing QC RAW SBUS packet");
					PX4_INFO_RAW("[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x]\n",
						     _read_buf[header + 0], _read_buf[header + 1], _read_buf[header + 2], _read_buf[header + 3], _read_buf[header + 4],
						     _read_buf[header + 5],
						     _read_buf[header + 6], _read_buf[header + 7], _read_buf[header + 8], _read_buf[header + 9], _read_buf[header + 10],
						     _read_buf[header + 11],
						     _read_buf[header + 12], _read_buf[header + 13], _read_buf[header + 14], _read_buf[header + 15], _read_buf[header + 16],
						     _read_buf[header + 17],
						     _read_buf[header + 18], _read_buf[header + 19], _read_buf[header + 20], _read_buf[header + 21], _read_buf[header + 22],
						     _read_buf[header + 23],
						     _read_buf[header + 24], _read_buf[header + 25], _read_buf[header + 26], _read_buf[header + 27], _read_buf[header + 28],
						     _read_buf[header + 29]
						    );
				}

				read_retries--;
				break;
			}

			input_rc_s input_rc;
			uint16_t num_values;
			bool sbus_failsafe = false;
			bool sbus_frame_drop = false;
			uint16_t max_channels = sizeof(_raw_rc_values) / sizeof(_raw_rc_values[0]);
			hrt_abstime now = hrt_absolute_time();
			bool rc_updated = sbus_parse(now, &_read_buf[header + SBUS_PAYLOAD], SBUS_FRAME_SIZE, _raw_rc_values, &num_values,
						     &sbus_failsafe, &sbus_frame_drop, &_sbus_frame_drops, max_channels);

			if (rc_updated) {
				if (_pwm_on && _debug) {
					PX4_INFO("Decoded packet, header pos: %i", header);
					PX4_INFO("[%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]",
						 _raw_rc_values[0], _raw_rc_values[1], _raw_rc_values[2],
						 _raw_rc_values[3], _raw_rc_values[4], _raw_rc_values[5],
						 _raw_rc_values[6], _raw_rc_values[7], _raw_rc_values[8],
						 _raw_rc_values[9], _raw_rc_values[10], _raw_rc_values[11],
						 _raw_rc_values[12], _raw_rc_values[13], _raw_rc_values[14],
						 _raw_rc_values[15], _raw_rc_values[16], _raw_rc_values[17]
						);
				}

				input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;
				fill_rc_in(num_values, _raw_rc_values, now, sbus_frame_drop, sbus_failsafe, _sbus_frame_drops, -1, input_rc);

				if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
					_rc_last_valid = input_rc.timestamp;
				}

				input_rc.timestamp_last_signal = _rc_last_valid;
				_rc_pub.publish(input_rc);

				_bytes_received += res;
				_packets_received++;
				read_succeeded = 1;
				break;

			} else if (_pwm_on && _debug) {
				PX4_ERR("Failed to decode SBUS packet, header pos: %i", header);

				if (sbus_frame_drop) {
					PX4_WARN("SBUS frame dropped");
				}

				PX4_ERR("[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x]",
					_read_buf[header + 0], _read_buf[header + 1], _read_buf[header + 2], _read_buf[header + 3], _read_buf[header + 4],
					_read_buf[header + 5],
					_read_buf[header + 6], _read_buf[header + 7], _read_buf[header + 8], _read_buf[header + 9], _read_buf[header + 10],
					_read_buf[header + 11],
					_read_buf[header + 12], _read_buf[header + 13], _read_buf[header + 14], _read_buf[header + 15], _read_buf[header + 16],
					_read_buf[header + 17],
					_read_buf[header + 18], _read_buf[header + 19], _read_buf[header + 20], _read_buf[header + 21], _read_buf[header + 22],
					_read_buf[header + 23],
					_read_buf[header + 24], _read_buf[header + 25], _read_buf[header + 26], _read_buf[header + 27], _read_buf[header + 28],
					_read_buf[header + 29]
				       );
			}
		}

		read_retries--;
	}

	if (! read_succeeded) {
		_new_packet = false;
		return -EIO;
	}

	_new_packet = true;
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

	/* Handle RC */
	if (_rc_mode == RC_MODE::SCAN) {
		if (receive_sbus() == PX4_OK) {
			PX4_INFO("Found M0065 SBUS RC.");
			_rc_mode = RC_MODE::SBUS;
		}	// Add more cases here for other protocols in the future..

	} else if (_rc_mode == RC_MODE::SBUS) {
		receive_sbus();
	}

	/* Only update outputs if we have new values from RC */
	if (_new_packet || _rc_mode == RC_MODE::EXTERNAL) {
		_mixing_output.update(); //calls MixingOutput::limitAndUpdateOutputs which calls updateOutputs in this module
		_new_packet = false;
	}

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
			PX4_INFO("sending %d commands with delay %dus", _current_cmd.repeats, _current_cmd.repeat_delay_us);
			flush_uart_rx();

			do {
				PX4_INFO("CMDs left %d", _current_cmd.repeats);

				if (_uart_port->uart_write(_current_cmd.buf, _current_cmd.len) == _current_cmd.len) {
					if (_current_cmd.repeats == 0) {
						_current_cmd.clear();
					}


				} else {
					_bytes_sent += _current_cmd.len;
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
				PX4_INFO("Verbose mode enabled");
				get_instance()->_debug = true;
				break;

			case 'd':
				PX4_INFO("M0065 PWM outputs disabled");
				get_instance()->_outputs_disabled = true;
				break;

			case 'e':
				PX4_INFO("M0065 using external RC");
				get_instance()->_rc_mode = RC_MODE::EXTERNAL;
				break;

			case 'p':
				if (valid_port(atoi(myoptarg))) {
					snprintf(get_instance()->_device, 2, "%s", myoptarg);

				} else {
					PX4_ERR("Bad UART port number: %s (must be 2, 6, or 7).", myoptarg);
					_object.store(nullptr);
					_task_id = -1;
					return PX4_ERROR;
				}

				break;

			default:
				print_usage("Unknown command, parsing flags");
				break;
			}
		}

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


bool Voxl2IO::stop_all_pwms()
{
	int16_t _rate_req[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	int16_t _led_req[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	uint8_t _fb_idx = 0;

	Command cmd;
	cmd.len = voxl2_io_create_pwm_packet4_fb(_rate_req[0], _rate_req[1], _rate_req[2], _rate_req[3],
			_led_req[0], _led_req[1], _led_req[2], _led_req[3],
			_fb_idx, cmd.buf, sizeof(cmd.buf));

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
		return false;

	} else {
		_bytes_sent += cmd.len;
		_packets_sent++;
	}

	return true;
}

int Voxl2IO::send_cmd_thread_safe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}

int Voxl2IO::calibrate_escs()
{

	/* Disable outputs so Mixer isn't being a PITA while we calibrate */
	_outputs_disabled = true;

	Command cmd;
	int32_t fb_idx = -1;
	uint8_t data[VOXL2_IO_ESC_CAL_SIZE] {0};
	cmd.len = voxl2_io_create_packet(VOXL2_IO_PACKET_TYPE_TUNE_CONFIG, data, VOXL2_IO_ESC_CAL_SIZE, cmd.buf,
					 sizeof(cmd.buf));

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("ESC Calibration failed: Failed to send PWM OFF packet");
		_outputs_disabled = false;
		return -1;
	}

	/* Give user 10 seconds to plug in PWM cable for ESCs */
	PX4_INFO("Disconnected and reconnect your ESCs! (Calibration will start in ~10 seconds)");
	hrt_abstime start_cal = hrt_absolute_time();

	while (hrt_elapsed_time(&start_cal) < 10000000) {
		continue;
	}

	/* PWM MAX 3 seconds */
	PX4_INFO("Writing PWM MAX for 3 seconds!");
	int16_t max_pwm[4] {VOXL2_IO_MIXER_MAX, VOXL2_IO_MIXER_MAX, VOXL2_IO_MIXER_MAX, VOXL2_IO_MIXER_MAX};

	if (_debug) { PX4_INFO("%i %i %i %i", max_pwm[0], max_pwm[1], max_pwm[2], max_pwm[3]); }

	int16_t led_cmd[4] {0, 0, 0, 0};
	cmd.len = voxl2_io_create_pwm_packet4_fb(max_pwm[0], max_pwm[1], max_pwm[2], max_pwm[3],
			led_cmd[0], led_cmd[1], led_cmd[2], led_cmd[3],
			fb_idx, cmd.buf, sizeof(cmd.buf));

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("ESC Calibration failed: Failed to send PWM MAX packet");
		_outputs_disabled = false;
		return -1;

	} else {
		cmd.clear();
	}

	hrt_abstime start_pwm_max = hrt_absolute_time();

	while (hrt_elapsed_time(&start_pwm_max) < 3000000) {
		continue;
	}

	/* PWM MIN 4 seconds */
	PX4_INFO("Writing PWM MIN for 4 seconds!");
	int16_t min_pwm[4] {VOXL2_IO_MIXER_MIN, VOXL2_IO_MIXER_MIN, VOXL2_IO_MIXER_MIN, VOXL2_IO_MIXER_MIN};

	if (_debug) { PX4_INFO("%i %i %i %i", min_pwm[0], min_pwm[1], min_pwm[2], min_pwm[3]); }

	cmd.len = voxl2_io_create_pwm_packet4_fb(min_pwm[0], min_pwm[1], min_pwm[2], min_pwm[3],
			led_cmd[0], led_cmd[1], led_cmd[2], led_cmd[3],
			fb_idx, cmd.buf, sizeof(cmd.buf));

	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("ESC Calibration failed: Failed to send PWM MIN packet");
		_outputs_disabled = false;
		return -1;
	}

	hrt_abstime start_pwm_min = hrt_absolute_time();

	while (hrt_elapsed_time(&start_pwm_min) < 4000000) {
		continue;
	}

	PX4_INFO("ESC Calibration complete");

	_outputs_disabled = false;
	return 0;
}

int Voxl2IO::custom_command(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	Command  cmd;
	uint8_t  output_channel   = 0xF;
	int16_t  rate     = 0;

	uint32_t repeat_count    = 100;
	uint32_t repeat_delay_us = 10000;

	const char *verb = argv[argc - 1];

	if ((strcmp(verb, "pwm")) == 0 && argc < 3) {
		return print_usage("pwm command: missing args");

	} else if (argc < 1) {
		return print_usage("unknown command: missing args");
	}

	PX4_INFO("Executing the following command: %s", verb);

	/* start the FMU if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return Voxl2IO::task_spawn(argc, argv);
		}
	}

	if (!strcmp(verb, "status")) {
		if (!is_running()) {
			PX4_INFO("Not running");
			return -1;
		}

		return get_instance()->print_status();
	}

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;
	}

	if (!strcmp(verb, "calibrate_escs")) {
		if (get_instance()->_outputs_disabled) {
			PX4_WARN("Can't calibrate ESCs while outputs are disabled.");
			return -1;
		}

		return get_instance()->calibrate_escs();
	}

	while ((ch = px4_getopt(argc, argv, "c:n:t:r:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'c':
			output_channel = atoi(myoptarg);

			if (output_channel > VOXL2_IO_OUTPUT_CHANNELS - 1) {
				char reason[50];
				sprintf(reason, "Bad channel value: %d. Must be 0-%d.", output_channel, VOXL2_IO_OUTPUT_CHANNELS - 1);
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

		case 'p':
			if (valid_port(atoi(myoptarg))) {
				snprintf(get_instance()->_device, 2, "%s", myoptarg);

			} else {
				PX4_ERR("Bad UART port number: %s (must be 2, 6, or 7).", myoptarg);
				return 0;
			}

			break;

		default:
			print_usage("Unknown command, parsing flags");
			return 0;
		}
	}

	if (!strcmp(verb, "pwm")) {
		PX4_INFO("Output channel: %i", output_channel);
		PX4_INFO("Repeat count: %i", repeat_count);
		PX4_INFO("Repeat delay (us): %i", repeat_delay_us);
		PX4_INFO("Rate: %i", rate);

		if (output_channel < VOXL2_IO_OUTPUT_CHANNELS) {
			PX4_INFO("Request PWM for Output Channel: %i - PWM: %i", output_channel, rate);
			int16_t rate_req[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0};
			uint8_t id_fb = 0;

			if (output_channel ==
			    0xFF) {  //WARNING: this condition is not possible due to check 'if (esc_id < MODAL_IO_OUTPUT_CHANNELS)'.
				rate_req[0] = rate;
				rate_req[1] = rate;
				rate_req[2] = rate;
				rate_req[3] = rate;

			} else {
				rate_req[output_channel] = rate;
				id_fb = output_channel;
			}

			cmd.len = voxl2_io_create_pwm_packet4_fb(rate_req[0], rate_req[1], rate_req[2], rate_req[3],
					0, 0, 0, 0,
					id_fb, cmd.buf, sizeof(cmd.buf));

			cmd.response        = false;
			cmd.repeats         = repeat_count;
			cmd.resp_delay_us   = 1000;
			cmd.repeat_delay_us = repeat_delay_us;
			cmd.print_feedback  = false;

			PX4_INFO("feedback id debug: %i", id_fb);
			PX4_INFO("Sending UART M0065 power command %i", rate);

			if (get_instance()->_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
				PX4_ERR("Failed to send packet: stop PWMs");
				return -1;

			} else {
				get_instance()->_bytes_sent += cmd.len;
				get_instance()->_packets_sent++;
			}

		} else {
			print_usage("Invalid Output Channel, use 0-3");
			return 0;
		}
	}

	return print_usage("unknown custom command");
}

int Voxl2IO::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);
	PX4_INFO("PWM Rate: 400 Hz");	// Only support 400 Hz for now
	PX4_INFO("Outputs on: %s", _pwm_on ? "yes" : "no");
	PX4_INFO("FW version: v%u.%u", _version_info.sw_version, _version_info.hw_version);
	PX4_INFO("RC Type: SBUS");		// Only support SBUS through M0065 for now
	PX4_INFO("RC Connected: %s", hrt_absolute_time() - _rc_last_valid > 500000 ? "no" : "yes");
	PX4_INFO("RC Packets Received: %" PRIu16, _sbus_total_frames);
	PX4_INFO("UART port: %s", _device);
	PX4_INFO("UART open: %s", _uart_port->is_open() ? "yes" : "no");
	PX4_INFO("Packets sent: %" PRIu32, _packets_sent);
	PX4_INFO("");
	PX4_INFO("Params: VOXL2_IO_BAUD: %" PRId32, _parameters.baud_rate);
	PX4_INFO("Params: VOXL2_IO_FUNC1: %" PRId32, _parameters.function_map[0]);
	PX4_INFO("Params: VOXL2_IO_FUNC2: %" PRId32, _parameters.function_map[1]);
	PX4_INFO("Params: VOXL2_IO_FUNC3: %" PRId32, _parameters.function_map[2]);
	PX4_INFO("Params: VOXL2_IO_FUNC4: %" PRId32, _parameters.function_map[3]);

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate_escs", "Calibrate ESCs min/max range");
	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "Open-Loop PWM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, 3, "PWM OUTPUT Channel, 0-3", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 800, "Duty Cycle value, 0 to 800", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 100, 0, 1<<31, "Command repeat count, 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 10000, 0, 1<<31, "Delay between repeated commands (microseconds), 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int voxl2_io_main(int argc, char *argv[]);

int voxl2_io_main(int argc, char *argv[])
{
	return Voxl2IO::main(argc, argv);
}
