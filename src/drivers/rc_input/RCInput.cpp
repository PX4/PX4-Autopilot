/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include "RCInput.hpp"

#include "crsf_telemetry.h"
#include <uORB/topics/vehicle_command_ack.h>

#include <termios.h>

using namespace time_literals;

constexpr char const *RCInput::RC_PARSER_STRING[];

RCInput::RCInput(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
	// initialize it as RC lost
	_rc_in.rc_lost = true;

	// initialize raw_rc values and count
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		_raw_rc_values[i] = UINT16_MAX;
	}

	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

RCInput::~RCInput()
{
#if defined(SPEKTRUM_POWER_PASSIVE)
	// Disable power controls for Spektrum receiver
	SPEKTRUM_POWER_PASSIVE();
#endif
	dsm_deinit();

	delete _crsf_telemetry;
	delete _ghst_telemetry;

	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int
RCInput::init()
{
#ifdef RF_RADIO_POWER_CONTROL
	// power radio on
	RF_RADIO_POWER_CONTROL(true);
#endif // RF_RADIO_POWER_CONTROL

	// dsm_init sets some file static variables and returns a file descriptor
	// it also powers on the radio if needed
	_rcs_fd = dsm_init(_device);

	if (_rcs_fd < 0) {
		return -errno;
	}

	if (board_rc_swap_rxtx(_device)) {
#if defined(TIOCSSWAP)
		ioctl(_rcs_fd, TIOCSSWAP, SER_SWAP_ENABLED);
#endif // TIOCSSWAP
	}

	// assume SBUS input and immediately switch it to
	// so that if Single wire mode on TX there will be only
	// a short contention
	sbus_config(_rcs_fd, board_rc_singlewire(_device));

#ifdef GPIO_PPM_IN
	// disable CPPM input by mapping it away from the timer capture input
	px4_arch_unconfiggpio(GPIO_PPM_IN);
#endif // GPIO_PPM_IN

	return 0;
}

int
RCInput::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;
#if defined(RC_SERIAL_PORT)
	device_name = RC_SERIAL_PORT;
#endif // RC_SERIAL_PORT

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		RCInput *instance = new RCInput(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(_current_update_interval);

		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

void
RCInput::fill_rc_in(uint16_t raw_rc_count_local,
		    uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
		    hrt_abstime now, bool frame_drop, bool failsafe,
		    unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count_local;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}

		// once filled, reset values back to default
		_raw_rc_values[i] = UINT16_MAX;
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {
		if ((_param_rc_rssi_pwm_chan.get() > 0) && (_param_rc_rssi_pwm_chan.get() < _rc_in.channel_count)) {
			const int32_t rssi_pwm_chan = _param_rc_rssi_pwm_chan.get();
			const int32_t rssi_pwm_min = _param_rc_rssi_pwm_min.get();
			const int32_t rssi_pwm_max = _param_rc_rssi_pwm_max.get();

			// get RSSI from input channel
			int rc_rssi = ((_rc_in.values[rssi_pwm_chan - 1] - rssi_pwm_min) * 100) / (rssi_pwm_max - rssi_pwm_min);
			_rc_in.rssi = math::constrain(rc_rssi, 0, 100);

		} else if (_analog_rc_rssi_stable) {
			// set RSSI if analog RSSI input is present
			float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

			if (rssi_analog > 100.0f) {
				rssi_analog = 100.0f;
			}

			if (rssi_analog < 0.0f) {
				rssi_analog = 0.0f;
			}

			_rc_in.rssi = rssi_analog;

		} else {
			_rc_in.rssi = 255;
		}

	} else {
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

void RCInput::rc_io_invert(bool invert)
{
	// First check if the board provides a board-specific inversion method (e.g. via GPIO),
	// and if not use an IOCTL
	if (!board_rc_invert_input(_device, invert)) {
#if defined(TIOCSINVERT)

		if (invert) {
			ioctl(_rcs_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX);

		} else {
			ioctl(_rcs_fd, TIOCSINVERT, 0);
		}

#endif // TIOCSINVERT
	}
}

void RCInput::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}

	} else {

		perf_begin(_cycle_perf);

		// Check if parameters have changed
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);

			updateParams();
		}

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			}
		}

		/* vehicle command */
		vehicle_command_s vcmd;

		if (_vehicle_cmd_sub.update(&vcmd)) {
			// Check for a pairing command
			if (vcmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) {

				uint8_t cmd_ret = vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
#if defined(SPEKTRUM_POWER)

				if (!_rc_scan_locked && !_armed) {
					if ((int)vcmd.param1 == 0) {
						// DSM binding command
						int dsm_bind_mode = (int)vcmd.param2;

						int dsm_bind_pulses = 0;

						if (dsm_bind_mode == 0) {
							dsm_bind_pulses = DSM2_BIND_PULSES;

						} else if (dsm_bind_mode == 1) {
							dsm_bind_pulses = DSMX_BIND_PULSES;

						} else {
							dsm_bind_pulses = DSMX8_BIND_PULSES;
						}

						bind_spektrum(dsm_bind_pulses);

						cmd_ret = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
					}

				} else {
					cmd_ret = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

#endif // SPEKTRUM_POWER

				// publish acknowledgement
				vehicle_command_ack_s command_ack{};
				command_ack.command = vcmd.command;
				command_ack.result = cmd_ret;
				command_ack.target_system = vcmd.source_system;
				command_ack.target_component = vcmd.source_component;
				command_ack.timestamp = hrt_absolute_time();
				uORB::Publication<vehicle_command_ack_s> vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
				vehicle_command_ack_pub.publish(command_ack);
			}
		}


#if defined(ADC_RC_RSSI_CHANNEL)

		// update ADC sampling
		if (_adc_report_sub.updated()) {
			adc_report_s adc;

			if (_adc_report_sub.copy(&adc)) {
				for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
					if (adc.channel_id[i] == ADC_RC_RSSI_CHANNEL) {
						float adc_volt = adc.raw_data[i] *
								 adc.v_ref /
								 adc.resolution;

						if (_analog_rc_rssi_volt < 0.0f) {
							_analog_rc_rssi_volt = adc_volt;
						}

						_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.995f + adc_volt * 0.005f;

						/* only allow this to be used if we see a high RSSI once */
						if (_analog_rc_rssi_volt > 2.5f) {
							_analog_rc_rssi_stable = true;
						}
					}
				}
			}
		}

#endif // ADC_RC_RSSI_CHANNEL

		const hrt_abstime cycle_timestamp = hrt_absolute_time();
		enum RC_PARSER request_rc_parser = RC_PARSER_NONE;

		switch (_param_rc_input_proto.get()) {
		case RC_PROTO_SELECT_AUTO:
			// Auto scanner has control of requested parser
			request_rc_parser = scanner_check(cycle_timestamp);
			break;

		case RC_PROTO_SELECT_SBUS:
			request_rc_parser = RC_PARSER_SBUS;
			break;

		case RC_PROTO_SELECT_DSM:
			request_rc_parser = RC_PARSER_DSM;
			break;

		case RC_PROTO_SELECT_ST24:
			request_rc_parser = RC_PARSER_ST24;
			break;

		case RC_PROTO_SELECT_SUMD:
			request_rc_parser = RC_PARSER_SUMD;
			break;
#if defined(HRT_PPM_CHANNEL)

		case RC_PROTO_SELECT_PPM:
			request_rc_parser = RC_PARSER_PPM;
			break;
#endif // HRT_PPM_CHANNEL

		case RC_PROTO_SELECT_CRSF:
			request_rc_parser = RC_PARSER_CRSF;
			break;

		case RC_PROTO_SELECT_GHST:
			request_rc_parser = RC_PARSER_GHST;
			break;

		default:
			request_rc_parser = RC_PARSER_NONE;
			break;
		}

		// If the requested parser is not the current parser, teardown/setup for new parser
		if (request_rc_parser != _current_rc_parser) {
			switch_parser(request_rc_parser);
		}

		// read all available data from the serial RC input UART
		int new_bytes = ::read(_rcs_fd, &_rcs_buf[0], RC_MAX_BUFFER_SIZE);
		bool rc_updated = false;

		if (new_bytes > 0) {
			_bytes_rx += new_bytes;
		}

		switch (request_rc_parser) {
		case RC_PARSER_NONE:
			break;

		case RC_PARSER_SBUS:
			rc_updated = try_parse_sbus(cycle_timestamp, new_bytes);
			break;

		case RC_PARSER_DSM:
			rc_updated = try_parse_dsm(cycle_timestamp, new_bytes);
			break;

		case RC_PARSER_ST24:
			rc_updated = try_parse_st24(cycle_timestamp, new_bytes);
			break;

		case RC_PARSER_SUMD:
			rc_updated = try_parse_sumd(cycle_timestamp, new_bytes);
			break;

#if defined(HRT_PPM_CHANNEL)

		case RC_PARSER_PPM:
			rc_updated = try_parse_ppm(cycle_timestamp);
			break;
#endif // HRT_PPM_CHANNEL

		case RC_PARSER_CRSF:
			rc_updated = try_parse_crsf(cycle_timestamp, new_bytes);
			break;

		case RC_PARSER_GHST:
			rc_updated = try_parse_ghst(cycle_timestamp, new_bytes);
			break;

		default:
			break;
		}

		if (rc_updated) {
			_rc_scan_locked = true;
			perf_count(_publish_interval_perf);

			_to_input_rc.publish(_rc_in);
		}

		perf_end(_cycle_perf);
	}
}

void RCInput::switch_parser(enum RC_PARSER new_parser)
{
	if (new_parser == _current_rc_parser) {
		return;
	}

	switch (_current_rc_parser) {
	case RC_PARSER_NONE:
		break;

	case RC_PARSER_SBUS:
		rc_io_invert(false);
		break;

	case RC_PARSER_DSM:
		break;

	case RC_PARSER_ST24:
		break;

	case RC_PARSER_SUMD:
		break;

#if defined(HRT_PPM_CHANNEL)

	case RC_PARSER_PPM:
		// disable CPPM input by mapping it away from the timer capture input
		px4_arch_unconfiggpio(GPIO_PPM_IN);
		break;
#endif // HRT_PPM_CHANNEL

	case RC_PARSER_CRSF:
		break;

	case RC_PARSER_GHST:
		break;

	default:
		break;
	}

	switch (new_parser) {
	case RC_PARSER_NONE:
		break;

	case RC_PARSER_SBUS:
		// Configure serial port for SBUS
		sbus_config(_rcs_fd, board_rc_singlewire(_device));
		rc_io_invert(true);

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);
		memset(_rcs_buf, 0, sizeof(_rcs_buf));

		break;

	case RC_PARSER_DSM:
		// Configure serial port for DSM
		dsm_config(_rcs_fd);

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);
		memset(_rcs_buf, 0, sizeof(_rcs_buf));
		break;

	case RC_PARSER_ST24:
		// Configure serial port for DSM
		dsm_config(_rcs_fd);

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);
		memset(_rcs_buf, 0, sizeof(_rcs_buf));
		break;

	case RC_PARSER_SUMD:
		// Configure serial port for DSM
		dsm_config(_rcs_fd);

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);
		memset(_rcs_buf, 0, sizeof(_rcs_buf));
		break;

#if defined(HRT_PPM_CHANNEL)

	case RC_PARSER_PPM:
		// Configure timer input pin for CPPM
		px4_arch_configgpio(GPIO_PPM_IN);
		break;
#endif // HRT_PPM_CHANNEL

	case RC_PARSER_CRSF:
		// Configure serial port for CRSF
		crsf_config(_rcs_fd);

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);
		memset(_rcs_buf, 0, sizeof(_rcs_buf));
		break;

	case RC_PARSER_GHST:
		// Configure serial port for GHST
		ghst_config(_rcs_fd);

		// flush serial buffer and any existing buffered data
		tcflush(_rcs_fd, TCIOFLUSH);
		memset(_rcs_buf, 0, sizeof(_rcs_buf));
		break;

	default:
		break;
	}

	PX4_INFO("Parser switch %s -> %s",  RCInput::RC_PARSER_STRING[_current_rc_parser + 1],
		 RCInput::RC_PARSER_STRING[new_parser + 1]);

	_current_rc_parser = new_parser;
}

bool RCInput::try_parse_crsf(hrt_abstime cycle_timestamp, int new_bytes)
{
	bool rc_updated = false;

	// parse new data
	if (new_bytes > 0) {
		rc_updated = crsf_parse(cycle_timestamp, &_rcs_buf[0], new_bytes, &_raw_rc_values[0], &_raw_rc_count,
					input_rc_s::RC_INPUT_MAX_CHANNELS);

		if (rc_updated) {
			// we have a new CRSF frame. Publish it.
			_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF;
			fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp, false, false, 0);

			// on Pixhawk (-related) boards we cannot write to the RC UART
			// another option is to use a different UART port
#ifdef BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

			if (!_crsf_telemetry) {
				_crsf_telemetry = new CRSFTelemetry(_rcs_fd);
			}

#endif /* BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT */

			if (_crsf_telemetry) {
				_crsf_telemetry->update(cycle_timestamp);
			}
		}
	}

	return rc_updated;
}

bool RCInput::try_parse_sbus(hrt_abstime cycle_timestamp, int new_bytes)
{
	bool rc_updated = false;
	unsigned frame_drops = 0;

	// parse new data
	if (new_bytes > 0) {
		bool sbus_failsafe = false;
		bool sbus_frame_drop = false;

		rc_updated = sbus_parse(cycle_timestamp, &_rcs_buf[0], new_bytes, &_raw_rc_values[0], &_raw_rc_count, &sbus_failsafe,
					&sbus_frame_drop, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

		if (rc_updated) {
			// we have a new SBUS frame. Publish it.
			_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;
			fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
				   sbus_frame_drop, sbus_failsafe, frame_drops);
		}
	}

	return rc_updated;
}

bool RCInput::try_parse_dsm(hrt_abstime cycle_timestamp, int new_bytes)
{
	bool rc_updated = false;
	unsigned frame_drops = 0;

	if (new_bytes > 0) {
		int8_t dsm_rssi = 0;
		bool dsm_11_bit = false;

		// parse new data
		rc_updated = dsm_parse(cycle_timestamp, &_rcs_buf[0], new_bytes, &_raw_rc_values[0], &_raw_rc_count,
				       &dsm_11_bit, &frame_drops, &dsm_rssi, input_rc_s::RC_INPUT_MAX_CHANNELS);

		if (rc_updated) {
			// we have a new DSM frame. Publish it.
			_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;
			fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
				   false, false, frame_drops, dsm_rssi);
		}
	}

	return rc_updated;
}

bool RCInput::try_parse_st24(hrt_abstime cycle_timestamp, int new_bytes)
{
	bool rc_updated = false;
	unsigned frame_drops = 0;

	if (new_bytes > 0) {
		// parse new data
		uint8_t st24_rssi, lost_count;

		rc_updated = false;

		for (unsigned i = 0; i < (unsigned)new_bytes; i++) {
			/* set updated flag if one complete packet was parsed */
			st24_rssi = input_rc_s::RSSI_MAX;
			rc_updated = (OK == st24_decode(_rcs_buf[i], &st24_rssi, &lost_count,
							&_raw_rc_count, _raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS));
		}

		// The st24 will keep outputting RC channels and RSSI even if RC has been lost.
		// The only way to detect RC loss is therefore to look at the lost_count.

		if (rc_updated) {
			if (lost_count == 0) {
				// we have a new ST24 frame. Publish it.
				_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24;
				fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
					   false, false, frame_drops, st24_rssi);

			} else {
				// if the lost count > 0 means that there is an RC loss
				_rc_in.rc_lost = true;
			}
		}
	}

	return rc_updated;
}

bool RCInput::try_parse_sumd(hrt_abstime cycle_timestamp, int new_bytes)
{
	bool rc_updated = false;
	unsigned frame_drops = 0;

	if (new_bytes > 0) {
		// parse new data
		uint8_t sumd_rssi, rx_count;
		bool sumd_failsafe;

		rc_updated = false;

		for (unsigned i = 0; i < (unsigned)new_bytes; i++) {
			/* set updated flag if one complete packet was parsed */
			sumd_rssi = input_rc_s::RSSI_MAX;
			rc_updated = (OK == sumd_decode(_rcs_buf[i], &sumd_rssi, &rx_count,
							&_raw_rc_count, _raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS, &sumd_failsafe));
		}

		if (rc_updated) {
			// we have a new SUMD frame. Publish it.
			_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD;
			fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
				   false, sumd_failsafe, frame_drops, sumd_rssi);
		}
	}

	return rc_updated;
}

bool RCInput::try_parse_ghst(hrt_abstime cycle_timestamp, int new_bytes)
{
	bool rc_updated = false;

	// parse new data
	if (new_bytes > 0) {
		int8_t ghst_rssi = -1;
		rc_updated = ghst_parse(cycle_timestamp, &_rcs_buf[0], new_bytes, &_raw_rc_values[0], &ghst_rssi,
					&_raw_rc_count, input_rc_s::RC_INPUT_MAX_CHANNELS);

		if (rc_updated) {
			// we have a new GHST frame. Publish it.
			_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_GHST;
			fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp, false, false, 0, ghst_rssi);

			// ghst telemetry works on fmu-v5
			// on other Pixhawk (-related) boards we cannot write to the RC UART
			// another option is to use a different UART port
#ifdef BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

			if (!_ghst_telemetry) {
				_ghst_telemetry = new GHSTTelemetry(_rcs_fd);
			}

#endif /* BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT */

			if (_ghst_telemetry) {
				_ghst_telemetry->update(cycle_timestamp);
			}
		}
	}

	return rc_updated;
}

#if defined(HRT_PPM_CHANNEL)
bool RCInput::try_parse_ppm(hrt_abstime cycle_timestamp)
{
	// see if we have new PPM input data
	if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal) && ppm_decoded_channels > 3) {
		// we have a new PPM frame. Publish it.
		_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
		fill_rc_in(ppm_decoded_channels, ppm_buffer, cycle_timestamp, false, false, 0);
		_rc_in.rc_ppm_frame_length = ppm_frame_length;
		_rc_in.timestamp_last_signal = ppm_last_valid_decode;

		return true;
	}

	return false;
}
#endif // HRT_PPM_CHANNEL

RCInput::RC_PARSER RCInput::scanner_check(hrt_abstime cycle_timestamp)
{
	constexpr hrt_abstime rc_scan_max = 1_s;

	if (_rc_scan_locked) {
		// If we're locked but running the autoscanner, update the proto selection param
		// +1 to deal with RC_PARSER_NONE
		_param_rc_input_proto.set(_current_rc_parser);

		PX4_INFO("RC protocol SELECTED! %s", RC_PARSER_STRING[_current_rc_parser + 1]);

		return _current_rc_parser;
	}

	// We havn't gotten a frame yet, see if it's time to move onto the next parser
	if (cycle_timestamp - _rc_scan_begin > rc_scan_max) {
		_rc_scan_begin = hrt_absolute_time();
		RCInput::RC_PARSER new_parser = static_cast<RC_PARSER>((_current_rc_parser + 1) % PARSER_COUNT);

		PX4_INFO("RC protocol scan switched to %s, %llu", RC_PARSER_STRING[new_parser + 1], cycle_timestamp);
		return new_parser;
	}

	return _current_rc_parser;
}

#if defined(SPEKTRUM_POWER)
bool RCInput::bind_spektrum(int arg) const
{
	int ret = PX4_ERROR;

	/* specify 11ms DSMX. RX will automatically fall back to 22ms or DSM2 if necessary */

	/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
	PX4_INFO("DSM_BIND_START: DSM%s RX", (arg == 0) ? "2" : ((arg == 1) ? "-X" : "-X8"));

	if (arg == DSM2_BIND_PULSES ||
	    arg == DSMX_BIND_PULSES ||
	    arg == DSMX8_BIND_PULSES) {

		dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);

		dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);
		usleep(500000);

		dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
		usleep(72000);

		irqstate_t flags = px4_enter_critical_section();
		dsm_bind(DSM_CMD_BIND_SEND_PULSES, arg);
		px4_leave_critical_section(flags);

		usleep(50000);

		dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

		ret = OK;

	} else {
		PX4_ERR("DSM bind failed");
		ret = -EINVAL;
	}

	return (ret == PX4_OK);
}
#endif /* SPEKTRUM_POWER */

int RCInput::custom_command(int argc, char *argv[])
{
#if defined(SPEKTRUM_POWER)
	const char *verb = argv[0];

	if (!strcmp(verb, "bind")) {
		uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
		vehicle_command_s vcmd{};
		vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
		vcmd.timestamp = hrt_absolute_time();
		vehicle_command_pub.publish(vcmd);
		return 0;
	}

#endif /* SPEKTRUM_POWER */

	/* start the FMU if not running */
	if (!is_running()) {
		int ret = RCInput::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RCInput::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);

	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

#if ADC_RC_RSSI_CHANNEL

	if (_analog_rc_rssi_stable) {
		PX4_INFO("vrssi: %dmV", (int)(_analog_rc_rssi_volt * 1000.0f));
	}

#endif

	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);

	if (hrt_elapsed_time(&_rc_in.timestamp) < 1_s) {
		print_message(ORB_ID(input_rc), _rc_in);
	}

	return 0;
}

int
RCInput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the RC input parsing and auto-selecting the method. Supported methods are:
- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS Crossfire (CRSF)

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_input", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

#if defined(SPEKTRUM_POWER)
	PRINT_MODULE_USAGE_COMMAND_DESCR("bind", "Send a DSM bind command (module must be running)");
#endif /* SPEKTRUM_POWER */

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rc_input_main(int argc, char *argv[])
{
	return RCInput::main(argc, argv);
}
