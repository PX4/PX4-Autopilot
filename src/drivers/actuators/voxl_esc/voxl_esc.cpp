/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <inttypes.h>

#include <px4_platform_common/getopt.h>

#include "voxl_esc.hpp"

// future use:
#define MODALAI_PUBLISH_ESC_STATUS	0

const char *_device;

VoxlEsc::VoxlEsc() :
	OutputModuleInterface(MODULE_NAME, px4::serial_port_to_wq(VOXL_ESC_DEFAULT_PORT)),
	_mixing_output{"VOXL_ESC", VOXL_ESC_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false},
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")),
	_battery(1, nullptr, _battery_report_interval, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
	_device = VOXL_ESC_DEFAULT_PORT;

	_mixing_output.setAllFailsafeValues(0);
	_mixing_output.setAllDisarmedValues(0);

	_esc_status.timestamp          = hrt_absolute_time();
	_esc_status.counter            = 0;
	_esc_status.esc_count          = VOXL_ESC_OUTPUT_CHANNELS;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;

	for (unsigned i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
		_esc_status.esc[i].timestamp       = 0;
		_esc_status.esc[i].esc_address     = 0;
		_esc_status.esc[i].esc_rpm         = 0;
		_esc_status.esc[i].esc_state       = 0;
		_esc_status.esc[i].esc_cmdcount    = 0;
		_esc_status.esc[i].esc_voltage     = 0;
		_esc_status.esc[i].esc_current     = 0;
		_esc_status.esc[i].esc_temperature = 0;
		_esc_status.esc[i].esc_errorcount  = 0;
		_esc_status.esc[i].failures        = 0;
		_esc_status.esc[i].esc_power       = 0;
	}

	qc_esc_packet_init(&_fb_packet);

	_fb_idx = 0;
}

VoxlEsc::~VoxlEsc()
{
	_outputs_on = false;

	_uart_port.close();

	perf_free(_cycle_perf);
	perf_free(_output_update_perf);
}

int VoxlEsc::init()
{
	PX4_INFO("VOXL_ESC: Starting VOXL ESC driver");

	/* Getting initial parameter values */
	int ret = update_params();

	if (ret != OK) {
		PX4_ERR("VOXL_ESC: Failed to update params during init");
		return ret;
	}

	print_params();

	//WARING: uart port initialization and device detection does not happen here
	//because init() is called from a different thread from Run(), so fd opened in init() cannot be used in Run()
	//this is an issue (feature?) specific to nuttx where each thread group gets separate set of fds
	//https://cwiki.apache.org/confluence/display/NUTTX/Detaching+File+Descriptors
	//detaching file descriptors is not implemented in the current version of nuttx that px4 uses
	//
	//There is no problem when running on VOXL2, but in order to have the same logical flow on both systems,
	//we will initialize uart and query the device in Run()

	ScheduleNow();

	return 0;
}

int VoxlEsc::device_init()
{
	if (_device_initialized) {
		return 0;
	}

	// Open serial port
	if (!_uart_port.isOpen()) {
		PX4_INFO("VOXL_ESC: Opening UART ESC device %s, baud rate %" PRIi32, _device, _parameters.baud_rate);
#ifndef __PX4_QURT

		//warn user that unless DMA is enabled for UART RX, data can be lost due to high frequency of per char cpu interrupts
		//at least at 2mbit, there are definitely losses, did not test other baud rates to find the cut off
		if (_parameters.baud_rate > 250000) {
			PX4_WARN("VOXL_ESC: Baud rate is too high for non-DMA based UART, this can lead to loss of RX data");
		}

#endif

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

	// Reset output channel values
	memset(&_esc_chans, 0x00, sizeof(_esc_chans));

	//reset the ESC version info before requesting
	for (int esc_id = 0; esc_id < VOXL_ESC_OUTPUT_CHANNELS; ++esc_id) {
		memset(&(_version_info[esc_id]), 0, sizeof(_version_info[esc_id]));
		//_version_info[esc_id].sw_version = 0;  //invalid
		//_version_info[esc_id].hw_version = 0;  //invalid
		_version_info[esc_id].id         = esc_id;
	}

	// Detect ESCs
	PX4_INFO("VOXL_ESC: Detecting ESCs...");
	qc_esc_packet_init(&_fb_packet);

	//request extended version info from each ESC and wait for reply
	for (uint8_t esc_id = 0; esc_id < VOXL_ESC_OUTPUT_CHANNELS; esc_id++) {
		Command cmd;
		cmd.len = qc_esc_create_extended_version_request_packet(esc_id, cmd.buf, sizeof(cmd.buf));

		if (_uart_port.write(cmd.buf, cmd.len) != cmd.len) {
			PX4_ERR("VOXL_ESC: Could not write version request packet to UART port");
			return -1;
		}

		hrt_abstime t_request = hrt_absolute_time();
		hrt_abstime t_timeout = 50000; //50ms timeout for version info response
		bool got_response     = false;

		while ((!got_response) && (hrt_elapsed_time(&t_request) < t_timeout)) {
			px4_usleep(100); //sleep a bit while waiting for ESC to respond

			int nread = _uart_port.read(_read_buf, sizeof(_read_buf));

			for (int i = 0; i < nread; i++) {
				int16_t parse_ret = qc_esc_packet_process_char(_read_buf[i], &_fb_packet);

				if (parse_ret > 0) {
					hrt_abstime response_time = hrt_elapsed_time(&t_request);
					//PX4_INFO("got packet of length %i",ret);
					_rx_packet_count++;
					uint8_t packet_type = qc_esc_packet_get_type(&_fb_packet);
					uint8_t packet_size = qc_esc_packet_get_size(&_fb_packet);

					if (packet_type == ESC_PACKET_TYPE_VERSION_EXT_RESPONSE && packet_size == sizeof(QC_ESC_EXTENDED_VERSION_INFO)) {
						QC_ESC_EXTENDED_VERSION_INFO ver;
						memcpy(&ver, _fb_packet.buffer, packet_size);

						PX4_INFO("VOXL_ESC: \tESC ID     : %i", ver.id);
						PX4_INFO("VOXL_ESC: \tBoard Type : %i: %s", ver.hw_version, board_id_to_name(ver.hw_version));

						uint8_t *u = &ver.unique_id[0];
						PX4_INFO("VOXL_ESC: \tUnique ID  : 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
							 u[11], u[10], u[9], u[8], u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);

						PX4_INFO("VOXL_ESC: \tFirmware   : version %4d, hash %.12s", ver.sw_version, ver.firmware_git_version);
						PX4_INFO("VOXL_ESC: \tBootloader : version %4d, hash %.12s", ver.bootloader_version, ver.bootloader_git_version);
						PX4_INFO("VOXL_ESC: \tReply time : %" PRIu32 "us", (uint32_t)response_time);
						PX4_INFO("VOXL_ESC:");

						if (ver.id == esc_id) {
							memcpy(&_version_info[esc_id], &ver, sizeof(ver));
							got_response = true;
						}
					}
				}
			}
		}

		if (!got_response) {
			PX4_ERR("VOXL_ESC: ESC %d version info response timeout", esc_id);
		}
	}

	//check the SW version of the ESCs
	bool esc_detection_fault = false;

	for (int esc_id = 0; esc_id < VOXL_ESC_OUTPUT_CHANNELS; esc_id++) {
		if (_version_info[esc_id].sw_version == 0) {
			PX4_ERR("VOXL_ESC: ESC ID %d was not detected", esc_id);
			esc_detection_fault = true;
		}
	}

	//check the firmware hashes to make sure they are the same. Firmware hash has 8 chars plus optional "*"
	for (int esc_id = 1; esc_id < VOXL_ESC_OUTPUT_CHANNELS; esc_id++) {
		if (strncmp(_version_info[0].firmware_git_version, _version_info[esc_id].firmware_git_version, 9) != 0) {
			PX4_ERR("VOXL_ESC: ESC %d Firmware hash does not match ESC 0 firmware hash:  (%.12s) != (%.12s)",
				esc_id, _version_info[esc_id].firmware_git_version, _version_info[0].firmware_git_version);
			esc_detection_fault = true;
		}
	}

	//if firmware version is equal or greater than VOXL_ESC_EXT_RPM, ESC packet with extended rpm range is supported. use it
	_extended_rpm      = true;

	for (int esc_id = 0; esc_id < VOXL_ESC_OUTPUT_CHANNELS; esc_id++) {
		if (_version_info[esc_id].sw_version < VOXL_ESC_EXT_RPM) {
			_extended_rpm = false;
		}
	}

	if (esc_detection_fault) {
		PX4_ERR("VOXL_ESC: Critical error during ESC initialization");
		return -1;
	}

	PX4_INFO("VOXL_ESC: Use extened rpm packet : %d", _extended_rpm);

	PX4_INFO("VOXL_ESC: All ESCs successfully detected");

	_device_initialized =  true;

	return 0;
}

int VoxlEsc::load_params(voxl_esc_params_t *params, ch_assign_t *map)
{
	int ret = PX4_OK;

	// initialize out
	for (int i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
		params->function_map[i] = (int)OutputFunction::Disabled;
		params->direction_map[i] = 0;
		params->motor_map[i] = 0;
	}

	param_get(param_find("VOXL_ESC_CONFIG"),  &params->config);
	param_get(param_find("VOXL_ESC_MODE"),    &params->mode);
	param_get(param_find("VOXL_ESC_BAUD"),    &params->baud_rate);

	param_get(param_find("VOXL_ESC_T_PERC"),  &params->turtle_motor_percent);
	param_get(param_find("VOXL_ESC_T_DEAD"),  &params->turtle_motor_deadband);
	param_get(param_find("VOXL_ESC_T_EXPO"),  &params->turtle_motor_expo);
	param_get(param_find("VOXL_ESC_T_MINF"),  &params->turtle_stick_minf);
	param_get(param_find("VOXL_ESC_T_COSP"),  &params->turtle_cosphi);

	param_get(param_find("VOXL_ESC_FUNC1"),  &params->function_map[0]);
	param_get(param_find("VOXL_ESC_FUNC2"),  &params->function_map[1]);
	param_get(param_find("VOXL_ESC_FUNC3"),  &params->function_map[2]);
	param_get(param_find("VOXL_ESC_FUNC4"),  &params->function_map[3]);

	param_get(param_find("VOXL_ESC_SDIR1"),  &params->direction_map[0]);
	param_get(param_find("VOXL_ESC_SDIR2"),  &params->direction_map[1]);
	param_get(param_find("VOXL_ESC_SDIR3"),  &params->direction_map[2]);
	param_get(param_find("VOXL_ESC_SDIR4"),  &params->direction_map[3]);

	param_get(param_find("VOXL_ESC_RPM_MIN"), &params->rpm_min);
	param_get(param_find("VOXL_ESC_RPM_MAX"), &params->rpm_max);

	param_get(param_find("VOXL_ESC_VLOG"),    &params->verbose_logging);
	param_get(param_find("VOXL_ESC_PUB_BST"), &params->publish_battery_status);

	param_get(param_find("VOXL_ESC_T_WARN"), &params->esc_warn_temp_threshold);
	param_get(param_find("VOXL_ESC_T_OVER"), &params->esc_over_temp_threshold);

	if (params->rpm_min >= params->rpm_max) {
		PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_RPM_MIN.  Please verify parameters.");
		params->rpm_min = 0;
		ret = PX4_ERROR;
	}

	if (params->turtle_motor_percent < 0 || params->turtle_motor_percent > 100) {
		PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_T_PERC.  Please verify parameters.");
		params->turtle_motor_percent = 0;
		ret = PX4_ERROR;
	}

	if (params->turtle_motor_deadband < 0 || params->turtle_motor_deadband > 100) {
		PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_T_DEAD.  Please verify parameters.");
		params->turtle_motor_deadband = 0;
		ret = PX4_ERROR;
	}

	if (params->turtle_motor_expo < 0 || params->turtle_motor_expo > 100) {
		PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_T_EXPO.  Please verify parameters.");
		params->turtle_motor_expo = 0;
		ret = PX4_ERROR;
	}

	if (params->turtle_stick_minf < 0.0f || params->turtle_stick_minf > 100.0f) {
		PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_T_MINF.  Please verify parameters.");
		params->turtle_stick_minf = 0.0f;
		ret = PX4_ERROR;
	}

	if (params->turtle_cosphi < 0.0f || params->turtle_cosphi > 100.0f) {
		PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_T_COSP.  Please verify parameters.");
		params->turtle_cosphi = 0.0f;
		ret = PX4_ERROR;
	}

	for (int i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
		if (params->function_map[i] < (int)OutputFunction::Motor1 || params->function_map[i] > (int)OutputFunction::Motor4) {
			PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_FUNCX.  Only supports motors 1-4.  Please verify parameters.");
			params->function_map[i] = 0;
			ret = PX4_ERROR;

		} else {
			//
			// Motor function IDs start at 100, Motor1 = 101, Motor2 = 102...
			// This motor_map array represents ESC IDs 0-3 (matching the silkscreen)
			// This array will hold ESC ID to Motor ID (e.g. motor_map[0] = 1, means ESC ID0 wired to motor 1)
			//
			params->motor_map[i] = (params->function_map[i] - (int)OutputFunction::Motor1) + 1;
		}
	}

	for (int i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
		if (params->motor_map[i] == VOXL_ESC_OUTPUT_DISABLED ||
		    params->motor_map[i] < -(VOXL_ESC_OUTPUT_CHANNELS) ||
		    params->motor_map[i] > VOXL_ESC_OUTPUT_CHANNELS) {
			PX4_ERR("VOXL_ESC: Invalid parameter VOXL_ESC_MOTORX.  Please verify parameters.");
			params->motor_map[i] = 0;
			ret = PX4_ERROR;
		}

		// Keep tabs on motor map for turtle mode where we mix ourselves
		map[i].number = params->motor_map[i];
		map[i].direction = (params->direction_map[i] > 0) ? -1 : 1;
	}

	return ret;
}

int VoxlEsc::task_spawn(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "dv", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_device = argv[myoptind];
			break;

		default:
			break;
		}
	}

	VoxlEsc *instance = new VoxlEsc();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	// This will cause a crash on SLPI DSP
	// delete instance;

	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VoxlEsc::read_response(Command *out_cmd)
{
	px4_usleep(_current_cmd.resp_delay_us);

	int res = _uart_port.read(_read_buf, sizeof(_read_buf));

	if (res > 0) {
		//PX4_INFO("read %i bytes",res);
		if (parse_response(_read_buf, res, out_cmd->print_feedback) < 0) {
			//PX4_ERR("Error parsing response");
			return -1;
		}

	} else {
		//PX4_ERR("Read error: %i", res);
		return -1;
	}

	//_current_cmd.response = false;

	return 0;
}

int VoxlEsc::parse_response(uint8_t *buf, uint8_t len, bool print_feedback)
{
	hrt_abstime tnow = hrt_absolute_time();

	for (int i = 0; i < len; i++) {
		int16_t ret = qc_esc_packet_process_char(buf[i], &_fb_packet);

		if (ret > 0) {
			//PX4_INFO("got packet of length %i",ret);
			_rx_packet_count++;
			uint8_t packet_type = qc_esc_packet_get_type(&_fb_packet);
			uint8_t packet_size = qc_esc_packet_get_size(&_fb_packet);

			if (packet_type == ESC_PACKET_TYPE_FB_RESPONSE && packet_size == sizeof(QC_ESC_FB_RESPONSE_V2)) {
				// PX4_INFO("Got feedback V2 packet!");
				QC_ESC_FB_RESPONSE_V2 fb;
				memcpy(&fb, _fb_packet.buffer, packet_size);

				uint32_t id             = (fb.id_state & 0xF0) >> 4;  //ID of the ESC based on hardware address

				if (id < VOXL_ESC_OUTPUT_CHANNELS) {

					int motor_idx = _output_map[id].number - 1; // mapped motor id.. user defined mapping is 1-4, array is 0-3

					if (print_feedback) {
						uint32_t rpm         = fb.rpm;
						uint32_t power       = fb.power;
						uint32_t voltage     = fb.voltage;
						int32_t  current     = fb.current * 8;
						int32_t  temperature = fb.temperature / 100;
						PX4_INFO("VOXL_ESC: [%" PRId64 "] ID_RAW=%d ID=%d, RPM=%5d, PWR=%3d%%, V=%5dmV, I=%+5dmA, T=%+3dC", tnow, (int)id,
							 motor_idx + 1,
							 (int)rpm, (int)power, (int)voltage, (int)current, (int)temperature);
					}

					_esc_chans[id].rate_meas     = fb.rpm;
					_esc_chans[id].power_applied = fb.power;
					_esc_chans[id].state         = fb.id_state & 0x0F;
					_esc_chans[id].cmd_counter   = fb.cmd_counter;
					_esc_chans[id].voltage       = fb.voltage * 0.001f;
					_esc_chans[id].current       = fb.current * 0.008f;
					_esc_chans[id].temperature   = fb.temperature * 0.01f;
					_esc_chans[id].feedback_time = tnow;

					// also update our internal report for logging
					_esc_status.esc[id].esc_address  = motor_idx + 1; //remapped motor ID
					_esc_status.esc[id].timestamp    = tnow;
					_esc_status.esc[id].esc_rpm      = fb.rpm;
					_esc_status.esc[id].esc_power    = fb.power;
					_esc_status.esc[id].esc_state    = fb.id_state & 0x0F;
					_esc_status.esc[id].esc_cmdcount = fb.cmd_counter;
					_esc_status.esc[id].esc_voltage  = _esc_chans[id].voltage;
					_esc_status.esc[id].esc_current  = _esc_chans[id].current;
					_esc_status.esc[id].failures     = 0; //not implemented

					// this is hacky, but we need to set all 4 to online/armed otherwise commander times out on arming
					_esc_status.esc_online_flags = (1 << _esc_status.esc_count) - 1;
					// this is hacky, but we need to set all 4 to armed otherwise commander times out on arming
					_esc_status.esc_armed_flags = (1 << _esc_status.esc_count) - 1;


					int32_t t = fb.temperature / 100;  //divide by 100 to get deg C and cap for int8

					if (t < -127) { t = -127; }

					if (t > +127) { t = +127; }

					_esc_status.esc[id].esc_temperature = t;

					_esc_status.timestamp = _esc_status.esc[id].timestamp;
					_esc_status.counter++;


					if ((_parameters.esc_over_temp_threshold > 0)
					    && (_esc_status.esc[id].esc_temperature > _parameters.esc_over_temp_threshold)) {
						_esc_status.esc[id].failures |= 1 << (esc_report_s::FAILURE_OVER_ESC_TEMPERATURE);
					}

					//TODO: do we also issue a warning if over-temperature threshold is exceeded?
					if ((_parameters.esc_warn_temp_threshold > 0)
					    && (_esc_status.esc[id].esc_temperature > _parameters.esc_warn_temp_threshold)) {
						_esc_status.esc[id].failures |= 1 << (esc_report_s::FAILURE_WARN_ESC_TEMPERATURE);
					}


					//print ESC status just for debugging
					/*
					PX4_INFO("[%lld] ID=%d, ADDR %d, STATE=%d, RPM=%5d, PWR=%3d%%, V=%.2fdV, I=%.2fA, T=%+3dC, CNT %d, FAIL %d",
						_esc_status.esc[id].timestamp, id, _esc_status.esc[id].esc_address,
						_esc_status.esc[id].esc_state, _esc_status.esc[id].esc_rpm, _esc_status.esc[id].esc_power,
						(double)_esc_status.esc[id].esc_voltage, (double)_esc_status.esc[id].esc_current, _esc_status.esc[id].esc_temperature,
					  _esc_status.esc[id].esc_cmdcount, _esc_status.esc[id].failures);
					*/
				}
			}

			else if (packet_type == ESC_PACKET_TYPE_VERSION_RESPONSE && packet_size == sizeof(QC_ESC_VERSION_INFO)) {
				QC_ESC_VERSION_INFO ver;
				memcpy(&ver, _fb_packet.buffer, packet_size);

				PX4_INFO("VOXL_ESC: ESC ID: %i", ver.id);
				PX4_INFO("VOXL_ESC: HW Version: %i", ver.hw_version);
				PX4_INFO("VOXL_ESC: SW Version: %i", ver.sw_version);
				PX4_INFO("VOXL_ESC: Unique ID: %i", (int)ver.unique_id);

			} else if (packet_type == ESC_PACKET_TYPE_VERSION_EXT_RESPONSE && packet_size == sizeof(QC_ESC_EXTENDED_VERSION_INFO)) {
				QC_ESC_EXTENDED_VERSION_INFO ver;
				memcpy(&ver, _fb_packet.buffer, packet_size);
				PX4_INFO("VOXL_ESC: \tESC ID     : %i", ver.id);
				PX4_INFO("VOXL_ESC: \tBoard      : %i", ver.hw_version);
				PX4_INFO("VOXL_ESC: \tSW Version : %i", ver.sw_version);

				uint8_t *u = &ver.unique_id[0];
				PX4_INFO("VOXL_ESC: \tUnique ID  : 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					 u[11], u[10], u[9], u[8], u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);

				PX4_INFO("VOXL_ESC: \tFirmware   : version %4d, hash %.12s", ver.sw_version, ver.firmware_git_version);
				PX4_INFO("VOXL_ESC: \tBootloader : version %4d, hash %.12s", ver.bootloader_version, ver.bootloader_git_version);

			} else if (packet_type == ESC_PACKET_TYPE_FB_POWER_STATUS && packet_size == sizeof(QC_ESC_FB_POWER_STATUS)) {
				QC_ESC_FB_POWER_STATUS packet;
				memcpy(&packet, _fb_packet.buffer, packet_size);

				float voltage = packet.voltage * 0.001f; // Voltage is reported at 1 mV resolution
				float current = packet.current * 0.008f; // Total current is reported at 8mA resolution

				// Limit the frequency of battery status reports
				if (_parameters.publish_battery_status) {
					_battery.setConnected(true);
					_battery.updateVoltage(voltage);
					_battery.updateCurrent(current);

					hrt_abstime current_time = hrt_absolute_time();

					if ((current_time - _last_battery_report_time) >= _battery_report_interval) {
						_last_battery_report_time = current_time;
						_battery.updateAndPublishBatteryStatus(current_time);
					}
				}

			}

		} else { //parser error
			switch (ret) {
			case ESC_ERROR_BAD_CHECKSUM:
				_rx_crc_error_count++;
				// PX4_INFO("BAD ESC packet checksum");
				break;

			case ESC_ERROR_BAD_LENGTH:
				// PX4_INFO("BAD ESC packet length");
				break;
			}
		}
	}

	return 0;
}

int VoxlEsc::check_for_esc_timeout()
{
	hrt_abstime tnow = hrt_absolute_time();

	for (int i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
		// PX4 motor indexed user defined mapping is 1-4, we want to use in bitmask (0-3)
		uint8_t motor_idx = _output_map[i].number - 1;

		if (motor_idx < VOXL_ESC_OUTPUT_CHANNELS) {
			// we are using PX4 motor index in the bitmask
			if (_esc_status.esc_online_flags & (1 << motor_idx)) {
				// using index i here for esc_chans enumeration stored in ESC ID order
				if ((tnow - _esc_chans[i].feedback_time) > VOXL_ESC_DISCONNECT_TIMEOUT_US) {
					// stale data, assume offline and clear armed
					_esc_status.esc_online_flags &= ~(1 << motor_idx);
					_esc_status.esc_armed_flags &= ~(1 << motor_idx);
				}
			}
		}
	}

	return 0;

}

int VoxlEsc::send_cmd_thread_safe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}



int VoxlEsc::custom_command(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	Command  cmd;
	uint8_t  esc_id   = 255;
	uint8_t  period   = 0;
	uint8_t  duration = 0;
	uint8_t  power    = 0;
	uint16_t led_mask = 0;
	int16_t  rate     = 0;

	uint32_t repeat_count    = 100;
	uint32_t repeat_delay_us = 10000;

	if (argc < 3) {
		return print_usage("unknown command");
	}

	const char *verb = argv[argc - 1];

	/* start the driver if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return VoxlEsc::task_spawn(argc, argv);
		}
	}

	if (!is_running()) {
		PX4_INFO("VOXL_ESC:Not running");
		return -1;

	}

	while ((ch = px4_getopt(argc, argv, "i:p:d:v:l:n:r:t:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			esc_id = atoi(myoptarg);
			break;

		case 'p':
			period = atoi(myoptarg);
			break;

		case 'd':
			duration = atoi(myoptarg);
			break;

		case 'v':
			power = atoi(myoptarg);
			break;

		case 'l':
			led_mask = atoi(myoptarg);
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

	if (!strcmp(verb, "reset")) {
		if (esc_id < VOXL_ESC_OUTPUT_CHANNELS) {
			PX4_INFO("VOXL_ESC: Reset ESC: %i", esc_id);
			cmd.len = qc_esc_create_reset_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->send_cmd_thread_safe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "version")) {
		if (esc_id < VOXL_ESC_OUTPUT_CHANNELS) {
			PX4_INFO("VOXL_ESC: Request version for ESC: %i", esc_id);
			cmd.len = qc_esc_create_version_request_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = true;
			cmd.resp_delay_us = 2000;
			return get_instance()->send_cmd_thread_safe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "version-ext")) {
		if (esc_id < VOXL_ESC_OUTPUT_CHANNELS) {
			PX4_INFO("VOXL_ESC: Request extended version for ESC: %i", esc_id);
			cmd.len = qc_esc_create_extended_version_request_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = true;
			cmd.resp_delay_us = 5000;
			return get_instance()->send_cmd_thread_safe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "tone")) {
		if (esc_id < VOXL_ESC_OUTPUT_CHANNELS) {
			PX4_INFO("VOXL_ESC: Request tone for ESC mask: %i", esc_id);
			cmd.len = qc_esc_create_sound_packet(period, duration, power, esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->send_cmd_thread_safe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "led")) {
		if (led_mask <= 0x0FFF) {
			get_instance()->_led_rsc.test = true;
			get_instance()->_led_rsc.breath_en = false;
			PX4_INFO("VOXL_ESC: Request LED control for ESCs with mask: %i", led_mask);

			get_instance()->_esc_chans[0].led = (led_mask & 0x0007);
			get_instance()->_esc_chans[1].led = (led_mask & 0x0038) >> 3;
			get_instance()->_esc_chans[2].led = (led_mask & 0x01C0) >> 6;
			get_instance()->_esc_chans[3].led = (led_mask & 0x0E00) >> 9;
			return 0;

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	}  else if (!strcmp(verb, "rpm")) {
		if (esc_id < VOXL_ESC_OUTPUT_CHANNELS) {
			PX4_INFO("VOXL_ESC: Request RPM for ESC ID: %i - RPM: %i", esc_id, rate);
			int16_t rate_req[VOXL_ESC_OUTPUT_CHANNELS] = {0, 0, 0, 0};
			uint8_t id_fb = 0;

			if (esc_id == 0xFF) {  //WARNING: this condition is not possible due to check 'if (esc_id < VOXL_ESC_OUTPUT_CHANNELS)'.
				rate_req[0] = rate;
				rate_req[1] = rate;
				rate_req[2] = rate;
				rate_req[3] = rate;

			} else {
				rate_req[esc_id] = rate;
				id_fb = esc_id;
			}

			cmd.len = qc_esc_create_rpm_packet4_fb(rate_req[0],
							       rate_req[1],
							       rate_req[2],
							       rate_req[3],
							       0,
							       0,
							       0,
							       0,
							       id_fb,
							       cmd.buf,
							       sizeof(cmd.buf),
							       get_instance()->_extended_rpm);

			cmd.response        = true;
			cmd.repeats         = repeat_count;
			cmd.resp_delay_us   = 0;
			cmd.repeat_delay_us = repeat_delay_us;
			cmd.print_feedback  = true;

			PX4_INFO("VOXL_ESC: Feedback id debug: %i", id_fb);
			PX4_INFO("VOXL_ESC: Sending UART ESC RPM command %i", rate);

			return get_instance()->send_cmd_thread_safe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "pwm")) {
		if (esc_id < VOXL_ESC_OUTPUT_CHANNELS) {
			PX4_INFO("VOXL_ESC: Request PWM for ESC ID: %i - PWM: %i", esc_id, rate);
			int16_t rate_req[VOXL_ESC_OUTPUT_CHANNELS] = {0, 0, 0, 0};
			uint8_t id_fb = 0;

			if (esc_id == 0xFF) {  //WARNING: this condition is not possible due to check 'if (esc_id < VOXL_ESC_OUTPUT_CHANNELS)'.
				rate_req[0] = rate;
				rate_req[1] = rate;
				rate_req[2] = rate;
				rate_req[3] = rate;

			} else {
				rate_req[esc_id] = rate;
				id_fb = esc_id;
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

			PX4_INFO("VOXL_ESC: Feedback id debug: %i", id_fb);
			PX4_INFO("VOXL_ESC: Sending UART ESC power command %i", rate);

			return get_instance()->send_cmd_thread_safe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}
	}

	return print_usage("unknown command");
}

int VoxlEsc::update_params()
{
	int ret = PX4_ERROR;

	updateParams();
	ret = load_params(&_parameters, (ch_assign_t *)&_output_map);

	if (ret == PX4_OK) {
		_mixing_output.setAllDisarmedValues(0);
		_mixing_output.setAllFailsafeValues(0);
		_mixing_output.setAllMinValues(_parameters.rpm_min);
		_mixing_output.setAllMaxValues(_parameters.rpm_max);

		_rpm_fullscale = _parameters.rpm_max - _parameters.rpm_min;
	}

	return ret;
}

void VoxlEsc::update_leds(vehicle_control_mode_s mode, led_control_s control)
{
	int i = 0;
	uint8_t led_mask = _led_rsc.led_mask;

	if (_led_rsc.test) {
		return;
	}

	/*
	 * TODO - this is just a simple approach to get started.
	 *
	 */
	if (mode.timestamp != _led_rsc.mode.timestamp) {
		_led_rsc.mode = mode;
	}

	if (control.timestamp != _led_rsc.control.timestamp) {
		_led_rsc.control = control;

		switch (_led_rsc.control.color) {
		case led_control_s::COLOR_RED:
			led_mask = QC_ESC_LED_RED_ON;
			break;

		case led_control_s::COLOR_GREEN:
			led_mask = QC_ESC_LED_GREEN_ON;
			break;

		case led_control_s::COLOR_BLUE:
			led_mask = QC_ESC_LED_BLUE_ON;
			break;

		case led_control_s::COLOR_WHITE:
			led_mask = QC_ESC_LED_RED_ON | QC_ESC_LED_GREEN_ON | QC_ESC_LED_BLUE_ON;
			break;

		case led_control_s::COLOR_OFF:
			led_mask = 0;
			break;
		}

		_led_rsc.breath_en = false;

		switch (_led_rsc.control.mode) {
		case led_control_s::MODE_OFF:
			break;

		case led_control_s::MODE_ON:
			break;

		case led_control_s::MODE_DISABLED:
			break;

		case led_control_s::MODE_BLINK_SLOW:
			break;

		case led_control_s::MODE_BLINK_NORMAL:
			break;

		case led_control_s::MODE_BLINK_FAST:
			break;

		case led_control_s::MODE_BREATHE:
			_led_rsc.breath_en = true;
			_led_rsc.breath_counter = 0;
			break;

		case led_control_s::MODE_FLASH:
			break;

		default:
			break;
		}

		_led_rsc.led_mask = led_mask;
	}

	if (_led_rsc.mode.flag_armed) {
		led_mask = QC_ESC_LED_BLUE_ON;

		if (_led_rsc.mode.flag_control_position_enabled) {
			led_mask = QC_ESC_LED_GREEN_ON;

		} else if (_led_rsc.mode.flag_control_offboard_enabled) {
			led_mask = QC_ESC_LED_RED_ON;
		}

		_led_rsc.led_mask = led_mask;
	}

	if (_led_rsc.breath_en) {
		/* 8 bit counter for a decent blink visual effect for
		 * 'breathing' use case
		 */
		if ((_led_rsc.breath_counter += 8) < 128) {
			led_mask = 0;
		}
	}

	for (i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
		_esc_chans[i].led = led_mask;
	}
}

void VoxlEsc::mix_turtle_mode(uint16_t outputs[MAX_ACTUATORS])
{
	bool use_pitch = true;
	bool use_roll  = true;
	bool use_yaw   = true;
	bool isolate   = false;

	const float flip_pwr_mult = 1.0f - ((float)_parameters.turtle_motor_expo / 100.0f);

	// Sitck deflection
	const float stick_def_r_abs = fabsf(_manual_control_setpoint.roll);
	const float stick_def_p_abs = fabsf(_manual_control_setpoint.pitch);
	const float stick_def_y_abs = fabsf(_manual_control_setpoint.yaw);

	const float stick_def_p_expo = flip_pwr_mult * stick_def_p_abs + powf(stick_def_p_abs,
				       3.0) * (1 - flip_pwr_mult);
	const float stick_def_r_expo  = flip_pwr_mult * stick_def_r_abs + powf(stick_def_r_abs,
					3.0) * (1 - flip_pwr_mult);
	const float stick_def_y_expo  = flip_pwr_mult * stick_def_y_abs + powf(stick_def_y_abs,
					3.0) * (1 - flip_pwr_mult);

	float sign_r = _manual_control_setpoint.roll < 0 ? 1 : -1;
	float sign_p = _manual_control_setpoint.pitch < 0 ? 1 : -1;
	float sign_y = _manual_control_setpoint.yaw < 0 ? 1 : -1;

	float stick_def_len      = sqrtf(powf(stick_def_p_abs, 2.0) + powf(stick_def_r_abs, 2.0));
	float stick_def_expo_len = sqrtf(powf(stick_def_p_expo, 2.0) + powf(stick_def_r_expo, 2.0));

	// If yaw is the dominant, disable pitch and roll
	if (stick_def_y_abs > math::max(stick_def_p_abs, stick_def_r_abs)) {
		stick_def_len = stick_def_y_abs;
		stick_def_expo_len = stick_def_y_expo;
		sign_r = 0;
		sign_p = 0;
		use_pitch = false;
		use_roll = false;
	}

	// If pitch/roll dominant, disable yaw
	else {
		sign_y = 0;
		use_yaw = false;
	}

	const float cos_phi = (stick_def_len > 0) ? (stick_def_p_abs + stick_def_r_abs) / (sqrtf(
				      2.0f) * stick_def_len) : 0;

	// TODO: this is hardcoded in betaflight...
	const float cos_thresh = sqrtf(3.0f) / 2.0f; // cos(PI/6.0f)

	// This cos_phi values is 1.0 when sticks are in the far corners, which means we are trying to select a single motor
	if (cos_phi > _parameters.turtle_cosphi) {
		isolate = true;
		use_pitch = false;
		use_roll = false;
	}

	// When cos_phi is less than cos_thresh, the user is in a narrow slot on the pitch or roll axis
	else if (cos_phi < cos_thresh) {
		// Enforce either roll or pitch exclusively, if not on diagonal
		if (stick_def_r_abs > stick_def_p_abs) {
			sign_p = 0;
			use_pitch = false;

		} else if (stick_def_r_abs < stick_def_p_abs) {
			sign_r = 0;
			use_roll = false;
		}
	}

	const float crash_flip_stick_min_expo = flip_pwr_mult *  _parameters.turtle_stick_minf + powf(
			_parameters.turtle_stick_minf, 3.0) * (1 - flip_pwr_mult);
	const float flip_stick_range = 1.0f - crash_flip_stick_min_expo;
	const float flip_power = math::max(0.0f, stick_def_expo_len - crash_flip_stick_min_expo) / flip_stick_range;

	/* At this point, we are switching on what PX4 motor we want to talk to */
	for (unsigned i = 0; i < 4; i++) {
		outputs[i] = 0;

		float motor_output_normalised = math::min(1.0f, flip_power);
		float motor_output = _rpm_turtle_min + motor_output_normalised * _parameters.rpm_max * ((
					     float)_parameters.turtle_motor_percent / 100.f);

		// Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
		motor_output = (motor_output < _rpm_turtle_min + _parameters.turtle_motor_deadband) ? 0.0f :
			       (motor_output - _parameters.turtle_motor_deadband);

		// using the output map here for clarity as PX4 motors are 1-4
		switch (_output_map[i].number) {
		/* PX4 motor 1 - front right */
		case 1:
			if (isolate && sign_p < 0 && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p < 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y > 0) {
				outputs[i] = motor_output;
			}

			break;

		/* PX4 motor 2 - rear left */
		case 2:
			if (isolate && sign_p > 0 && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p > 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y > 0) {
				outputs[i] = motor_output;
			}

			break;

		/* PX4 motor 3 - front left */
		case 3:
			if (isolate && sign_p < 0 && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p < 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y < 0) {
				outputs[i] = motor_output;
			}

			break;

		/* PX4 motor 4 - rear right */
		case 4:
			if (isolate && sign_p > 0 && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p > 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y < 0) {
				outputs[i] = motor_output;
			}

			break;
		}
	}

	/*
		static int filter = 0;
		if(filter++ > 32){
			printf("map: %.2f %.2f %.2f %.2f - exp: %.2f %.2f %.2f - deflect: %.2f %.2f - sign: %.2f %.2f %.2f - outputs: %.2f %.2f %.2f %.2f\n",
				(double)_output_map[0].number,(double)_output_map[1].number,(double)_output_map[2].number,(double)_output_map[3].number,
				(double)stick_def_p_expo, (double)stick_def_r_expo,(double)stick_def_y_expo,
				(double)stick_def_len, (double)stick_def_expo_len,
				(double)sign_p, (double)sign_r, (double)sign_y,
				(double)outputs[0], (double)outputs[1],(double)outputs[2],(double)outputs[3]);
			filter = 0;
		}
	*/

}

/* OutputModuleInterface */
bool VoxlEsc::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			    unsigned num_outputs, unsigned num_control_groups_updated)
{
	//in Run() we call _mixing_output.update(), which calls MixingOutput::limitAndUpdateOutputs which calls _interface.updateOutputs (this function)
	//So, if Run() is blocked by a custom command, this function will not be called until Run is running again

	if (num_outputs != VOXL_ESC_OUTPUT_CHANNELS) {
		return false;
	}

	// don't use mixed values... recompute now.
	if (_turtle_mode_en) {
		mix_turtle_mode(outputs);
	}

	for (int i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
		if (!_outputs_on || stop_motors) {
			_esc_chans[i].rate_req = 0;

		} else {
			if (_extended_rpm) {
				if (outputs[i] > VOXL_ESC_RPM_MAX_EXT) { outputs[i] = VOXL_ESC_RPM_MAX_EXT; }

			} else {
				if (outputs[i] > VOXL_ESC_RPM_MAX) { outputs[i] = VOXL_ESC_RPM_MAX; }
			}

			if (!_turtle_mode_en) {
				_esc_chans[i].rate_req = outputs[i] * _output_map[i].direction;

			} else {
				// mapping updated in mixTurtleMode, no remap needed here, but reverse direction
				_esc_chans[i].rate_req = outputs[i] * _output_map[i].direction * (-1);
			}
		}
	}


	Command cmd;
	cmd.len = qc_esc_create_rpm_packet4_fb(_esc_chans[0].rate_req,
					       _esc_chans[1].rate_req,
					       _esc_chans[2].rate_req,
					       _esc_chans[3].rate_req,
					       _esc_chans[0].led,
					       _esc_chans[1].led,
					       _esc_chans[2].led,
					       _esc_chans[3].led,
					       _fb_idx,
					       cmd.buf,
					       sizeof(cmd.buf),
					       _extended_rpm);

	if (_uart_port.write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("VOXL_ESC: Failed to send packet");
		return false;
	}

	// increment ESC id from which to request feedback in round robin order
	_fb_idx = (_fb_idx + 1) % VOXL_ESC_OUTPUT_CHANNELS;


	/*
	 * Here we read and parse response from ESCs. Since the latest command has just been sent out,
	 * the response packet we may read here is probabaly from previous iteration, but it is totally ok.
	 * uart_read is non-blocking and we will just parse whatever bytes came in up until this point
	 */

	int res = _uart_port.read(_read_buf, sizeof(_read_buf));

	if (res > 0) {
		parse_response(_read_buf, res, false);
	}

	/* handle loss of comms / disconnect */
	// TODO - enable after CRC issues in feedback are addressed
	//check_for_esc_timeout();

	// publish the actual command that we sent and the feedback received
	if (_parameters.verbose_logging) {
		actuator_outputs_s actuator_outputs{};
		actuator_outputs.noutputs = num_outputs;

		for (size_t i = 0; i < num_outputs; ++i) {
			actuator_outputs.output[i] = _esc_chans[i].rate_req;
		}

		actuator_outputs.timestamp = hrt_absolute_time();

		_outputs_debug_pub.publish(actuator_outputs);

	}

	_esc_status_pub.publish(_esc_status);

	// If any extra external modal io data has been received then
	// send it over as well
	while (_voxl2_io_data_sub.updated()) {
		buffer128_s io_data{};
		_voxl2_io_data_sub.copy(&io_data);

		// PX4_INFO("Got Modal IO data: %u bytes", io_data.len);
		// PX4_INFO("   0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x",
		// 		 io_data.data[0], io_data.data[1], io_data.data[2], io_data.data[3],
		// 		 io_data.data[4], io_data.data[5], io_data.data[6], io_data.data[7]);
		if (_uart_port.write(io_data.data, io_data.len) != io_data.len) {
			PX4_ERR("VOXL_ESC: Failed to send modal io data to esc");
			return false;
		}
	}

	perf_count(_output_update_perf);

	return true;
}


void VoxlEsc::Run()
{
	if (should_exit()) {
		PX4_ERR("VOXL_ESC: Stopping the module");
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	//check to see if we need to open uart port and query the device
	//see comment in init() regarding why we do not initialize the device there

	int retries_left = VOXL_ESC_NUM_INIT_RETRIES;

	while ((!_device_initialized) && (retries_left > 0)) {
		retries_left--;
		int dev_init_ret = device_init();

		if (dev_init_ret != 0) {
			PX4_WARN("VOXL_ESC: Failed to initialize device, retries left %d", retries_left);
		}
	}

	if (!_device_initialized) {
		PX4_ERR("VOXL_ESC: Failed to initialize device, exiting the module");
		ScheduleClear();
		_mixing_output.unregister();
		exit_and_cleanup();
		return;
	}

	_mixing_output.update();  //calls MixingOutput::limitAndUpdateOutputs which calls updateOutputs in this module

	/* update output status if armed */
	_outputs_on = _mixing_output.armed().armed;

	/* check for parameter updates */
	if (!_outputs_on && _parameter_update_sub.updated()) {
		/* clear update */
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		/* update parameters from storage */
		update_params();
	}

	vehicle_control_mode_s vehicle_control_mode{};

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);
		update_leds(vehicle_control_mode, _led_rsc.control);
	}

	led_control_s led_control{};

	if (_led_update_sub.updated()) {
		_led_update_sub.copy(&led_control);
		update_leds(_led_rsc.mode, led_control);
	}

	/* breathing requires continuous updates */
	if (_led_rsc.breath_en) {
		update_leds(_led_rsc.mode, _led_rsc.control);
	}

	if (_parameters.mode > 0) {
		/* if turtle mode enabled, we go straight to the sticks, no mix */
		if (_manual_control_setpoint_sub.updated()) {

			_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

			if (!_outputs_on) {

				float setpoint = VOXL_ESC_MODE_DISABLED_SETPOINT;

				if (_parameters.mode == VOXL_ESC_MODE_TURTLE_AUX1) {
					setpoint = _manual_control_setpoint.aux1;

				} else if (_parameters.mode == VOXL_ESC_MODE_TURTLE_AUX2) {
					setpoint = _manual_control_setpoint.aux2;
				}

				if (setpoint > VOXL_ESC_MODE_THRESHOLD) {
					_turtle_mode_en = true;

				} else {
					_turtle_mode_en = false;
				}
			}
		}

	}

	if (!_outputs_on) {
		if (_actuator_test_sub.updated()) {
			// values are set in ActuatorTest::update, we just need to enable outputs to let them through
			_outputs_on = true;
		}
	}

	/* Don't process commands if outputs on */
	if (!_outputs_on) {
		if (_current_cmd.valid()) {
			//PX4_INFO("sending %d commands with delay %dus",_current_cmd.repeats,_current_cmd.repeat_delay_us);
			_uart_port.flush();

			do {
				//PX4_INFO("CMDs left %d",_current_cmd.repeats);
				if (_uart_port.write(_current_cmd.buf, _current_cmd.len) == _current_cmd.len) {
					if (_current_cmd.repeats == 0) {
						_current_cmd.clear();
					}

					if (_current_cmd.response) {
						if (read_response(&_current_cmd) == 0) {
							_esc_status_pub.publish(_esc_status);
						}
					}

				} else {
					if (_current_cmd.retries == 0) {
						_current_cmd.clear();
						PX4_ERR("VOXL_ESC: Failed to send command, errno: %i", errno);

					} else {
						_current_cmd.retries--;
						PX4_ERR("VOXL_ESC: Failed to send command, errno: %i", errno);
					}
				}

				px4_usleep(_current_cmd.repeat_delay_us);
			} while (_current_cmd.repeats-- > 0);

			PX4_INFO("VOXL_ESC: RX packet count: %d", (int)_rx_packet_count);
			PX4_INFO("VOXL_ESC: CRC error count: %d", (int)_rx_crc_error_count);

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


int VoxlEsc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for...

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ todo

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("voxl_esc", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Send reset request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("version", "Send version request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("version-ext", "Send extended version request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("rpm", "Closed-Loop RPM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, -32768, 32768, "RPM, -32,768 to 32,768", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 100, 0, 1<<31, "Command repeat count, 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 10000, 0, 1<<31, "Delay between repeated commands (microseconds), 0 to INT_MAX", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "Open-Loop PWM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 800, "Duty Cycle value, 0 to 800", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 100, 0, 1<<31, "Command repeat count, 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 10000, 0, 1<<31, "Delay between repeated commands (microseconds), 0 to INT_MAX", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("tone", "Send tone generation request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 255, "Period of sound, inverse frequency, 0-255", false);
	PRINT_MODULE_USAGE_PARAM_INT('d', 0, 0, 255, "Duration of the sound, 0-255, 1LSB = 13ms", false);
	PRINT_MODULE_USAGE_PARAM_INT('v', 0, 0, 100, "Power (volume) of sound, 0-100", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "Send LED control request");
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, 4095, "Bitmask 0x0FFF (12 bits) - ESC0 (RGB) ESC1 (RGB) ESC2 (RGB) ESC3 (RGB)", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void VoxlEsc::print_params()
{
	PX4_INFO("Params: VOXL_ESC_CONFIG: %" PRId32, _parameters.config);
	PX4_INFO("Params: VOXL_ESC_MODE: %" PRId32, _parameters.mode);
	PX4_INFO("Params: VOXL_ESC_BAUD: %" PRId32, _parameters.baud_rate);

	PX4_INFO("Params: VOXL_ESC_FUNC1: %" PRId32, _parameters.function_map[0]);
	PX4_INFO("Params: VOXL_ESC_FUNC2: %" PRId32, _parameters.function_map[1]);
	PX4_INFO("Params: VOXL_ESC_FUNC3: %" PRId32, _parameters.function_map[2]);
	PX4_INFO("Params: VOXL_ESC_FUNC4: %" PRId32, _parameters.function_map[3]);

	PX4_INFO("Params: VOXL_ESC_SDIR1: %" PRId32, _parameters.direction_map[0]);
	PX4_INFO("Params: VOXL_ESC_SDIR2: %" PRId32, _parameters.direction_map[1]);
	PX4_INFO("Params: VOXL_ESC_SDIR3: %" PRId32, _parameters.direction_map[2]);
	PX4_INFO("Params: VOXL_ESC_SDIR4: %" PRId32, _parameters.direction_map[3]);

	PX4_INFO("Params: VOXL_ESC_RPM_MIN: %" PRId32, _parameters.rpm_min);
	PX4_INFO("Params: VOXL_ESC_RPM_MAX: %" PRId32, _parameters.rpm_max);

	PX4_INFO("Params: VOXL_ESC_T_PERC: %" PRId32, _parameters.turtle_motor_percent);
	PX4_INFO("Params: VOXL_ESC_T_DEAD: %" PRId32, _parameters.turtle_motor_deadband);
	PX4_INFO("Params: VOXL_ESC_T_EXPO: %" PRId32, _parameters.turtle_motor_expo);
	PX4_INFO("Params: VOXL_ESC_T_MINF: %f",       (double)_parameters.turtle_stick_minf);
	PX4_INFO("Params: VOXL_ESC_T_COSP: %f",       (double)_parameters.turtle_cosphi);

	PX4_INFO("Params: VOXL_ESC_VLOG: %" PRId32,    _parameters.verbose_logging);
	PX4_INFO("Params: VOXL_ESC_PUB_BST: %" PRId32, _parameters.publish_battery_status);
	
	PX4_INFO("Params: VOXL_ESC_T_WARN: %" PRId32, _parameters.esc_warn_temp_threshold);
	PX4_INFO("Params: VOXL_ESC_T_OVER: %" PRId32, _parameters.esc_over_temp_threshold);
}

int VoxlEsc::print_status()
{
	PX4_INFO("Max update rate: %i Hz", _current_update_rate);
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	PX4_INFO("UART port: %s", _device);
	PX4_INFO("UART open: %s", _uart_port.isOpen() ? "yes" : "no");

	PX4_INFO("");
	print_params();
	PX4_INFO("");

	for( int i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++){
		PX4_INFO("-- ID: %i", i);
		PX4_INFO("   Motor:           %i", _output_map[i].number);
		PX4_INFO("   Direction:       %i", _output_map[i].direction);
		PX4_INFO("   State:           %i", _esc_chans[i].state);
		PX4_INFO("   Requested:       %" PRIi32 " RPM", _esc_chans[i].rate_req);
		PX4_INFO("   Measured:        %" PRIi32 " RPM", _esc_chans[i].rate_meas);
		PX4_INFO("   Command Counter: %i", _esc_chans[i].cmd_counter);
		PX4_INFO("   Voltage:         %f VDC", (double)_esc_chans[i].voltage);
		PX4_INFO("");
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_output_update_perf);

	_mixing_output.printStatus();

	return 0;
}

const char * VoxlEsc::board_id_to_name(int board_id)
{
	switch(board_id){
		case 31: return "ModalAi 4-in-1 ESC V2 RevB (M0049)";
		case 32: return "Blheli32 4-in-1 ESC Type A (Tmotor F55A PRO F051)";
		case 33: return "Blheli32 4-in-1 ESC Type B (Tmotor F55A PRO G071)";
		case 34: return "ModalAi 4-in-1 ESC (M0117-1)";
		case 35: return "ModalAi I/O Expander (M0065)";
		case 36: return "ModalAi 4-in-1 ESC (M0117-3)";
		case 37: return "ModalAi 4-in-1 ESC (M0134-1)";
		case 38: return "ModalAi 4-in-1 ESC (M0134-3)";
		case 39: return "ModalAi 4-in-1 ESC (M0129-1)";
		case 40: return "ModalAi 4-in-1 ESC (M0129-3)";
		case 41: return "ModalAi 4-in-1 ESC (M0134-6)";
		case 42: return "ModalAi 4-in-1 ESC (M0138-1)";
		default: return "Unknown Board";
	}
}

extern "C" __EXPORT int voxl_esc_main(int argc, char *argv[]);

int voxl_esc_main(int argc, char *argv[])
{
	return VoxlEsc::main(argc, argv);
}
