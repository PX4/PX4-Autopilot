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

#include <px4_platform_common/getopt.h>

#include <px4_platform_common/sem.hpp>

#include "modalai_esc.hpp"
#include "modalai_esc_serial.hpp"

#define MODALAI_ESC_DEVICE_PATH 	"/dev/uart_esc"
#define MODALAI_ESC_DEFAULT_PORT 	"/dev/ttyS1"
#define MODALAI_ESC_VOXL_PORT     "/dev/ttyS4"

//TODO: make this a param!!!
#define MODALAI_PUBLISH_ESC_STATUS 1

const char *_device;

ModalaiEsc::ModalaiEsc() :
	CDev(MODALAI_ESC_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval"))
{
	_device = MODALAI_ESC_DEFAULT_PORT;

	_mixing_output.setAllFailsafeValues(0);
	_mixing_output.setAllDisarmedValues(0);

	_esc_status.timestamp          = hrt_absolute_time();
	_esc_status.counter            = 0;
	_esc_status.esc_count          = MODALAI_ESC_OUTPUT_CHANNELS;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;

	for (unsigned i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
		_esc_status.esc[i].timestamp       = 0;
		_esc_status.esc[i].esc_address     = 0;
		_esc_status.esc[i].esc_rpm         = 0;
		_esc_status.esc[i].esc_state       = 0;
		//_esc_status.esc[i].esc_cmdcount    = 0;
		_esc_status.esc[i].esc_voltage     = 0;
		_esc_status.esc[i].esc_current     = 0;
		_esc_status.esc[i].esc_temperature = 0;
		_esc_status.esc[i].esc_errorcount  = 0;
		_esc_status.esc[i].failures        = 0;
	}

	qc_esc_packet_init(&_fb_packet);
	qc_esc_packet_init(&_uart_bridge_packet);

	_fb_idx = 0;
}

ModalaiEsc::~ModalaiEsc()
{
	_outputs_on = false;

	if (_uart_port) {
		_uart_port->uart_close();
		_uart_port = nullptr;
	}

	if (_uart_port_bridge) {
		_uart_port_bridge->uart_close();
		_uart_port_bridge = nullptr;
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_output_update_perf);
}

int ModalaiEsc::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(MODALAI_ESC_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	/* Getting initial parameter values */
	ret = update_params();

	if (ret != OK) {
		return ret;
	}

	_uart_port = new ModalaiEscSerial();
	_uart_port_bridge = new ModalaiEscSerial();

	//get_instance()->ScheduleOnInterval(10000); //100hz

	ScheduleNow();

	return 0;
}

int ModalaiEsc::load_params(uart_esc_params_t *params, ch_assign_t *map)
{
	int ret = PX4_OK;

	param_get(param_find("UART_ESC_CONFIG"),  &params->config);
	param_get(param_find("UART_ESC_MODE"),    &params->mode);
	param_get(param_find("UART_ESC_DEAD1"),   &params->dead_zone_1);
	param_get(param_find("UART_ESC_DEAD2"),   &params->dead_zone_2);
	param_get(param_find("UART_ESC_BAUD"),    &params->baud_rate);
	param_get(param_find("UART_ESC_MOTOR1"),  &params->motor_map[0]);
	param_get(param_find("UART_ESC_MOTOR2"),  &params->motor_map[1]);
	param_get(param_find("UART_ESC_MOTOR3"),  &params->motor_map[2]);
	param_get(param_find("UART_ESC_MOTOR4"),  &params->motor_map[3]);
	param_get(param_find("UART_ESC_RPM_MIN"), &params->rpm_min);
	param_get(param_find("UART_ESC_RPM_MAX"), &params->rpm_max);

	if (params->rpm_min >= params->rpm_max) {
		PX4_ERR("Invalid parameter UART_ESC_RPM_MIN.  Please verify parameters.");
		params->rpm_min = 0;
		ret = PX4_ERROR;
	}

	//                      Example, PX4 Motor 1
	//                      X = don't activate
	//        [setpoint.x]
	//                 [1.0]
	//             '   |
	//             '   |
	//             '   |
	//             '   |          (ACTIVATE)
	//             '   |
	// DEADZONE_1  + - + -  -  - +
	//              X X|X X X X X'
	// DEADZONE_2 - X X+X X X X X+
	//              X X|X X X X X'
	//       [0.0]-+---+---+-----+---------------- [1.0] [setpoint.y]
	//            / X X|X X X X X'  (ACTIVATE)
	//           /  X X|X X X X X'
	//          /    [0.0] +  -   -    -     -   -
	//  -(DEADZONE_2)       \ DEADZONE_2
	//

	if ((params->dead_zone_1 < MODALAI_ESC_MODE_DEAD_ZONE_MIN) || (params->dead_zone_2 < MODALAI_ESC_MODE_DEAD_ZONE_MIN) ||
	    (params->dead_zone_1 >= MODALAI_ESC_MODE_DEAD_ZONE_MAX) || (params->dead_zone_2 >= MODALAI_ESC_MODE_DEAD_ZONE_MAX) ||
	    (params->dead_zone_2 >= params->dead_zone_1)) {
		PX4_ERR("Invalid parameter UART_ESC_DEAD1 or UART_ESC_DEAD2.  Please verify parameters.");
		params->dead_zone_1 = MODALAI_ESC_MODE_DEAD_ZONE_1;
		params->dead_zone_2 = MODALAI_ESC_MODE_DEAD_ZONE_2;
		ret = PX4_ERROR;
	}

	for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
		if (params->motor_map[i] == MODALAI_ESC_OUTPUT_DISABLED ||
		    params->motor_map[i] < -(MODALAI_ESC_OUTPUT_CHANNELS) ||
		    params->motor_map[i] > MODALAI_ESC_OUTPUT_CHANNELS) {
			PX4_ERR("Invalid parameter UART_ESC_MOTORX.  Please verify parameters.");
			params->motor_map[i] = 0;
			ret = PX4_ERROR;
		}

		/* Can map -4 to 4, 0 being disabled.  Negative represents reverse direction */
		map[i].number = abs(params->motor_map[i]);
		map[i].direction = (params->motor_map[i] > 0) ? 1 : -1;
	}

	return ret;
}

int ModalaiEsc::task_spawn(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_device = argv[myoptind];
			break;

		default:
			break;
		}
	}

	ModalaiEsc *instance = new ModalaiEsc();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ModalaiEsc::flushUartRx()
{
	while (_uart_port->uart_read(_read_buf, sizeof(_read_buf)) > 0) {}

	return 0;
}

int ModalaiEsc::readResponse(Command *out_cmd)
{
	px4_usleep(_current_cmd.resp_delay_us);

	int res = _uart_port->uart_read(_read_buf, sizeof(_read_buf));

	if (res > 0) {
		//PX4_INFO("read %i bytes",res);
		if (parseResponse(_read_buf, res, out_cmd->print_feedback) < 0) {
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

int ModalaiEsc::parseResponse(uint8_t *buf, uint8_t len, bool print_feedback)
{
	hrt_abstime tnow = hrt_absolute_time();

	for (int i = 0; i < len; i++) {
		int16_t ret = qc_esc_packet_process_char(buf[i], &_fb_packet);

		if (ret > 0) {
			//PX4_INFO("got packet of length %i",ret);
			uint8_t packet_type = qc_esc_packet_get_type(&_fb_packet);
			uint8_t packet_size = qc_esc_packet_get_size(&_fb_packet);

			if (packet_type == ESC_PACKET_TYPE_FB_RESPONSE && packet_size == sizeof(QC_ESC_FB_RESPONSE_V2)) {
				//PX4_INFO("Got feedback V2 packet!");
				QC_ESC_FB_RESPONSE_V2 fb;
				memcpy(&fb, _fb_packet.buffer, packet_size);

				uint32_t id             = (fb.id_state & 0xF0) >> 4;  //ID of the ESC based on hardware address

				if (id < MODALAI_ESC_OUTPUT_CHANNELS) {

					int motor_idx = _output_map[id].number - 1; // mapped motor id.. user defined mapping is 1-4, array is 0-3

					if (print_feedback) {
						uint32_t rpm         = fb.rpm;
						uint32_t power       = fb.power;
						uint32_t voltage     = fb.voltage;
						int32_t  current     = fb.current * 8;
						int32_t  temperature = fb.temperature / 100;
						PX4_INFO("[%lld] ID_RAW=%" PRIu32 " ID=%d, RPM=%5" PRIu32 ", PWR=%3" PRIu32 "%%, V=%5" PRIu32 "mV, I=%+5" PRIi32
							 "mA, T=%+3" PRIi32 "C",
							 tnow, id, motor_idx + 1, rpm, power, voltage, current, temperature);
					}

					_esc_chans[id].rate_meas     = fb.rpm;
					_esc_chans[id].power_applied = fb.power;
					_esc_chans[id].state         = fb.id_state & 0x0F;
					_esc_chans[id].cmd_counter   = fb.cmd_counter;
					_esc_chans[id].voltage       = fb.voltage * 0.001;
					_esc_chans[id].current       = fb.current * 0.008;
					_esc_chans[id].temperature   = fb.temperature * 0.01;
					_esc_chans[id].feedback_time = tnow;

					// also update our internal report for logging
					_esc_status.esc[id].esc_address  = motor_idx + 1; //remapped motor ID
					_esc_status.esc[id].timestamp    = tnow;
					_esc_status.esc[id].esc_rpm      = fb.rpm;
					//_esc_status.esc[id].esc_power    = fb.power;
					_esc_status.esc[id].esc_state    = fb.id_state & 0x0F;
					//_esc_status.esc[id].esc_cmdcount = fb.cmd_counter;
					_esc_status.esc[id].esc_voltage  = _esc_chans[id].voltage;
					_esc_status.esc[id].esc_current  = _esc_chans[id].current;
					_esc_status.esc[id].failures     = 0; //not implemented

					// use PX4 motor index here (already brough down to 0-3 above), so reporting of ESC online maps to PX4 motors
					_esc_status.esc_online_flags |= (1 << motor_idx);

					// state == 0 is stopped, but in turtle mode idle is OK so consider armed
					if (_esc_chans[id].state > 0 || _turtle_mode_en) {
						_esc_status.esc_armed_flags |= (1 << motor_idx);

					} else {
						_esc_status.esc_armed_flags &= ~(1 << motor_idx);
					}

					int32_t t = fb.temperature / 100;  //divide by 100 to get deg C and cap for int8

					if (t < -127) { t = -127; }

					if (t > +127) { t = +127; }

					_esc_status.esc[id].esc_temperature = t;

					_esc_status.timestamp = _esc_status.esc[id].timestamp;
					_esc_status.counter++;

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
				PX4_INFO("ESC ID: %i", ver.id);
				PX4_INFO("HW Version: %i", ver.hw_version);
				PX4_INFO("SW Version: %i", ver.sw_version);
				PX4_INFO("Unique ID: %" PRIu32, ver.unique_id);

			} else if (packet_type == ESC_PACKET_TYPE_VERSION_EXT_RESPONSE && packet_size == sizeof(QC_ESC_EXTENDED_VERSION_INFO)) {
				QC_ESC_EXTENDED_VERSION_INFO ver;
				memcpy(&ver, _fb_packet.buffer, packet_size);
				PX4_INFO("\tESC ID     : %i", ver.id);
				PX4_INFO("\tBoard      : %i", ver.hw_version);
				PX4_INFO("\tSW Version : %i", ver.sw_version);

				uint8_t *u = &ver.unique_id[0];
				PX4_INFO("\tUnique ID  : 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					 u[11], u[10], u[9], u[8], u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);

				PX4_INFO("\tFirmware   : version %4d, hash %.12s", ver.sw_version, ver.firmware_git_version);
				PX4_INFO("\tBootloader : version %4d, hash %.12s", ver.bootloader_version, ver.bootloader_git_version);
			}

		} else { //parser error
			/*
			switch (ret)
			{
				case ESC_ERROR_BAD_CHECKSUM: PX4_INFO("BAD ESC packet checksum"); break;
				case ESC_ERROR_BAD_LENGTH:   PX4_INFO("BAD ESC packet length");   break;
			}
			*/
		}
	}

	/*
		if (len < 4 || buf[0] != ESC_PACKET_HEADER) {
			return -1;
		}

		switch (buf[2]) {
		case ESC_PACKET_TYPE_VERSION_RESPONSE:
			if (len != sizeof(QC_ESC_VERSION_INFO)) {
				return -1;

			} else {
				QC_ESC_VERSION_INFO ver;
				memcpy(&ver, buf, len);
				PX4_INFO("ESC ID: %i", ver.id);
				PX4_INFO("HW Version: %i", ver.hw_version);
				PX4_INFO("SW Version: %i", ver.sw_version);
				PX4_INFO("Unique ID: %i", ver.unique_id);
			}

			break;

		case ESC_PACKET_TYPE_FB_RESPONSE:
			if (len != sizeof(QC_ESC_FB_RESPONSE)) {
				return -1;

			} else {
				QC_ESC_FB_RESPONSE fb;
				memcpy(&fb, buf, len);
				uint8_t id = (fb.state & 0xF0) >> 4;

				if (id < MODALAI_ESC_OUTPUT_CHANNELS) {
					_esc_chans[id].rate_meas = fb.rpm;
					_esc_chans[id].state = fb.state & 0x0F;
					_esc_chans[id].cmd_counter = fb.cmd_counter;
					_esc_chans[id].voltage = 9.0 + fb.voltage / 34.0;
				}
			}

			break;

		default:
			return -1;
		}
	*/

	return 0;
}

int ModalaiEsc::checkForEscTimeout()
{
	hrt_abstime tnow = hrt_absolute_time();

	for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
		// PX4 motor indexed user defined mapping is 1-4, we want to use in bitmask (0-3)
		uint8_t motor_idx = _output_map[i].number - 1;

		if (motor_idx < MODALAI_ESC_OUTPUT_CHANNELS) {
			// we are using PX4 motor index in the bitmask
			if (_esc_status.esc_online_flags & (1 << motor_idx)) {
				// using index i here for esc_chans enumeration stored in ESC ID order
				if ((tnow - _esc_chans[i].feedback_time) > MODALAI_ESC_DISCONNECT_TIMEOUT_US) {
					// stale data, assume offline and clear armed
					_esc_status.esc_online_flags &= ~(1 << motor_idx);
					_esc_status.esc_armed_flags &= ~(1 << motor_idx);
				}
			}
		}
	}

	return 0;

}

int ModalaiEsc::sendCommandThreadSafe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}



int ModalaiEsc::custom_command(int argc, char *argv[])
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

	/* start the FMU if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return ModalaiEsc::task_spawn(argc, argv);
		}
	}

	if (!is_running()) {
		PX4_INFO("Not running");
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
		if (esc_id < 4) {
			PX4_INFO("Reset ESC: %i", esc_id);
			cmd.len = qc_esc_create_reset_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "version")) {
		if (esc_id < 4) {
			PX4_INFO("Request version for ESC: %i", esc_id);
			cmd.len = qc_esc_create_version_request_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = true;
			cmd.resp_delay_us = 2000;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "version-ext")) {
		if (esc_id < 4) {
			PX4_INFO("Request version for ESC: %i", esc_id);
			cmd.len = qc_esc_create_extended_version_request_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = true;
			cmd.resp_delay_us = 5000;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "tone")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request tone for ESC mask: %i", esc_id);
			cmd.len = qc_esc_create_sound_packet(period, duration, power, esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	} else if (!strcmp(verb, "led")) {
		if (led_mask <= 0x0FFF) {
			get_instance()->_led_rsc.test = true;
			get_instance()->_led_rsc.breath_en = false;
			PX4_INFO("Request LED control for ESCs with mask: %i", led_mask);

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
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request RPM for ESC bit mask: %i - RPM: %i", esc_id, rate);
			int16_t rate_req[MODALAI_ESC_OUTPUT_CHANNELS];
			int16_t outputs[MODALAI_ESC_OUTPUT_CHANNELS];
			outputs[0] = (esc_id & 1) ? rate : 0;
			outputs[1] = (esc_id & 2) ? rate : 0;
			outputs[2] = (esc_id & 4) ? rate : 0;
			outputs[3] = (esc_id & 8) ? rate : 0;

			//the motor mapping is.. if I want to spin Motor 1 (1-4) then i need to provide non-zero rpm for motor map[m-1]

			uart_esc_params_t params;
			ch_assign_t map[MODALAI_ESC_OUTPUT_CHANNELS];
			get_instance()->load_params(&params, (ch_assign_t *)&map);

			uint8_t id_fb_raw = 0;
			uint8_t id_fb = 0;

			if (esc_id & 1) { id_fb_raw = 0; }

			else if (esc_id & 2) { id_fb_raw = 1; }

			else if (esc_id & 4) { id_fb_raw = 2; }

			else if (esc_id & 8) { id_fb_raw = 3; }

			for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
				int motor_idx = map[i].number - 1; // user defined mapping is 1-4, array is 0-3

				if (motor_idx > 0 && motor_idx <= MODALAI_ESC_OUTPUT_CHANNELS) {
					rate_req[i] = outputs[motor_idx] * map[i].direction;
				}

				if (motor_idx == id_fb_raw) {
					id_fb = i;
				}
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
							       sizeof(cmd.buf));

			cmd.response        = true;
			cmd.repeats         = repeat_count;
			cmd.resp_delay_us   = 0;
			cmd.repeat_delay_us = repeat_delay_us;
			cmd.print_feedback  = true;

			PX4_INFO("ESC map: %d %d %d %d", map[0].number, map[1].number, map[2].number, map[3].number);
			PX4_INFO("feedback id debug: %i, %i", id_fb_raw, id_fb);
			PX4_INFO("Sending UART ESC RPM command %i", rate);

			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	} else if (!strcmp(verb, "pwm")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request PWM for ESC mask: %i - PWM: %i", esc_id, rate);
			int16_t rate_req[MODALAI_ESC_OUTPUT_CHANNELS];
			int16_t outputs[MODALAI_ESC_OUTPUT_CHANNELS];
			outputs[0] = (esc_id & 1) ? rate : 0;
			outputs[1] = (esc_id & 2) ? rate : 0;
			outputs[2] = (esc_id & 4) ? rate : 0;
			outputs[3] = (esc_id & 8) ? rate : 0;

			uart_esc_params_t params;
			ch_assign_t map[MODALAI_ESC_OUTPUT_CHANNELS];
			get_instance()->load_params(&params, (ch_assign_t *)&map);

			uint8_t id_fb_raw = 0;
			uint8_t id_fb = 0;

			if (esc_id & 1) { id_fb_raw = 0; }

			else if (esc_id & 2) { id_fb_raw = 1; }

			else if (esc_id & 4) { id_fb_raw = 2; }

			else if (esc_id & 8) { id_fb_raw = 3; }

			for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
				int motor_idx = map[i].number - 1; // user defined mapping is 1-4, array is 0-3

				if (motor_idx > 0 && motor_idx <= MODALAI_ESC_OUTPUT_CHANNELS) {
					rate_req[i] = outputs[motor_idx] * map[i].direction;
				}

				if (motor_idx == id_fb_raw) {
					id_fb = i;
				}
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

			PX4_INFO("ESC map: %d %d %d %d", map[0].number, map[1].number, map[2].number, map[3].number);
			PX4_INFO("feedback id debug: %i, %i", id_fb_raw, id_fb);
			PX4_INFO("Sending UART ESC power command %i", rate);


			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}
	}

	return print_usage("unknown command");
}

int ModalaiEsc::update_params()
{
	int ret = PX4_ERROR;

	updateParams();
	ret = load_params(&_parameters, (ch_assign_t *)&_output_map);

	if (ret == PX4_OK) {
		_mixing_output.setAllMinValues(_parameters.rpm_min);
		_mixing_output.setAllMaxValues(_parameters.rpm_max);
		_rpm_fullscale = _parameters.rpm_max - _parameters.rpm_min;
	}

	return ret;
}


int ModalaiEsc::ioctl(file *filp, int cmd, unsigned long arg)
{
	SmartLock lock_guard(_lock);

	int ret = OK;

	PX4_DEBUG("modalai_esc ioctl cmd: %d, arg: %ld", cmd, arg);

	switch (cmd) {
	case PWM_SERVO_ARM:
		PX4_INFO("PWM_SERVO_ARM");
		break;

	case PWM_SERVO_DISARM:
		PX4_INFO("PWM_SERVO_DISARM");
		break;

	case MIXERIOCRESET:
		_mixing_output.resetMixer();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixer(buf, buflen);
		}
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

/* OutputModuleInterface */
void ModalaiEsc::mixerChanged()
{
	/*
	 * This driver is only supporting 4 channel ESC
	 */
}


void ModalaiEsc::updateLeds(vehicle_control_mode_s mode, led_control_s control)
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

	for (i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
		_esc_chans[i].led = led_mask;
	}
}

/* OutputModuleInterface */
bool ModalaiEsc::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			       unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (num_outputs != MODALAI_ESC_OUTPUT_CHANNELS) {
		return false;
	}

	uint8_t motor_idx;

	for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
		if (!_outputs_on || stop_motors) {
			_esc_chans[i].rate_req = 0;

		} else {
			motor_idx = _output_map[i].number;

			if (motor_idx > 0 && motor_idx <= MODALAI_ESC_OUTPUT_CHANNELS) {
				/* user defined mapping is 1-4, array is 0-3 */
				motor_idx--;

				if (!_turtle_mode_en) {
					_esc_chans[i].rate_req = outputs[motor_idx] * _output_map[i].direction;

				} else {
					/* we may have rolled back into a dead zone by now, clear out */
					_esc_chans[i].rate_req = 0;

					float setpoint = 0.0f;
					bool use_setpoint = false;

					/* At this point, we are switching on what PX4 motor we want to talk to */
					switch (_output_map[i].number) {
					/*
					 * An ASCII graphic of this dead zone logic is above in load_params
					 */

					/* PX4 motor 1 - front right */
					case 1:

						/* Pitch and roll */
						if (_manual_control_setpoint.x > _parameters.dead_zone_1) {
							if (_manual_control_setpoint.y > -(_parameters.dead_zone_2)) {
								setpoint = _manual_control_setpoint.x;
								use_setpoint = true;
								//PX4_ERR("motor1");
							}

						} else if (_manual_control_setpoint.y > _parameters.dead_zone_1) {
							if (_manual_control_setpoint.x > -(_parameters.dead_zone_2)) {
								setpoint = _manual_control_setpoint.y;
								use_setpoint = true;
								//PX4_ERR("motor1");
							}
						}

						/* Yaw */
						if (_manual_control_setpoint.r < -(_parameters.dead_zone_1)) {
							setpoint = fabs(_manual_control_setpoint.r);
							use_setpoint = true;
							//PX4_ERR("motor1");
						}

						break;

					/* PX4 motor 3 - front left */
					case 3:

						/* Pitch and roll */
						if (_manual_control_setpoint.x > _parameters.dead_zone_1) {
							if (_manual_control_setpoint.y < _parameters.dead_zone_2) {
								setpoint = _manual_control_setpoint.x;
								use_setpoint = true;
								//PX4_ERR("motor3");
							}

						} else if (_manual_control_setpoint.y < -(_parameters.dead_zone_1)) {
							if (_manual_control_setpoint.x > -(_parameters.dead_zone_2)) {
								setpoint = fabs(_manual_control_setpoint.y);
								use_setpoint = true;
								//PX4_ERR("motor3");
							}
						}

						/* Yaw */
						if (_manual_control_setpoint.r > _parameters.dead_zone_1) {
							setpoint = _manual_control_setpoint.r;
							use_setpoint = true;
							//PX4_ERR("motor3");
						}

						break;

					/* PX4 motor 2 - rear left */
					case 2:

						/* Pitch and roll */
						if (_manual_control_setpoint.x < -(_parameters.dead_zone_1)) {
							if (_manual_control_setpoint.y < _parameters.dead_zone_2) {
								setpoint = fabs(_manual_control_setpoint.x);
								use_setpoint = true;
								//PX4_ERR("motor2");
							}

						} else if (_manual_control_setpoint.y < -(_parameters.dead_zone_1)) {
							if (_manual_control_setpoint.x < _parameters.dead_zone_2) {
								setpoint = fabs(_manual_control_setpoint.y);
								use_setpoint = true;
								//PX4_ERR("motor2");
							}
						}

						/* Yaw */
						if (_manual_control_setpoint.r < -(_parameters.dead_zone_1)) {
							setpoint = fabs(_manual_control_setpoint.r);
							use_setpoint = true;
							//PX4_ERR("motor2");
						}

						break;

					/* PX4 motor 4- rear right */
					case 4:

						/* Pitch and roll */
						if (_manual_control_setpoint.x < -_parameters.dead_zone_1) {
							if (_manual_control_setpoint.y > -_parameters.dead_zone_2) {
								setpoint = fabs(_manual_control_setpoint.x);
								use_setpoint = true;
								//PX4_ERR("motor4");
							}
						}

						if (_manual_control_setpoint.y > _parameters.dead_zone_1) {
							if (_manual_control_setpoint.x < _parameters.dead_zone_2) {
								setpoint = _manual_control_setpoint.y;
								use_setpoint = true;
								//PX4_ERR("motor4");
							}
						}

						/* Yaw */
						if (_manual_control_setpoint.r > _parameters.dead_zone_1) {
							setpoint = _manual_control_setpoint.r;
							use_setpoint = true;
							//PX4_ERR("motor4");
						}

						break;
					}

					// set rate
					float rate = 0.0f;

					if (use_setpoint) {
						rate = (float)_parameters.rpm_min + ((float)_rpm_fullscale * setpoint);
						rate = (-1.0f) * rate * (float)_output_map[i].direction;
					}

					_esc_chans[i].rate_req = (int16_t)rate;
				}
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
					       sizeof(cmd.buf));


	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
		return false;
	}

	// round robin
	_fb_idx = (_fb_idx + 1) % MODALAI_ESC_OUTPUT_CHANNELS;


	/*
	 * Here we parse the feedback response.  Rarely the packet is mangled
	 * but this means we simply miss a feedback response and will come back
	 * around in roughly 8ms for another... so don't freak out and keep on
	 * trucking I say
	 */
	int res = _uart_port->uart_read(_read_buf, sizeof(_read_buf));

	if (res > 0) {
		parseResponse(_read_buf, res, false);
	}

	/* handle loss of comms / disconnect */
	checkForEscTimeout();

	// publish the actual command that we sent and the feedback received
	if (MODALAI_PUBLISH_ESC_STATUS) {
		// actuator_outputs_s actuator_outputs{};
		// actuator_outputs.noutputs = num_outputs;

		// for (size_t i = 0; i < num_outputs; ++i) {
		// 	actuator_outputs.output[i] = _esc_chans[i].rate_req;
		// }

		// actuator_outputs.timestamp = hrt_absolute_time();

		// _outputs_debug_pub.publish(actuator_outputs);

		_esc_status_pub.publish(_esc_status);
	}

	perf_count(_output_update_perf);

	return true;
}


void ModalaiEsc::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	SmartLock lock_guard(_lock);

	perf_begin(_cycle_perf);

	/* Open serial port in this thread */
	if (!_uart_port->is_open()) {
		if (_uart_port->uart_open(_device, _parameters.baud_rate) == PX4_OK) {
			PX4_INFO("Opened UART ESC device");

		} else {
			PX4_ERR("Failed openening device");
			return;
		}
	}

	/*
		for (int ii=0; ii<9; ii++)
		{
		  const char * test_str = "Hello World!";
		  _uart_port_bridge->uart_write((char*)test_str,12);
	    px4_usleep(10000);
		}
	*/
	/*
	uint8_t echo_buf[16];
	int bytes_read = _uart_port_bridge->uart_read(echo_buf,sizeof(echo_buf));
	if (bytes_read > 0)
	  _uart_port_bridge->uart_write(echo_buf,bytes_read);
	*/


	_mixing_output.update();

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
		updateLeds(vehicle_control_mode, _led_rsc.control);
	}

	led_control_s led_control{};

	if (_led_update_sub.updated()) {
		_led_update_sub.copy(&led_control);
		updateLeds(_led_rsc.mode, led_control);
	}

	/* breathing requires continuous updates */
	if (_led_rsc.breath_en) {
		updateLeds(_led_rsc.mode, _led_rsc.control);
	}

	if (_parameters.mode > 0) {
		/* if turtle mode enabled, we go straight to the sticks, no mix */
		if (_manual_control_setpoint_sub.updated()) {

			_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

			if (!_outputs_on) {

				float setpoint = MODALAI_ESC_MODE_DISABLED_SETPOINT;

				if (_parameters.mode == MODALAI_ESC_MODE_TURTLE_AUX1) {
					setpoint = _manual_control_setpoint.aux1;

				} else if (_parameters.mode == MODALAI_ESC_MODE_TURTLE_AUX2) {
					setpoint = _manual_control_setpoint.aux2;
				}

				if (setpoint > MODALAI_ESC_MODE_THRESHOLD) {
					_turtle_mode_en = true;
					//PX4_ERR("turtle mode enabled\n");

				} else {
					_turtle_mode_en = false;
					//PX4_ERR("turtle mode disabled\n");
				}
			}
		}

		if (_parameters.mode == MODALAI_ESC_MODE_UART_BRIDGE) {
			if (!_uart_port_bridge->is_open()) {
				if (_uart_port_bridge->uart_open(MODALAI_ESC_VOXL_PORT, 230400) == PX4_OK) {
					PX4_INFO("Opened UART ESC Bridge device");

				} else {
					PX4_ERR("Failed openening UART ESC Bridge device");
					return;
				}
			}

			//uart passthrough test code
			//run 9 times because i just don't know how to change update rate of the module from 10hz to 100hz..
			for (int ii = 0; ii < 9; ii++) {
				uint8_t uart_buf[128];
				int bytes_read = _uart_port_bridge->uart_read(uart_buf, sizeof(uart_buf));

				if (bytes_read > 0) {
					_uart_port->uart_write(uart_buf, bytes_read);

					for (int i = 0; i < bytes_read; i++) {
						int16_t ret = qc_esc_packet_process_char(uart_buf[i], &_uart_bridge_packet);

						if (ret > 0) {
							//PX4_INFO("got packet of length %i",ret);
							uint8_t packet_type = qc_esc_packet_get_type(&_uart_bridge_packet);

							//uint8_t packet_size = qc_esc_packet_get_size(&_uart_bridge_packet);
							//if we received a command for ESC to reset, most likely firmware update is coming, switch to bootloader baud rate
							if (packet_type == ESC_PACKET_TYPE_RESET_CMD) {
								int bootloader_baud_rate = 230400;

								if (_uart_port->uart_get_baud() != bootloader_baud_rate) {
									px4_usleep(5000);
									_uart_port->uart_set_baud(bootloader_baud_rate);
								}

							} else {
								if (_uart_port->uart_get_baud() != _parameters.baud_rate) {
									px4_usleep(5000);
									_uart_port->uart_set_baud(_parameters.baud_rate);  //restore normal baud rate
								}
							}
						}
					}
				}

				bytes_read = _uart_port->uart_read(uart_buf, sizeof(uart_buf));

				if (bytes_read > 0) {
					_uart_port_bridge->uart_write(uart_buf, bytes_read);
				}

				px4_usleep(10000);
			}
		}

	} else {
		if (_uart_port_bridge->is_open()) {
			PX4_INFO("Closed UART ESC Bridge device");
			_uart_port_bridge->uart_close();
		}
	}

	/* Don't process commands if outputs on */
	if (!_outputs_on) {
		if (_current_cmd.valid()) {
			//PX4_INFO("sending %d commands with delay %dus",_current_cmd.repeats,_current_cmd.repeat_delay_us);
			flushUartRx();

			do {
				//PX4_INFO("CMDs left %d",_current_cmd.repeats);
				if (_uart_port->uart_write(_current_cmd.buf, _current_cmd.len) == _current_cmd.len) {
					if (_current_cmd.repeats == 0) {
						_current_cmd.clear();
					}

					if (_current_cmd.response) {
						readResponse(&_current_cmd);
					}

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

int ModalaiEsc::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("modalai_esc", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Send reset request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("version", "Send version request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("version-ext", "Send extended version request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("rpm", "Closed-Loop RPM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, -32768, 32768, "RPM, -32,768 to 32,768", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 100, 0, 1<<31, "Command repeat count, 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 10000, 0, 1<<31, "Delay between repeated commands (microseconds), 0 to INT_MAX", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "Open-Loop PWM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 800, "Duty Cycle value, 0 to 800", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 100, 0, 1<<31, "Command repeat count, 0 to INT_MAX", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 10000, 0, 1<<31, "Delay between repeated commands (microseconds), 0 to INT_MAX", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("tone", "Send tone generation request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 255, "Period of sound, inverse frequency, 0-255", false);
	PRINT_MODULE_USAGE_PARAM_INT('d', 0, 0, 255, "Duration of the sound, 0-255, 1LSB = 13ms", false);
	PRINT_MODULE_USAGE_PARAM_INT('v', 0, 0, 100, "Power (volume) of sound, 0-100", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "Send LED control request");
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, 4095, "Bitmask 0x0FFF (12 bits) - ESC0 (RGB) ESC1 (RGB) ESC2 (RGB) ESC3 (RGB)", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ModalaiEsc::print_status()
{
	PX4_INFO("Max update rate: %i Hz", _current_update_rate);
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	PX4_INFO("UART port: %s", _device);
	PX4_INFO("UART open: %s", _uart_port->is_open() ? "yes" : "no");

	PX4_INFO("");

	PX4_INFO("Params: UART_ESC_CONFIG: % " PRIi32, _parameters.config);
	PX4_INFO("Params: UART_ESC_BAUD: % " PRIi32, _parameters.baud_rate);
	PX4_INFO("Params: UART_ESC_MOTOR1: % " PRIi32, _parameters.motor_map[0]);
	PX4_INFO("Params: UART_ESC_MOTOR2: % " PRIi32, _parameters.motor_map[1]);
	PX4_INFO("Params: UART_ESC_MOTOR3: % " PRIi32, _parameters.motor_map[2]);
	PX4_INFO("Params: UART_ESC_MOTOR4: % " PRIi32, _parameters.motor_map[3]);
	PX4_INFO("Params: UART_ESC_RPM_MIN: % " PRIi32, _parameters.rpm_min);
	PX4_INFO("Params: UART_ESC_RPM_MAX: % " PRIi32, _parameters.rpm_max);

	PX4_INFO("");

	for( int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++){
		PX4_INFO("-- ID: %i", i);
		PX4_INFO("   State:           %i", _esc_chans[i].state);
		PX4_INFO("   Requested:       %i RPM", _esc_chans[i].rate_req);
		PX4_INFO("   Measured:        %i RPM", _esc_chans[i].rate_meas);
		PX4_INFO("   Command Counter: %i", _esc_chans[i].cmd_counter);
		PX4_INFO("   Voltage:         %f VDC", (double)_esc_chans[i].voltage);
		PX4_INFO("");
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_output_update_perf);

	_mixing_output.printStatus();

	return 0;
}

extern "C" __EXPORT int modalai_esc_main(int argc, char *argv[])
{
	return ModalaiEsc::main(argc, argv);
}
