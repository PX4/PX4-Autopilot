/****************************************************************************
 *
 *   Copyright (c) 2023 ModalAI, Inc. All rights reserved.
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


#include <string>
#include <px4_log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Subscription.hpp>
#include <drivers/device/qurt/uart.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <lib/parameters/param.h>
#include "elrs_led.h"


extern "C" { __EXPORT int elrs_led_main(int argc, char *argv[]); }

namespace elrs_led
{

std::string _port = "7";
int _uart_fd = -1;
bool _initialized = false;
bool _is_running = false;
static bool _debug = false;

static GENERIC_CRC8 crsf_crc{};
static LEDState _state = LEDState::DEFAULT;
static ControllerInput _off = ControllerInput::DEFAULT;
static ControllerInput _on = ControllerInput::DEFAULT;
static ControllerInput _ir = ControllerInput::DEFAULT;
static ControllerInput _cmd = ControllerInput::DEFAULT;
static ControllerInput _prev_cmd = ControllerInput::DEFAULT;
static std::map<ControllerInput, std::string> ControllerInputMap{
	{ControllerInput::DLEFT, "DLEFT"},
	{ControllerInput::DRIGHT, "DRIGHT"},
	{ControllerInput::DDOWN, "DDOWN"},
	{ControllerInput::DUP, "DUP"},
	{ControllerInput::BACK, "BACK"},
	{ControllerInput::START, "START"},
	{ControllerInput::Y, "Y"},
	{ControllerInput::B, "B"},
	{ControllerInput::A, "A"},
	{ControllerInput::X, "X"},
	{ControllerInput::STICK_RIGHT, "STICK_RIGHT"},
	{ControllerInput::STICK_LEFT, "STICK_LEFT"},
	{ControllerInput::BUMPER_RIGHT, "BUMPER_RIGHT"},
	{ControllerInput::BUMPER_LEFT, "BUMPER_LEFT"},
	{ControllerInput::DEFAULT, "Unkown"}
};
static px4_task_t _task_handle = -1;
void debug_info(LEDState, uint8_t *);
void make_packet(LEDState, uint8_t *);

int initialize()
{
	if (_initialized) {
		// Already successfully initialized
		return 0;
	}

	if (_uart_fd < 0) {
		_uart_fd = qurt_uart_open(_port.c_str(), 420000);
	}

	if (_uart_fd < 0) {
		PX4_ERR("Open failed in %s", __FUNCTION__);
		return -1;
	}

	_initialized = true;

	return 0;
}

void elrs_led_task()
{

	PX4_INFO("Starting task for elrs_led");

	int ret = 0;
	int manual_control_input_fd  = orb_subscribe(ORB_ID(manual_control_input));
	uint8_t pwmPacket[11] = {0xEC, 0x09, 0x32, 0x70, 0x77, 0x6D, 0x07, 0x75, 0x00, 0x00, 0x00};

	px4_pollfd_struct_t fds[1] = { { .fd = manual_control_input_fd,  .events = POLLIN }	};

	struct manual_control_setpoint_s setpoint_req;

	_is_running = true;

	while (true) {
		px4_poll(fds, 1, 10000);

		if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(manual_control_input), manual_control_input_fd, &setpoint_req);

			_cmd = (ControllerInput)setpoint_req.buttons;

			// skip duplicate cmds
			if (_cmd == _prev_cmd) {
				continue;
			}

			if (_cmd == _off) {
				_prev_cmd = _cmd;
				_state = LEDState::OFF;
				make_packet(_state, pwmPacket);
				ret = qurt_uart_write(_uart_fd, (char *) &pwmPacket[0], sizeof(pwmPacket));

				if (_debug) {
					debug_info(_state, pwmPacket);
				}

			} else if (_cmd == _on) {
				_prev_cmd = _cmd;
				_state = LEDState::ON;
				make_packet(_state, pwmPacket);
				ret = qurt_uart_write(_uart_fd, (char *) &pwmPacket[0], sizeof(pwmPacket));

				if (_debug) {
					debug_info(_state, pwmPacket);
				}

			} else if (_cmd == _ir) {
				_prev_cmd = _cmd;
				_state = LEDState::IR;
				make_packet(_state, pwmPacket);
				ret = qurt_uart_write(_uart_fd, (char *) &pwmPacket[0], sizeof(pwmPacket));

				if (_debug) {
					debug_info(_state, pwmPacket);
				}

			}

		}
	}
}

int start(int argc, char *argv[])
{

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:o:l:i:d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			_port = myoptarg;
			break;

		case 'o':
			_off = getKey(ControllerInputMap, myoptarg);
			break;

		case 'l':
			_on = getKey(ControllerInputMap, myoptarg);
			break;

		case 'i':
			_ir = getKey(ControllerInputMap, myoptarg);
			break;

		case 'd':
			_debug = true;
			break;

		default:
			break;
		}
	}

	if (_debug) {
		PX4_INFO("ELRS LED Debug Mode Enabled");
		PX4_INFO("Port: %s", _port.c_str());
		PX4_INFO("Button Configuration:");
		PX4_INFO("\tOn: %s", ControllerInputMap.at(_on).c_str());
		PX4_INFO("\tIR: %s", ControllerInputMap.at(_ir).c_str());
		PX4_INFO("\tOff: %s", ControllerInputMap.at(_off).c_str());
	}

	if (! _initialized) {
		if (initialize()) {
			return -1;
		}
	}

	if (_is_running) {
		PX4_WARN("Already started");
		return 0;
	}

	_task_handle = px4_task_spawn_cmd("elrs_led_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t) &elrs_led_task,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: elrs_led start [options]");
	PX4_INFO("Options: -p <number>    uart port number");
	PX4_INFO("Options: -o <number>    LEDs off button");
	PX4_INFO("Options: -l <number>    Overt LEDs on button");
	PX4_INFO("Options: -i <number>    IR LEDs on button");
	PX4_INFO("Options: -d <number>    enable debug messages");
}

void debug_info(LEDState led_state, uint8_t *pwmPacket)
{
	PX4_INFO("");

	if (led_state == LEDState::ON) {
		PX4_INFO("Turning LEDs on");

	} else if (led_state == LEDState::OFF) {
		PX4_INFO("Turning LEDs off");

	} else if (led_state == LEDState::IR) {
		PX4_INFO("Turning IR LEDs on");

	} else {
		PX4_WARN("ELRS LED: LED state unknown: 0x%x", led_state);
	}

	PX4_INFO("Wrote packet: [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]",
		 pwmPacket[0], pwmPacket[1], pwmPacket[2], pwmPacket[3], pwmPacket[4], pwmPacket[5],
		 pwmPacket[6], pwmPacket[7], pwmPacket[8], pwmPacket[9], pwmPacket[10]);
}

void make_packet(LEDState led_state, uint8_t *pwmPacket)
{
	if (led_state == LEDState::OFF) {
		pwmPacket[8] = 0x03;
		pwmPacket[9] = 0x84;
		pwmPacket[10] = crsf_crc.calc(&pwmPacket[CRSF_FRAME_NOT_COUNTED_BYTES], PWM_FRAME_SIZE - 1, 0);

	} else if (led_state == LEDState::ON) {
		pwmPacket[8] = 0x05;
		pwmPacket[9] = 0xAA;
		pwmPacket[10] = crsf_crc.calc(&pwmPacket[CRSF_FRAME_NOT_COUNTED_BYTES], PWM_FRAME_SIZE - 1, 0);

	} else if (led_state == LEDState::IR) {
		pwmPacket[8] = 0x07;
		pwmPacket[9] = 0xFF;
		pwmPacket[10] = crsf_crc.calc(&pwmPacket[CRSF_FRAME_NOT_COUNTED_BYTES], PWM_FRAME_SIZE - 1, 0);

	} else {
		PX4_WARN("ELRS LED: Unknown LED state.");
	}
}

} // End namespance elrs_led

int elrs_led_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		elrs_led::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return elrs_led::start(argc - 1, argv + 1);

	} else {
		elrs_led::usage();
		return -1;
	}

	return 0;
}
