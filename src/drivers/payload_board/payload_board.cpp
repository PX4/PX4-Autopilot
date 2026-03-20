/****************************************************************************
 *
 *	Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "payload_board.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <termios.h>

#include "impl/green_tech.hpp"
#include "impl/sky_fall.hpp"

ModuleBase::Descriptor PayloadBoard::desc{task_spawn, custom_command, print_usage};

using namespace time_literals;

PayloadBoard::PayloadBoard(const char *device)
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	  _perf_cycle(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle time")),
	  _comms_errors(perf_alloc(PC_COUNT, MODULE_NAME ": comms errors")),
	  _protocol(nullptr)
{
	if (device) {
		strlcpy(_device, device, sizeof(_device));
	}
}

PayloadBoard::~PayloadBoard()
{
	delete _protocol;
	perf_free(_perf_cycle);
	perf_free(_comms_errors);
}

int PayloadBoard::init()
{
	if (!_protocol) {
		if (_param_pb_driver.get() == 0) {
			_protocol = new payload_board::GreenTech{static_cast<uint8_t>(_param_pb_charge_time.get())};

		} else {
			_protocol = new payload_board::SkyFall{_input_rc_sub, _vehicle_status_sub};
		}

		if (!_protocol) {
			PX4_ERR("Failed to allocate protocol");
			return PX4_ERROR;
		}
	}

	_protocol->setSwapRxTx(_param_pb_swap_rx_tx.get());
	return _protocol->init(_device);
}

void PayloadBoard::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_perf_cycle);

	if (!_initialized) {
		PX4_DEBUG("Initializing payload board driver.");
		_initialized = (init() == PX4_OK);

	} else {
		if (_protocol->update() == PX4_OK) {
			if (_protocol->copy_to(&_last_response) == PX4_OK) {
				_payload_response_pub.publish(_last_response);

			} else {
				PX4_DEBUG("Failed to publish payload board response.");
			}

		} else {
			PX4_DEBUG("Failed to receive payload board response.");
			perf_count(_comms_errors);

			if (!_protocol->serial_valid()) {
				PX4_WARN("Going to reinitialize serial.");
				_initialized = false;
			}
		}
	}

	ScheduleDelayed(25_ms);
	perf_end(_perf_cycle);
}

int PayloadBoard::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "start")) {
		if (is_running(desc)) {
			return print_usage("already running");
		}

		int ret = PayloadBoard::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	if (!is_running(desc)) {
		return print_usage("not running");
	}

	return print_usage("unknown command");
}

int PayloadBoard::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			return -1;
			break;
		}
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		auto *const instance = new PayloadBoard(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		instance->ScheduleNow();

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

const char *PayloadBoard::get_state()
{
	switch (_last_response.state) {
	case payload_response_s::STATE_INACTIVE:
		return "INACTIVE";

	case payload_response_s::STATE_CHARGING:
		return "CHARGING";

	case payload_response_s::STATE_READY:
		return "READY";

	case payload_response_s::STATE_ACTIVE:
		return "ACTIVE";

	default:
		return "ERR";
	}
}
int PayloadBoard::print_status()
{
	if (_device[0]) { PX4_INFO("UART device: %s", _device); }

	PX4_INFO("State: %s", get_state());

	PX4_INFO("Countdown: %d", _last_response.countdown);
	PX4_INFO("Custom message: %.16s", _last_response.custom_message);

	perf_print_counter(_perf_cycle);
	perf_print_counter(_comms_errors);

	return 0;
}

int PayloadBoard::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module communicates with an Initiation board via serial port.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("payload_board", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Initiation board device", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int payload_board_main(int argc, char *argv[]) { return ModuleBase::main(PayloadBoard::desc, argc, argv); }
