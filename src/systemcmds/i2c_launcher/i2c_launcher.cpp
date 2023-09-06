/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include <stdlib.h>
#include "i2c_launcher.hpp"
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_arch/i2c_hw_description.h>

constexpr I2CLauncher::I2CDevice I2CLauncher::_devices[];

I2CLauncher::I2CLauncher(int bus) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(bus)),
	_bus(bus)
{
}

I2CLauncher::~I2CLauncher()
{
}

bool I2CLauncher::init()
{
	ScheduleOnInterval(1_s);

	return true;
}

void I2CLauncher::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
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

	if (_armed) {
		// Don't try to configure devices when armed.
		return;
	}

	scan_i2c_bus(_bus);
}

void I2CLauncher::scan_i2c_bus(int bus)
{
	struct i2c_master_s *i2c_dev = px4_i2cbus_initialize(bus);

	if (i2c_dev == nullptr) {
		PX4_ERR("invalid bus %d", bus);
		return;
	}

	for (unsigned i = 0; i < sizeof(_devices) / sizeof(_devices[0]); ++i) {

		bool running = false;
		{
			// We need to check whether any of the devices with the same I2C address are already running,
			// because even if they are not running, we could not address them.
			for (unsigned j = 0; j < sizeof(_devices) / sizeof(_devices[0]); ++j) {

				if (_devices[i].i2c_addr != _devices[j].i2c_addr) {
					continue;
				}

				BusCLIArguments bus_cli_arguments{true, false};
				bus_cli_arguments.bus_option = I2CSPIBusOption::I2CExternal;
				bus_cli_arguments.requested_bus = bus;

				BusInstanceIterator i2c_bus_instance_iterator {
					_devices[j].cmd, bus_cli_arguments, _devices[j].devid_driver_index};

				while (i2c_bus_instance_iterator.next()) {
					if (i2c_bus_instance_iterator.runningInstancesOnBusCount() > 0) {
						running = true;
						break;
					}
				}
			}
		}

		if (running) {
			continue;
		}

		const unsigned retries = 1;

		bool found = false;

		for (unsigned retry_count = 0; retry_count < retries; ++retry_count) {

			uint8_t send_data = 0;
			uint8_t recv_data = 0;
			i2c_msg_s msgv[2] {};

			// Send
			msgv[0].frequency = 100000;
			msgv[0].addr = _devices[i].i2c_addr;
			msgv[0].flags = 0;
			msgv[0].buffer = &send_data;
			msgv[0].length = sizeof(send_data);

			// Receive
			msgv[1].frequency = 100000;
			msgv[1].addr = _devices[i].i2c_addr;
			msgv[1].flags = I2C_M_READ;
			msgv[1].buffer = &recv_data;;
			msgv[1].length = sizeof(recv_data);

			if (I2C_TRANSFER(i2c_dev, &msgv[0], 2) == PX4_OK) {
				found = true;
				break;
			}
		}

		if (found) {
			char buf[32];
			snprintf(buf, sizeof(buf), "%s -X -b %d -t %d start", _devices[i].cmd, bus, bus);

			PX4_INFO("Found address 0x%x, running '%s'\n", _devices[i].i2c_addr, buf);

			// Try starting, if it succeeds we assume it's started and we no longer have to
			// check this device.
			const int ret = system(buf);

			if (ret == 0) {
				PX4_INFO("Started 0x%x successfully", _devices[i].i2c_addr);

			} else {
				PX4_INFO("Could not start 0x%x, returned %d", _devices[i].i2c_addr, ret);
			}
		}
	}

	px4_i2cbus_uninitialize(i2c_dev);
}

int I2CLauncher::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int I2CLauncher::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Daemon that starts drivers based on found I2C devices.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("i2c_launcher", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 1, 4, "Bus number", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int i2c_launcher_main(int argc, char *argv[])
{
	using ThisDriver = I2CLauncher;

	static I2CLauncher* instances[I2C_BUS_MAX_BUS_ITEMS];
	int bus = -1;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	const char *verb = argv[1];

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			bus = strtol(myoptarg, nullptr, 10);
			break;

		default:
			return ThisDriver::print_usage("unrecognized flag");
		}
	}

	if (bus == -1) {
		PX4_ERR("bus not set");
		return PX4_ERROR;
	}

	if (bus > I2C_BUS_MAX_BUS_ITEMS) {
		PX4_ERR("bus out of bound");
		return PX4_ERROR;
	}


	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	if (strcmp(verb, "start") == 0) {

		instances[bus] = new I2CLauncher(bus);

		if (instances[bus]) {

			if (instances[bus]->init()) {
				return PX4_OK;
			}

		} else {
			PX4_ERR("alloc failed");
		}

		delete instances[bus];

		return PX4_ERROR;
	}

	ThisDriver::print_usage();
	return -1;
}
