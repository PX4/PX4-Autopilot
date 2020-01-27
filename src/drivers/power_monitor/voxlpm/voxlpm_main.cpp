/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/module.h>

#include "voxlpm.hpp"

enum VOXLPM_BUS {
	VOXLPM_BUS_I2C_EXTERNAL,
	VOXLPM_BUS_I2C_EXTERNAL1,
	VOXLPM_BUS_I2C_EXTERNAL2
};

/**
 * Local functions in support of the shell command.
 */
namespace voxlpm
{
struct voxlpm_chan {
	const char *devpath;
	uint32_t device;
	VOXLPM *dev;
};

struct voxlpm_bus_option {
	enum VOXLPM_BUS busid;
	uint8_t busnum;
	bool external;
	struct voxlpm_chan vbat;
	struct voxlpm_chan vpwr;
} bus_options[] = {
#if defined(PX4_I2C_BUS_EXPANSION)
	{
		VOXLPM_BUS_I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION, true,
		{"/dev/voxlpm_vbat", VOXLPM_LTC2946_ADDR_VBATT, NULL },
		{"/dev/voxlpm_p5vd", VOXLPM_LTC2946_ADDR_P5VD, NULL}
	},
#endif
#if defined(PX4_I2C_BUS_EXPANSION1)
	{
		VOXLPM_BUS_I2C_EXTERNAL1, PX4_I2C_BUS_EXPANSION1, true,
		{"/dev/voxlpm_vbat", VOXLPM_LTC2946_ADDR_VBATT, NULL },
		{"/dev/voxlpm_p5vd", VOXLPM_LTC2946_ADDR_P5VD, NULL}
	},
#endif
#if defined(PX4_I2C_BUS_EXPANSION2)
	{
		VOXLPM_BUS_I2C_EXTERNAL2, PX4_I2C_BUS_EXPANSION2, true,
		{"/dev/voxlpm_vbat", VOXLPM_LTC2946_ADDR_VBATT, NULL },
		{"/dev/voxlpm_p5vd", VOXLPM_LTC2946_ADDR_P5VD, NULL}
	},
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))


/**
 * Start the driver.
 */
bool
start_bus(struct voxlpm_bus_option &bus)
{
	/* assume if we've got the battery channel we have the output channel as well */
	if (bus.vbat.dev != nullptr) {
		PX4_ERR("bus option already started");
		exit(1);
	}

	/* create the battery voltage / current channel */
	bus.vbat.dev = new VOXLPM(bus.vbat.devpath, bus.busnum, bus.vbat.device, VOXLPM_CH_TYPE_VBATT);

	if (bus.vbat.dev == nullptr) {
		return false;
	}

	/* create the 5VDC output / compute current channel */
	bus.vpwr.dev = new VOXLPM(bus.vpwr.devpath, bus.busnum, bus.vpwr.device, VOXLPM_CH_TYPE_P5VDC);

	if (bus.vbat.dev == nullptr) {
		return false;
	}

	if (bus.vbat.dev->init() != OK || bus.vpwr.dev->init() != OK) {
		delete bus.vbat.dev;
		bus.vbat.dev = nullptr;
		delete bus.vpwr.dev;
		bus.vpwr.dev = nullptr;
		return false;
	}

	return true;
}

void
start(enum VOXLPM_BUS busid)
{
	bool started = false;
	uint8_t i;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].busid == busid) {
			started = start_bus(bus_options[i]);
			break; // one bus only...
		}
	}

	if (!started) {
		PX4_WARN("bus option number is %d", i);
		PX4_ERR("driver start failed");
		exit(1);
	}

	exit(0);
}

void
info()
{
	uint8_t i;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].vbat.dev) {
			bus_options[i].vbat.dev->print_info();
		}

		if (bus_options[i].vpwr.dev) {
			bus_options[i].vpwr.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	PRINT_MODULE_USAGE_NAME_SIMPLE("voxlpm", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "start monitoring");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "display info");;
	PRINT_MODULE_USAGE_COMMAND_DESCR("-X", "PX4_I2C_BUS_EXPANSION");
	PRINT_MODULE_USAGE_COMMAND_DESCR("-T", "PX4_I2C_BUS_EXPANSION1");
	PRINT_MODULE_USAGE_COMMAND_DESCR("-R", "PX4_I2C_BUS_EXPANSION2 (default)");
}

} // namespace

extern "C" __EXPORT int voxlpm_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	enum VOXLPM_BUS busid = VOXLPM_BUS_I2C_EXTERNAL2;

	while ((ch = px4_getopt(argc, argv, "XTR", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = VOXLPM_BUS_I2C_EXTERNAL;
			break;

		case 'T':
			busid = VOXLPM_BUS_I2C_EXTERNAL1;
			break;

		case 'R':
			busid = VOXLPM_BUS_I2C_EXTERNAL2;
			break;

		default:
			voxlpm::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		voxlpm::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		voxlpm::start(busid);
	}

	if (!strcmp(verb, "info")) {
		voxlpm::info();
	}

	voxlpm::usage();
	return -1;
}
