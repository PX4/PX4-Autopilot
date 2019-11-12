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

/**
 * Driver for the standalone AK09916 magnetometer.
 */

#include <px4_config.h>
#include <px4_log.h>
#include <px4_time.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <px4_getopt.h>

#include "ak09916.hpp"


extern "C" { __EXPORT int ak09916_main(int argc, char *argv[]); }


namespace ak09916
{

AK09916 *g_dev_ext;
AK09916 *g_dev_int;

void start(bool, enum Rotation);
void info(bool);
void usage();

void start(bool external_bus, enum Rotation rotation)
{
	AK09916 **g_dev_ptr = (external_bus ? &g_dev_ext : &g_dev_int);
	const char *path = (external_bus ? AK09916_DEVICE_PATH_MAG_EXT : AK09916_DEVICE_PATH_MAG);

	if (*g_dev_ptr != nullptr) {
		PX4_ERR("already started");
		exit(0);
	}

	if (external_bus) {
#if defined(PX4_I2C_BUS_EXPANSION)
		*g_dev_ptr = new AK09916(PX4_I2C_BUS_EXPANSION, path, rotation);
#else
		PX4_ERR("External I2C not available");
		exit(0);
#endif

	} else {
		PX4_ERR("Internal I2C not available");
		exit(0);
	}

	if (*g_dev_ptr == nullptr || (OK != (*g_dev_ptr)->init())) {
		PX4_ERR("driver start failed");
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
		exit(1);
	}

	exit(0);
}

void
stop(bool external_bus)
{
	AK09916 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	(*g_dev_ptr)->stop();

	exit(0);
}

void
info(bool external_bus)
{
	AK09916 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', stop'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R    (rotation)");
}

} // namespace ak09916

AK09916::AK09916(int bus, const char *path, enum Rotation rotation) :
	I2C("AK09916", path, bus, AK09916_I2C_ADDR, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_mag(get_device_id(), ORB_PRIO_MAX, rotation),
	_mag_reads(perf_alloc(PC_COUNT, "ak09916_mag_reads")),
	_mag_errors(perf_alloc(PC_COUNT, "ak09916_mag_errors")),
	_mag_overruns(perf_alloc(PC_COUNT, "ak09916_mag_overruns")),
	_mag_overflows(perf_alloc(PC_COUNT, "ak09916_mag_overflows"))
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_AK09916);
	_px4_mag.set_scale(AK09916_MAG_RANGE_GA);
}

AK09916::~AK09916()
{
	perf_free(_mag_reads);
	perf_free(_mag_errors);
	perf_free(_mag_overruns);
	perf_free(_mag_overflows);
}

int
AK09916::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		PX4_WARN("AK09916 mag init failed");
		return ret;
	}

	ret = reset();

	if (ret != PX4_OK) {
		return ret;
	}

	start();

	return PX4_OK;
}

void
AK09916::try_measure()
{
	if (!is_ready()) {
		return;
	}

	measure();
}

bool
AK09916::is_ready()
{
	uint8_t st1;
	const int ret = transfer(&AK09916REG_ST1, sizeof(AK09916REG_ST1), &st1, sizeof(st1));

	if (ret != OK) {
		return false;
	}

	// Monitor if data overrun flag is ever set.
	if (st1 & AK09916_ST1_DOR) {
		perf_count(_mag_overruns);
	}

	return (st1 & AK09916_ST1_DRDY);
}

void
AK09916::measure()
{
	ak09916_regs regs;

	const hrt_abstime now = hrt_absolute_time();

	const int ret = transfer(&AK09916REG_HXL, sizeof(AK09916REG_HXL),
				 reinterpret_cast<uint8_t *>(&regs), sizeof(regs));

	if (ret != OK) {
		_px4_mag.set_error_count(perf_event_count(_mag_errors));
		return;
	}

	// Monitor if magnetic sensor overflow flag is set.
	if (regs.st2 & AK09916_ST2_HOFL) {
		perf_count(_mag_overflows);
	}

	_px4_mag.set_external(external());
	_px4_mag.update(now, regs.x, regs.y, regs.z);
}

void
AK09916::print_info()
{
	perf_print_counter(_mag_reads);
	perf_print_counter(_mag_errors);
	perf_print_counter(_mag_overruns);
	_px4_mag.print_status();
}

uint8_t
AK09916::read_reg(uint8_t reg)
{
	const uint8_t cmd = reg;
	uint8_t ret{};

	transfer(&cmd, 1, &ret, 1);

	return ret;
}

bool
AK09916::check_id()
{
	const uint8_t deviceid = read_reg(AK09916REG_WIA);

	return (AK09916_DEVICE_ID_A == deviceid);
}

void
AK09916::write_reg(uint8_t reg, uint8_t value)
{
	const uint8_t cmd[2] = { reg, value};
	transfer(cmd, 2, nullptr, 0);
}

int
AK09916::reset()
{
	int rv = probe();

	if (rv == OK) {
		// Now reset the mag.
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		// Then re-initialize the bus/mag.
		rv = setup();
	}

	return rv;
}

int
AK09916::probe()
{
	int retries = 10;

	do {
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		if (check_id()) {
			return OK;
		}

		retries--;
	} while (retries > 0);

	return PX4_ERROR;
}

int
AK09916::setup()
{
	write_reg(AK09916REG_CNTL2, AK09916_CNTL2_CONTINOUS_MODE_100HZ);

	return OK;
}

void
AK09916::start()
{
	_cycle_interval = AK09916_CONVERSION_INTERVAL_us;

	ScheduleNow();
}

void
AK09916::stop()
{
	// Ensure no new items are queued while we cancel this one.
	_cycle_interval = 0;

	ScheduleClear();
}

void
AK09916::Run()
{
	if (_cycle_interval == 0) {
		return;
	}

	try_measure();

	if (_cycle_interval > 0) {
		ScheduleDelayed(_cycle_interval);
	}
}

int
ak09916_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			ak09916::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		ak09916::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ak09916::start(external_bus, rotation);
	}

	if (!strcmp(verb, "stop")) {
		ak09916::stop(external_bus);
	}

	if (!strcmp(verb, "info")) {
		ak09916::info(external_bus);
	}

	ak09916::usage();
	return -1;
}
