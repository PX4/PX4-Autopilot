/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file baro.cpp
 * Driver for the simulated barometric pressure sensor
 */

#include <inttypes.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_time.h>
#include <px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <simulator/simulator.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include "barosim.h"
#include "VirtDevObj.hpp"

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

#define BAROSIM_DEV_PATH "/dev/barosim"

#define BAROSIM_MEASURE_INTERVAL_US (10000)

using namespace DriverFramework;

class BAROSIM : public VirtDevObj
{
public:
	BAROSIM(const char *path);
	virtual ~BAROSIM();

	virtual int init() override;

	virtual int devIOCTL(unsigned long cmd, unsigned long arg) override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

protected:

	ringbuffer::RingBuffer	*_reports;

	/* last report */
	struct baro_report	report;

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_orb_class_instance == 0); /* XXX put this into the interface class */ }

	virtual void _measure() override;

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

static BAROSIM *g_barosim = nullptr;

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int barosim_main(int argc, char *argv[]);

BAROSIM::BAROSIM(const char *path) :
	VirtDevObj("BAROSIM", path, BARO_BASE_DEVICE_PATH, BAROSIM_MEASURE_INTERVAL_US),
	_reports(nullptr),
	report{},
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "barosim_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "barosim_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "barosim_comms_errors"))
{

}

BAROSIM::~BAROSIM()
{
	/* make sure we are truly inactive */
	stop();
	setSampleInterval(0);

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
}

int
BAROSIM::init()
{
	int ret;
	struct baro_report brp = {};

	ret = VirtDevObj::init();

	if (ret != OK) {
		PX4_ERR("VirtDevObj init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(baro_report));

	if (_reports == nullptr) {
		PX4_ERR("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}

	_reports->flush();

	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
					  &_orb_class_instance, (is_external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

	if (_baro_topic == nullptr) {
		PX4_ERR("failed to create sensor_baro publication");
		return -ENODEV;
	}

out:
	return ret;
}

int
BAROSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return VirtDevObj::devIOCTL(cmd, arg);
}

void
BAROSIM::_measure()
{
	collect();
}

int
BAROSIM::collect()
{
	bool status;

	simulator::RawBaroData raw_baro;

	perf_begin(_sample_perf);

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	/* read requested */
	Simulator *sim = Simulator::getInstance();

	if (sim == nullptr) {
		PX4_ERR("Error BAROSIM_DEV::transfer no simulator");
		return -ENODEV;
	}

	PX4_DEBUG("BAROSIM_DEV::transfer getting sample");
	status = sim->getBaroSample((uint8_t *)(&raw_baro), sizeof(raw_baro));

	if (!status) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -1;
	}

	report.pressure = raw_baro.pressure;
	report.temperature = raw_baro.temperature;

	/* fake device ID */
	report.device_id = 478459;

	/* publish it */
	if (!(m_pub_blocked)) {
		if (_baro_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);

		} else {
			PX4_WARN("BAROSIM::collect _baro_topic not initialized");
		}
	}

	_reports->force(&report);

	perf_end(_sample_perf);

	return OK;
}

void
BAROSIM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u usec", m_sample_interval_usecs);
	_reports->print_info("report queue");
	PX4_INFO("TEMP:           %f", (double)report.temperature);
	PX4_INFO("P:              %.3f", (double)report.pressure);
}

namespace barosim
{

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
static int
start()
{
	g_barosim = new BAROSIM(BAROSIM_DEV_PATH);

	if (g_barosim != nullptr && OK != g_barosim->init()) {
		delete g_barosim;
		g_barosim = nullptr;
		PX4_ERR("bus init failed");
		return false;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
static int
info()
{
	if (g_barosim != nullptr) {
		PX4_INFO("%s", BAROSIM_DEV_PATH);
		g_barosim->print_info();
	}

	return 0;
}

static void
usage()
{
	PX4_WARN("missing command: try 'start', 'info'");
}

}; // namespace barosim

int
barosim_main(int argc, char *argv[])
{
	int ret;

	if (argc < 2) {
		barosim::usage();
		return 1;
	}

	const char *verb = argv[1];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ret = barosim::start();
	}

	/*
	 * Print driver information.
	 */
	else if (!strcmp(verb, "info")) {
		ret = barosim::info();

	} else {
		barosim::usage();
		return 1;
	}

	return ret;
}
