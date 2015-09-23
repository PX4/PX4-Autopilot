/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file meas_airspeed_sim.cpp
 * @author Lorenz Meier
 * @author Sarthak Kaingade
 * @author Simon Wilks
 * @author Thomas Gubler
 * @author Roman Bapst
 *
 * Driver for a simulated airspeed sensor.
 *
 */


#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <systemlib/airspeed.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>

#include "airspeedsim.h"

/* I2C bus address is 1010001x */
#define I2C_ADDRESS_MS4525DO	0x28	/**< 7-bit address. Depends on the order code (this is for code "I") */
#define PATH_MS4525		"/dev/ms4525"
/* The MS5525DSO address is 111011Cx, where C is the complementary value of the pin CSB */
#define I2C_ADDRESS_MS5525DSO	0x77	//0x77/* 7-bit address, addr. pin pulled low */
#define PATH_MS5525		"/dev/ms5525"

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define MEAS_DRIVER_FILTER_FREQ 1.2f
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */

class MEASAirspeedSim : public AirspeedSim
{
public:
	MEASAirspeedSim(int bus, int address = I2C_ADDRESS_MS4525DO, const char *path = PATH_MS4525);

protected:

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	virtual void	cycle();
	virtual int	measure();
	virtual int	collect();

	math::LowPassFilter2p	_filter;

	/**
	 * Correct for 5V rail voltage variations
	 */
	void voltage_correction(float &diff_pres_pa, float &temperature);

	int _t_system_power;
	struct system_power_s system_power;
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int measairspeedsim_main(int argc, char *argv[]);

MEASAirspeedSim::MEASAirspeedSim(int bus, int address, const char *path) : AirspeedSim(bus, address,
			CONVERSION_INTERVAL, path),
	_filter(MEAS_RATE, MEAS_DRIVER_FILTER_FREQ),
	_t_system_power(-1),
	system_power{}
{}

int
MEASAirspeedSim::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = 0;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
MEASAirspeedSim::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	#pragma pack(push, 1)
	struct {
		float		temperature;
		float		diff_pressure;
	} airspeed_report;
	#pragma pack(pop)


	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, (uint8_t *)&airspeed_report, sizeof(airspeed_report));

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint8_t status = 0;

	switch (status) {
	case 0:
		break;

	case 1:

	/* fallthrough */
	case 2:

	/* fallthrough */
	case 3:
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	float temperature = airspeed_report.temperature;
	float diff_press_pa_raw = airspeed_report.diff_pressure * 100.0f; // convert from millibar to bar

	struct differential_pressure_s report;

	/* track maximum differential pressure measured (so we can work out top speed). */
	if (diff_press_pa_raw > _max_differential_pressure_pa) {
		_max_differential_pressure_pa = diff_press_pa_raw;
	}

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = temperature;
	report.differential_pressure_filtered_pa =  _filter.apply(diff_press_pa_raw);

	report.differential_pressure_raw_pa = diff_press_pa_raw;
	report.max_differential_pressure_pa = _max_differential_pressure_pa;

	if (_airspeed_pub != nullptr && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(differential_pressure), _airspeed_pub, &report);
	}

	new_report(report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
MEASAirspeedSim::cycle()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
			/* restart the measurement state machine */
			start();
			_sensor_ok = false;
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&AirspeedSim::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&AirspeedSim::cycle_trampoline,
		   this,
		   USEC2TICK(CONVERSION_INTERVAL));
}

/**
   correct for 5V rail voltage if the system_power ORB topic is
   available

   See http://uav.tridgell.net/MS4525/MS4525-offset.png for a graph of
   offset versus voltage for 3 sensors
 */
void
MEASAirspeedSim::voltage_correction(float &diff_press_pa, float &temperature)
{
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2

	if (_t_system_power == -1) {
		_t_system_power = orb_subscribe(ORB_ID(system_power));
	}

	if (_t_system_power == -1) {
		// not available
		return;
	}

	bool updated = false;
	orb_check(_t_system_power, &updated);

	if (updated) {
		orb_copy(ORB_ID(system_power), _t_system_power, &system_power);
	}

	if (system_power.voltage5V_v < 3.0f || system_power.voltage5V_v > 6.0f) {
		// not valid, skip correction
		return;
	}

	const float slope = 65.0f;
	/*
	  apply a piecewise linear correction, flattening at 0.5V from 5V
	 */
	float voltage_diff = system_power.voltage5V_v - 5.0f;

	if (voltage_diff > 0.5f) {
		voltage_diff = 0.5f;
	}

	if (voltage_diff < -0.5f) {
		voltage_diff = -0.5f;
	}

	diff_press_pa -= voltage_diff * slope;

	/*
	  the temperature masurement varies as well
	 */
	const float temp_slope = 0.887f;
	voltage_diff = system_power.voltage5V_v - 5.0f;

	if (voltage_diff > 0.5f) {
		voltage_diff = 0.5f;
	}

	if (voltage_diff < -1.0f) {
		voltage_diff = -1.0f;
	}

	temperature -= voltage_diff * temp_slope;
#endif // CONFIG_ARCH_BOARD_PX4FMU_V2
}

/**
 * Local functions in support of the shell command.
 */
namespace meas_airspeed_sim
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MEASAirspeedSim	*g_dev = nullptr;

int	start(int i2c_bus);
int	stop();
int test();
int	reset();
int	info();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is up and running
 * or failed to detect the sensor.
 */
int
start(int i2c_bus)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_WARN("already started");
	}

	/* create the driver, try the MS4525DO first */
	g_dev = new MEASAirspeedSim(i2c_bus, I2C_ADDRESS_MS4525DO, PATH_MS4525);

	/* check if the MS4525DO was instantiated */
	if (g_dev == nullptr) {
		goto fail;
	}

	/* try the MS5525DSO next if init fails */
	if (OK != g_dev->AirspeedSim::init()) {
		delete g_dev;
		g_dev = new MEASAirspeedSim(i2c_bus, I2C_ADDRESS_MS5525DSO, PATH_MS5525);

		/* check if the MS5525DSO was instantiated */
		if (g_dev == nullptr) {
			goto fail;
		}

		/* both versions failed if the init for the MS5525DSO fails, give up */
		if (OK != g_dev->AirspeedSim::init()) {
			goto fail;
		}
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(PATH_MS4525, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	return 0;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("no MS4525 airspeedSim sensor connected");
	return 1;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
	}

	return 0;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct differential_pressure_s report;
	ssize_t sz;
	int ret;

	int fd = px4_open(PATH_MS4525, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'meas_airspeed_sim start' if the driver is not running", PATH_MS4525);
			return 1;
	}

	/* do a simple demand read */
	sz = px4_read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return 1;
	}

	PX4_WARN("single read");
	PX4_WARN("diff pressure: %d pa", (int)report.differential_pressure_filtered_pa);

	/* start the sensor polling at 2Hz */
	if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_WARN("failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			warnx("timed out");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_WARN("periodic read failed");
		}

		PX4_WARN("periodic read %u", i);
		PX4_WARN("diff pressure: %d pa", (int)report.differential_pressure_filtered_pa);
		PX4_WARN("temperature: %d C (0x%02x)", (int)report.temperature, (unsigned) report.temperature);
	}

	/* reset the sensor polling to its default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default rate");
		return 1;
	}

	PX4_WARN("PASS");

	return 0;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = open(PATH_MS4525, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed ");
		return 1;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return 1;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return 1;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_WARN("driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

} // namespace


static void
meas_airspeed_usage()
{
	PX4_WARN("usage: meas_airspeed_sim command [options]");
	PX4_WARN("options:");
	PX4_WARN("\t-b --bus i2cbus (%d)", 1);
	PX4_WARN("command:");
	PX4_WARN("\tstart|stop|reset|test|info");
}

int
measairspeedsim_main(int argc, char *argv[])
{
	int i2c_bus = 1;//PX4_I2C_BUS_DEFAULT;

	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bus") == 0) {
			if (argc > i + 1) {
				i2c_bus = atoi(argv[i + 1]);
			}
		}
	}

	int ret;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		ret = meas_airspeed_sim::start(i2c_bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		ret = meas_airspeed_sim::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		ret = meas_airspeed_sim::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		ret = meas_airspeed_sim::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		ret = meas_airspeed_sim::info();
	}

	meas_airspeed_usage();
	return ret;
}
