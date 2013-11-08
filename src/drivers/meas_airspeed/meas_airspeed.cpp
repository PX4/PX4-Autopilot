/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file meas_airspeed.cpp
 * @author Lorenz Meier
 * @author Sarthak Kaingade
 * @author Simon Wilks
 *
 * Driver for the MEAS Spec series connected via I2C.
 *
 * Supported sensors:
 *
 *    - MS4525DO (http://www.meas-spec.com/downloads/MS4525DO.pdf)
 *    - untested: MS5525DSO (http://www.meas-spec.com/downloads/MS5525DSO.pdf)
 *
 * Interface application notes:
 *
 *    - Interfacing to MEAS Digital Pressure Modules (http://www.meas-spec.com/downloads/Interfacing_to_MEAS_Digital_Pressure_Modules.pdf)
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/airspeed.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/subsystem_info.h>

#include <drivers/airspeed/airspeed.h>

/* I2C bus address is 1010001x */
#define I2C_ADDRESS_MS4525DO	0x28	//0x51 /* 7-bit address. */
/* The MS5525DSO address is 111011Cx, where C is the complementary value of the pin CSB */
#define I2C_ADDRESS_MS5525DSO	0x77	//0x77/* 7-bit address, addr. pin pulled low */

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

class MEASAirspeed : public Airspeed
{
public:
	MEASAirspeed(int bus, int address = I2C_ADDRESS_MS4525DO);

protected:

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	virtual void	cycle();
	virtual int	measure();
	virtual int	collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int meas_airspeed_main(int argc, char *argv[]);

MEASAirspeed::MEASAirspeed(int bus, int address) : Airspeed(bus, address,
			CONVERSION_INTERVAL)
{

}

int
MEASAirspeed::measure()
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
MEASAirspeed::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[4] = {0, 0, 0, 0};


	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 4);

	if (ret < 0) {
                perf_count(_comms_errors);
                perf_end(_sample_perf);
		return ret;
	}

	uint8_t status = val[0] & 0xC0;

	if (status == 2) {
		log("err: stale data");
                perf_count(_comms_errors);
                perf_end(_sample_perf);
		return ret;
	} else if (status == 3) {
		log("err: fault");
                perf_count(_comms_errors);
                perf_end(_sample_perf);
		return ret;                
	}

	//uint16_t diff_pres_pa = (val[1]) | ((val[0] & ~(0xC0)) << 8);
	uint16_t temp = (val[3] & 0xE0) << 8 | val[2];

	// XXX leaving this in until new calculation method has been cross-checked
	//diff_pres_pa = abs(diff_pres_pa - (16384 / 2.0f));
	//diff_pres_pa -= _diff_pres_offset;
	int16_t dp_raw = 0, dT_raw = 0;
	dp_raw = (val[0] << 8) + val[1];
	dp_raw = 0x3FFF & dp_raw;
	dT_raw = (val[2] << 8) + val[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;
	float temperature = ((200 * dT_raw) / 2047) - 50;

	// XXX we may want to smooth out the readings to remove noise.

	// Calculate differential pressure. As its centered around 8000
	// and can go positive or negative, enforce absolute value
	uint16_t diff_press_pa = abs(dp_raw - (16384 / 2.0f));
	struct differential_pressure_s report;

	// Track maximum differential pressure measured (so we can work out top speed).
	if (diff_press_pa > _max_differential_pressure_pa) {
	    _max_differential_pressure_pa = diff_press_pa;
	}

	report.timestamp = hrt_absolute_time();
        report.error_count = perf_event_count(_comms_errors);
	report.temperature = temperature;
	report.differential_pressure_pa = diff_press_pa;
	report.voltage = 0;
	report.max_differential_pressure_pa = _max_differential_pressure_pa;

	/* announce the airspeed if needed, just publish else */
	orb_publish(ORB_ID(differential_pressure), _airspeed_pub, &report);

	new_report(report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
MEASAirspeed::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			/* restart the measurement state machine */
			start();
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
				   (worker_t)&Airspeed::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&Airspeed::cycle_trampoline,
		   this,
		   USEC2TICK(CONVERSION_INTERVAL));
}

/**
 * Local functions in support of the shell command.
 */
namespace meas_airspeed
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MEASAirspeed	*g_dev = nullptr;

void	start(int i2c_bus);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(int i2c_bus)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver, try the MS4525DO first */
	g_dev = new MEASAirspeed(i2c_bus, I2C_ADDRESS_MS4525DO);

	/* check if the MS4525DO was instantiated */
	if (g_dev == nullptr)
		goto fail;

	/* try the MS5525DSO next if init fails */
	if (OK != g_dev->Airspeed::init()) {
		delete g_dev;
		g_dev = new MEASAirspeed(i2c_bus, I2C_ADDRESS_MS5525DSO);

		/* check if the MS5525DSO was instantiated */
		if (g_dev == nullptr)
			goto fail;

		/* both versions failed if the init for the MS5525DSO fails, give up */
		if (OK != g_dev->Airspeed::init())
			goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct differential_pressure_s report;
	ssize_t sz;
	int ret;

	int fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'meas_airspeed start' if the driver is not running", AIRSPEED_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("diff pressure: %d pa", report.differential_pressure_pa);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("diff pressure: %d pa", report.differential_pressure_pa);
		warnx("temperature: %d C (0x%02x)", (int)report.temperature, (unsigned) report.temperature);
	}

	/* reset the sensor polling to its default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT))
		errx(1, "failed to set default rate");

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace


static void
meas_airspeed_usage()
{
	warnx("usage: meas_airspeed command [options]");
	warnx("options:");
	warnx("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	warnx("command:");
	warnx("\tstart|stop|reset|test|info");
}

int
meas_airspeed_main(int argc, char *argv[])
{
	int i2c_bus = PX4_I2C_BUS_DEFAULT;

	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bus") == 0) {
			if (argc > i + 1) {
				i2c_bus = atoi(argv[i + 1]);
			}
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		meas_airspeed::start(i2c_bus);

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop"))
		meas_airspeed::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		meas_airspeed::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		meas_airspeed::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		meas_airspeed::info();

	meas_airspeed_usage();
	exit(0);
}
