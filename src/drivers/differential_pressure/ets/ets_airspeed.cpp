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
 * @file ets_airspeed.cpp
 * @author Simon Wilks
 *
 * Driver for the Eagle Tree Airspeed V3 connected via I2C.
 */

#include <float.h>

#include <drivers/airspeed/airspeed.h>
#include <px4_platform_common/getopt.h>

/* I2C bus address */
#define I2C_ADDRESS	0x75	/* 7-bit address. 8-bit address is 0xEA */
#define ETS_PATH	"/dev/ets_airspeed"

/* Register address */
#define READ_CMD	0x07	/* Read the data */

/**
 * The Eagle Tree Airspeed V3 cannot provide accurate reading below speeds of 15km/h.
 * You can set this value to 12 if you want a zero reading below 15km/h.
 */
#define MIN_ACCURATE_DIFF_PRES_PA 0

/* Measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

class ETSAirspeed : public Airspeed
{
public:
	ETSAirspeed(int bus, int address = I2C_ADDRESS, const char *path = ETS_PATH);

protected:

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void	Run() override;
	int	measure() override;
	int	collect() override;

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ets_airspeed_main(int argc, char *argv[]);

ETSAirspeed::ETSAirspeed(int bus, int address, const char *path) : Airspeed(bus, address,
			CONVERSION_INTERVAL, path)
{
	_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_MS4525;
}

int
ETSAirspeed::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = READ_CMD;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
ETSAirspeed::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		perf_count(_comms_errors);
		return ret;
	}

	float diff_pres_pa_raw = (float)(val[1] << 8 | val[0]);

	differential_pressure_s report;
	report.timestamp = hrt_absolute_time();

	if (diff_pres_pa_raw < FLT_EPSILON) {
		// a zero value indicates no measurement
		// since the noise floor has been arbitrarily killed
		// it defeats our stuck sensor detection - the best we
		// can do is to output some numerical noise to show
		// that we are still correctly sampling.
		diff_pres_pa_raw = 0.001f * (report.timestamp & 0x01);
	}

	// The raw value still should be compensated for the known offset
	diff_pres_pa_raw -= _diff_pres_offset;

	report.error_count = perf_event_count(_comms_errors);

	// XXX we may want to smooth out the readings to remove noise.
	report.differential_pressure_filtered_pa = diff_pres_pa_raw;
	report.differential_pressure_raw_pa = diff_pres_pa_raw;
	report.temperature = -1000.0f;
	report.device_id = _device_id.devid;

	if (_airspeed_pub != nullptr && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(differential_pressure), _airspeed_pub, &report);
	}

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
ETSAirspeed::Run()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
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
		if (_measure_interval > CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

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
	ScheduleDelayed(CONVERSION_INTERVAL);
}

/**
 * Local functions in support of the shell command.
 */
namespace ets_airspeed
{

ETSAirspeed	*g_dev = nullptr;

int start();
int start_bus(int i2c_bus);
int stop();
int reset();
int info();

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;

}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(int i2c_bus)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new ETSAirspeed(i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(AIRSPEED0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
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
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = px4_open(ETS_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

} // namespace


static void
ets_airspeed_usage()
{
	PX4_INFO("usage: ets_airspeed command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|reset|info");
}

int
ets_airspeed_main(int argc, char *argv[])
{
	int i2c_bus = PX4_I2C_BUS_DEFAULT;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			ets_airspeed_usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		ets_airspeed_usage();
		return -1;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return ets_airspeed::start();

		} else {
			return ets_airspeed::start_bus(i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return ets_airspeed::stop();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return ets_airspeed::reset();
	}

	ets_airspeed_usage();
	return 0;
}
