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
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <uORB/topics/sensor_differential_pressure.h>
#include <uORB/PublicationMulti.hpp>

#include <float.h>

/* I2C bus address */
#define I2C_ADDRESS	0x75	/* 7-bit address. 8-bit address is 0xEA */

/* Register address */
#define READ_CMD	0x07	/* Read the data */

/**
 * The Eagle Tree Airspeed V3 cannot provide accurate reading below speeds of 15km/h.
 * You can set this value to 12 if you want a zero reading below 15km/h.
 */
#define MIN_ACCURATE_DIFF_PRES_PA 0

/* Measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

class ETSAirspeed : public device::I2C, public I2CSPIDriver<ETSAirspeed>
{
public:
	ETSAirspeed(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = I2C_ADDRESS);

	virtual ~ETSAirspeed();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void	RunImpl();

private:
	int	probe() override;

	int	measure();
	int	collect();

	bool _sensor_ok{false};
	int _measure_interval{0};
	bool _collect_phase{false};
	unsigned _conversion_interval{0};

	uORB::PublicationMulti<sensor_differential_pressure_s>	_differential_pressure_pub{ORB_ID(sensor_differential_pressure)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};
};

ETSAirspeed::ETSAirspeed(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
	I2C(DRV_DIFF_PRESS_DEVTYPE_ETS3, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
}

ETSAirspeed::~ETSAirspeed()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int ETSAirspeed::probe()
{
	uint8_t cmd = READ_CMD;
	int ret = transfer(&cmd, 1, nullptr, 0);

	return ret;
}

int ETSAirspeed::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = READ_CMD;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int ETSAirspeed::collect()
{
	/* read from the sensor */
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	uint8_t val[2] {};
	int ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		perf_count(_comms_errors);
		return ret;
	}

	float diff_press_pa_raw = (float)(val[1] << 8 | val[0]);

	if (diff_press_pa_raw < FLT_EPSILON) {
		// a zero value indicates no measurement
		// since the noise floor has been arbitrarily killed
		// it defeats our stuck sensor detection - the best we
		// can do is to output some numerical noise to show
		// that we are still correctly sampling.
		diff_press_pa_raw = 0.001f * (hrt_absolute_time() & 0x01);
	}

	sensor_differential_pressure_s report{};
	report.timestamp_sample = timestamp_sample;
	report.device_id = get_device_id();
	report.differential_pressure_pa = diff_press_pa_raw;
	report.temperature = NAN;
	report.error_count = perf_event_count(_comms_errors);
	report.timestamp = hrt_absolute_time();
	_differential_pressure_pub.publish(report);

	perf_end(_sample_perf);

	return PX4_OK;
}

void ETSAirspeed::RunImpl()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
			/* restart the measurement state machine */
			_collect_phase = false;
			_sensor_ok = false;
			ScheduleNow();
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

I2CSPIDriverBase *ETSAirspeed::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	ETSAirspeed *instance = new ETSAirspeed(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->ScheduleNow();
	return instance;
}

void ETSAirspeed::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ets_airspeed", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int ets_airspeed_main(int argc, char *argv[])
{
	using ThisDriver = ETSAirspeed;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_ETS3);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
