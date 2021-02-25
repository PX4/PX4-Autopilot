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

#include "MS4525.hpp"

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */

MS4525::MS4525(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
	I2C(DRV_DIFF_PRESS_DEVTYPE_MS4525, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
}

MS4525::~MS4525()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int MS4525::probe()
{
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);

	return ret;
}

int MS4525::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int MS4525::collect()
{
	/* read from the sensor */
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	uint8_t val[4] {};
	int ret = transfer(nullptr, 0, &val[0], 4);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint8_t status = (val[0] & 0xC0) >> 6;

	switch (status) {
	case 0:
		// Normal Operation. Good Data Packet
		break;

	case 1:
		// Reserved
		return -EAGAIN;

	case 2:
		// Stale Data. Data has been fetched since last measurement cycle.
		return -EAGAIN;

	case 3:
		// Fault Detected
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	/* mask the used bits */
	int16_t dp_raw = (0x3FFF & ((val[0] << 8) + val[1]));
	int16_t dT_raw = (0xFFE0 & ((val[2] << 8) + val[3])) >> 5;

	// dT max is almost certainly an invalid reading
	if (dT_raw == 2047) {
		perf_count(_comms_errors);
		return -EAGAIN;
	}

	// only publish changes
	if ((dp_raw != _dp_raw_prev) || (dT_raw != _dT_raw_prev)) {

		_dp_raw_prev = dp_raw;
		_dT_raw_prev = dT_raw;

		float temperature = ((200.0f * dT_raw) / 2047) - 50;

		// Calculate differential pressure. As its centered around 8000
		// and can go positive or negative
		const float P_min = -1.0f;
		const float P_max = 1.0f;
		const float PSI_to_Pa = 6894.757f;
		/*
		this equation is an inversion of the equation in the
		pressure transfer function figure on page 4 of the datasheet

		We negate the result so that positive differential pressures
		are generated when the bottom port is used as the static
		port on the pitot and top port is used as the dynamic port
		*/
		float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
		float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;

		/*
		With the above calculation the MS4525 sensor will produce a
		positive number when the top port is used as a dynamic port
		and bottom port is used as the static port
		*/
		sensor_differential_pressure_s report{};
		report.timestamp_sample = timestamp_sample;
		report.device_id = get_device_id();
		report.differential_pressure_pa = diff_press_pa_raw;
		report.temperature = temperature;
		report.error_count = perf_event_count(_comms_errors);
		report.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(report);
	}

	perf_end(_sample_perf);

	return PX4_OK;
}

void MS4525::RunImpl()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
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

	if (PX4_OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(CONVERSION_INTERVAL);
}

I2CSPIDriverBase *MS4525::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	MS4525 *instance = new MS4525(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

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

void MS4525::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ms4525", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int ms4525_main(int argc, char *argv[])
{
	using ThisDriver = MS4525;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = I2C_ADDRESS_MS4525DO;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_MS4525);

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
