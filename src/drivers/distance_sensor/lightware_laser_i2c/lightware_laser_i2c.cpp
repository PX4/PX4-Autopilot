/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file lightware_laser_i2c.cpp
 *
 * @author ecmnet <ecm@gmx.de>
 * @author Vasily Evseenko <svpcom@gmail.com>
 *
 * Driver for the Lightware lidar range finder series.
 * Default I2C address 0x66 is used.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

/* Configuration Constants */
#define LIGHTWARE_LASER_BASEADDR		0x66

class LightwareLaser : public device::I2C, public I2CSPIDriver<LightwareLaser>, public ModuleParams
{
public:
	LightwareLaser(const I2CSPIDriverConfig &config);

	~LightwareLaser() override;

	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();

private:
	// I2C (legacy) binary protocol command
	static constexpr uint8_t I2C_LEGACY_CMD_READ_ALTITUDE = 0;
	static constexpr uint8_t I2C_LEGACY_CMD_WRITE_LASER_CONTROL = 5;

	enum class Register : uint8_t {
		// see http://support.lightware.co.za/sf20/#/commands
		ProductName = 0,
		DistanceOutput = 27,
		DistanceData = 44,
		LaserFiring = 50,
		MeasurementMode = 93,
		ZeroOffset = 94,
		LostSignalCounter = 95,
		Protocol = 120,
		ServoConnected = 121,
	};

	static constexpr uint16_t output_data_config = 0b11010110100;
	struct OutputData {
		int16_t first_return_median;
		int16_t first_return_strength;
		int16_t last_return_raw;
		int16_t last_return_median;
		int16_t last_return_strength;
		int16_t background_noise;
	};

	enum class Type {
		Generic = 0,
		LW20c
	};
	enum class State {
		Configuring,
		Running
	};

	int probe() override;

	void start();

	int readRegister(Register reg, uint8_t *data, int len);

	int configure();
	int enableI2CBinaryProtocol();
	int collect();

	int updateRestriction();

	PX4Rangefinder _px4_rangefinder;

	int _conversion_interval{-1};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};

	Type _type{Type::Generic};
	State _state{State::Configuring};
	int _consecutive_errors{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_SF1XX>) _param_sens_en_sf1xx,
		(ParamInt<px4::params::SF1XX_MODE>) _param_sf1xx_mode
	)
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	typeof(px4::msg::VehicleStatus::vehicle_type) _vehicle_type{px4::msg::VehicleStatus::VEHICLE_TYPE_UNKNOWN};
	bool _restriction{false};
	bool _auto_restriction{false};
};

LightwareLaser::LightwareLaser(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LIGHTWARE_LASER);
}

LightwareLaser::~LightwareLaser()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int LightwareLaser::init()
{
	int ret = PX4_ERROR;
	updateParams();
	const int32_t hw_model = _param_sens_en_sf1xx.get();

	switch (hw_model) {
	case 0:
		PX4_WARN("disabled.");
		return ret;

	case 1:  /* SF10/a (25m 32Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(25.0f);
		_conversion_interval = 31250;
		break;

	case 2:  /* SF10/b (50m 32Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(50.0f);
		_conversion_interval = 31250;
		break;

	case 3:  /* SF10/c (100m 16Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(100.0f);
		_conversion_interval = 62500;
		break;

	case 4:
		/* SF11/c (120m 20Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(120.0f);
		_conversion_interval = 50000;
		break;

	case 5:
		/* SF/LW20/b (50m 48-388Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(50.0f);
		_conversion_interval = 20834;
		break;

	case 6:
		/* SF/LW20/c (100m 48-388Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(100.0f);
		_conversion_interval = 20834;
		_type = Type::LW20c;
		break;

	case 7:
		/* SF/LW30/d (200m 49-20'000Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(200.0f);
		_conversion_interval = 20409;
		break;

	default:
		PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
		return ret;
	}

	/* do I2C init (and probe) first */
	ret = I2C::init();

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int LightwareLaser::readRegister(Register reg, uint8_t *data, int len)
{
	const uint8_t cmd = (uint8_t)reg;
	return transfer(&cmd, 1, data, len);
}

int LightwareLaser::probe()
{
	switch (_type) {

	case Type::Generic: {
			uint8_t cmd = I2C_LEGACY_CMD_READ_ALTITUDE;
			return transfer(&cmd, 1, nullptr, 0);
		}

	case Type::LW20c:
		// try to enable I2C binary protocol
		int ret = enableI2CBinaryProtocol();

		if (ret != 0) {
			return ret;
		}

		// read the product name
		uint8_t product_name[16];
		ret = readRegister(Register::ProductName, product_name, sizeof(product_name));
		product_name[sizeof(product_name) - 1] = '\0';
		PX4_DEBUG("product: %s", product_name);

		if (ret == 0 && (strncmp((const char *)product_name, "SF20", sizeof(product_name)) == 0 ||
				 strncmp((const char *)product_name, "LW20", sizeof(product_name)) == 0)) {
			return 0;
		}

		return -1;
	}

	return -1;
}

int LightwareLaser::enableI2CBinaryProtocol()
{
	const uint8_t cmd[] = {(uint8_t)Register::Protocol, 0xaa, 0xaa};
	int ret = transfer(cmd, sizeof(cmd), nullptr, 0);

	if (ret != 0) {
		return ret;
	}

	// Now read and check against the expected values
	for (int i = 0; i < 2; ++i) {
		uint8_t value[2];
		ret = transfer(cmd, 1, value, sizeof(value));

		if (ret != 0) {
			return ret;
		}

		PX4_DEBUG("protocol values: 0x%" PRIx8 " 0x%" PRIx8, value[0], value[1]);

		if (value[0] == 0xcc && value[1] == 0x00) {
			return 0;
		}

		// Occasionally the previous transfer returns ret == value[0] == value[1] == 0. If so, wait a bit and retry
		px4_usleep(1000);
	}

	return -1;
}

int LightwareLaser::configure()
{
	switch (_type) {
	case Type::Generic: {
			uint8_t cmd1 = I2C_LEGACY_CMD_READ_ALTITUDE;
			int ret = transfer(&cmd1, 1, nullptr, 0);
			const uint8_t cmd2[] = {I2C_LEGACY_CMD_WRITE_LASER_CONTROL, (uint8_t)(_restriction ? 0 : 1)};
			ret |= transfer(cmd2, sizeof(cmd2), nullptr, 0);

			if (PX4_OK != ret) {
				perf_count(_comms_errors);
				PX4_DEBUG("i2c::transfer returned %d", ret);
				return ret;
			}

			return ret;
		}
		break;

	case Type::LW20c:

		int ret = enableI2CBinaryProtocol();
		const uint8_t cmd1[] = {(uint8_t)Register::ServoConnected, 0};
		ret |= transfer(cmd1, sizeof(cmd1), nullptr, 0);
		const uint8_t cmd2[] = {(uint8_t)Register::ZeroOffset, 0, 0, 0, 0};
		ret |= transfer(cmd2, sizeof(cmd2), nullptr, 0);
		const uint8_t cmd3[] = {(uint8_t)Register::MeasurementMode, 1}; // 48Hz
		ret |= transfer(cmd3, sizeof(cmd3), nullptr, 0);
		const uint8_t cmd4[] = {(uint8_t)Register::DistanceOutput, output_data_config & 0xff, (output_data_config >> 8) & 0xff, 0, 0};
		ret |= transfer(cmd4, sizeof(cmd4), nullptr, 0);
		const uint8_t cmd5[] = {(uint8_t)Register::LostSignalCounter, 0, 0, 0, 0}; // immediately report lost signal
		ret |= transfer(cmd5, sizeof(cmd5), nullptr, 0);
		const uint8_t cmd6[] = {(uint8_t)Register::LaserFiring, (uint8_t)(_restriction ? 0 : 1)};
		ret |= transfer(cmd6, sizeof(cmd6), nullptr, 0);

		return ret;
		break;
	}

	return -1;
}

int LightwareLaser::collect()
{
	switch (_type) {
	case Type::Generic: {
			/* read from the sensor */
			perf_begin(_sample_perf);
			uint8_t val[2] {};
			const hrt_abstime timestamp_sample = hrt_absolute_time();

			if (transfer(nullptr, 0, &val[0], 2) < 0) {
				perf_count(_comms_errors);
				perf_end(_sample_perf);
				return PX4_ERROR;
			}

			perf_end(_sample_perf);

			uint16_t distance_cm = val[0] << 8 | val[1];
			float distance_m = float(distance_cm) * 1e-2f;

			_px4_rangefinder.update(timestamp_sample, distance_m);
			break;
		}

	case Type::LW20c:

		/* read from the sensor */
		perf_begin(_sample_perf);
		OutputData data;
		const hrt_abstime timestamp_sample = hrt_absolute_time();

		if (readRegister(Register::DistanceData, (uint8_t *)&data, sizeof(data)) < 0) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return PX4_ERROR;
		}

		perf_end(_sample_perf);

		// compare different outputs (median filter adds about 25ms delay)
		PX4_DEBUG("fm: %4" PRIi16 ", fs: %2" PRIi16 "%%, lm: %4" PRIi16 ", lr: %4" PRIi16 ", fs: %2" PRIi16 "%%, n: %" PRIi16,
			  data.first_return_median, data.first_return_strength, data.last_return_median, data.last_return_raw,
			  data.last_return_strength, data.background_noise);

		float distance_m = float(data.last_return_raw) * 1e-2f;
		int8_t quality = data.last_return_strength;

		_px4_rangefinder.update(timestamp_sample, distance_m, quality);
		break;
	}

	return PX4_OK;
}

void LightwareLaser::start()
{
	/* schedule a cycle to start things */
	ScheduleDelayed(_conversion_interval);
}

int LightwareLaser::updateRestriction()
{
	px4::msg::VehicleStatus vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		// Check if vehicle type changed
		if (vehicle_status.vehicle_type != _vehicle_type) {
			// Transition VTOL -> Fixed Wing
			if (_vehicle_type == px4::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING &&
			    vehicle_status.vehicle_type == px4::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING) {
				_auto_restriction = true;
			}

			// Transition Fixed Wing -> VTOL
			else if (_vehicle_type == px4::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING &&
				 vehicle_status.vehicle_type == px4::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING) {
				_auto_restriction = false;
			}

			_vehicle_type = vehicle_status.vehicle_type;
		}
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	bool _prev_restriction{_restriction};

	switch (_param_sf1xx_mode.get()) {
	case 0: // Sensor disabled
		_restriction = true;
		break;

	case 1: // Sensor enabled
	default:
		_restriction = false;
		break;

	case 2:
		_restriction = _auto_restriction;
		break;
	}

	if (_prev_restriction != _restriction) {
		PX4_INFO("Emission Control: %sabling sensor!", _restriction ? "dis" : "en");

		switch (_type) {
		case Type::Generic: {
				const uint8_t cmd[] = {I2C_LEGACY_CMD_WRITE_LASER_CONTROL, (uint8_t)(_restriction ? 0 : 1)};
				return transfer(cmd, sizeof(cmd), nullptr, 0);
			}

		case Type::LW20c: {
				const uint8_t cmd[] = {(uint8_t)Register::LaserFiring, (uint8_t)(_restriction ? 0 : 1)};
				return transfer(cmd, sizeof(cmd), nullptr, 0);
			}
		}
	}

	return 0;
}

void LightwareLaser::RunImpl()
{
	if (PX4_OK != updateRestriction()) {
		PX4_DEBUG("restriction error");
		perf_count(_comms_errors);
	}

	switch (_state) {
	case State::Configuring: {
			if (configure() == 0) {
				_state = State::Running;
				ScheduleDelayed(_conversion_interval);

			} else {
				// retry after a while
				PX4_DEBUG("Retrying...");
				ScheduleDelayed(300_ms);
			}

			break;
		}

	case State::Running:
		if (!_restriction) {
			if (PX4_OK != collect()) {
				PX4_DEBUG("collection error");

				if (++_consecutive_errors > 3) {
					_state = State::Configuring;
					_consecutive_errors = 0;
				}
			}
		}

		ScheduleDelayed(_conversion_interval);
		break;
	}
}

void LightwareLaser::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void LightwareLaser::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for Lightware SFxx series LIDAR rangefinders: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/main/en/sensor/sfxx_lidar.html
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lightware_laser_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x66);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int lightware_laser_i2c_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = LightwareLaser;
	BusCLIArguments cli{true, false};
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = LIGHTWARE_LASER_BASEADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_LIGHTWARE_LASER);

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
