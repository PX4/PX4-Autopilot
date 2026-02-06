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
 * @author Aaron Porter <aaron.porter@ascendengineer.com>
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
#include <uORB/topics/distance_sensor_mode_change_request.h>

using namespace time_literals;

/* Configuration Constants */
#define LIGHTWARE_LASER_BASEADDR		0x66
#define PAYLOAD_LENGTH				64

class LightwareGRF : public device::I2C, public I2CSPIDriver<LightwareGRF>, public ModuleParams
{
public:
	LightwareGRF(const I2CSPIDriverConfig &config);

	~LightwareGRF() override;

	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();

private:
	enum class Register : uint8_t {
		// Common registers
		ProductName = 0,
		DistanceData = 44,
		LaserFiring = 50,
		UpdateRate = 74,
		Protocol = 120,
	};

	enum class RegisterGRF : uint8_t {
		DistanceOutput = 27,
		MeasurementMode = 93,
		ZeroOffset = 94,
		LostSignalCounter = 95,
		ServoConnected = 121,
	};

	struct OutputData {
		int16_t first_return_median;
		int16_t first_return_strength;
		int16_t last_return_raw;
		int16_t last_return_median;
		int16_t last_return_strength;
		int16_t background_noise;
	};

	enum class Type {
		GRF250 =0,
		GRF500,
	};
	enum class State {
		Configuring,
		Running
	};

	int probe() override;

	void start();
	void set_update_rate();
	void set_dist_output();

	template<typename RegisterType>
	int readRegister(RegisterType reg, uint8_t *data, int len);

	int configure();
	int enableI2CBinaryProtocol(const char *product_name1, const char *product_name2);
	int collect();


	PX4Rangefinder _px4_rangefinder;

	int _conversion_interval{-1};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};

	Type _type{Type::GRF250};
	State _state{State::Configuring};
	int _consecutive_errors{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_GRFXXX>) _param_sens_en_grfxxx
	)
};

LightwareGRF::LightwareGRF(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LIGHTWARE_LASER);
}

LightwareGRF::~LightwareGRF()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int LightwareGRF::init()
{
	int ret = PX4_ERROR;
	updateParams();
	const int32_t hw_model = _param_sens_en_grfxxx.get();
	_conversion_interval = 200000;

	switch (hw_model) {
	case 0:
		PX4_WARN("disabled.");
		return ret;

	case 1:  /* SF10/a (25m 32Hz) */
		_px4_rangefinder.set_min_distance(0.1f);
		_px4_rangefinder.set_max_distance(250.0f);
		break;

	case 2:  /* SF10/b (50m 32Hz) */
		_px4_rangefinder.set_min_distance(0.1f);
		_px4_rangefinder.set_max_distance(500.0f);
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

template<typename RegisterType>
int LightwareGRF::readRegister(RegisterType reg, uint8_t *data, int len)
{
	const uint8_t cmd = (uint8_t)reg;
	return transfer(&cmd, 1, data, len);
}

int LightwareGRF::probe()
{
	return enableI2CBinaryProtocol("GRF250", "GRF500");
	return -1;
}

int LightwareGRF::enableI2CBinaryProtocol(const char *product_name1, const char *product_name2)
{
	for (int i = 0; i < 3; ++i) {
		// try to enable I2C binary protocol (this command is undocumented)
		const uint8_t cmd[] = {(uint8_t)Register::Protocol, 0xaa, 0xaa};
		int ret = transfer(cmd, sizeof(cmd), nullptr, 0);

		// read the product name
		uint8_t product_name[16];
		ret = readRegister(Register::ProductName, product_name, sizeof(product_name));
		product_name[sizeof(product_name) - 1] = '\0';
		PX4_INFO("product: %s", product_name);

		if (ret == 0 && (strncmp((const char *)product_name, product_name1, sizeof(product_name)) == 0 ||
				 strncmp((const char *)product_name, product_name2, sizeof(product_name)) == 0)) {
			return 0;
		}
	}

	return -1;
}

int LightwareGRF::configure()
{
	// Setting the update Rate
	set_update_rate();
	set_dist_output();
	return 0;
}


void LightwareGRF::set_update_rate()
{
	const uint8_t cmd[] = {(uint8_t) Register::UpdateRate,(uint8_t)(0), (uint8_t)(0), (uint8_t)(0), (uint8_t)(50)};
	int ret = transfer(cmd, sizeof(cmd), nullptr, 0);
}

void LightwareGRF::set_dist_output()
{
	// read the product name
	uint8_t cmd[] = {(uint8_t)RegisterGRF::DistanceOutput, (uint8_t)0, (uint8_t)0, (uint8_t)1};
	int ret = transfer(cmd, sizeof(cmd), 0, 0);
}



int LightwareGRF::collect()
{

	/* read from the sensor */
	perf_begin(_sample_perf);
	uint8_t val[4] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (readRegister((uint8_t)Register::DistanceData, &val[0], 4) < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		PX4_INFO("Register Read Error");
		return PX4_ERROR;
	}

	perf_end(_sample_perf);

	uint16_t distance_cm = val[2] << 16 | val[1] << 8 | val[0];
	float distance_m = float(distance_cm) * 1e-1f;

	_px4_rangefinder.update(timestamp_sample, distance_m);


	return PX4_OK;
}

void LightwareGRF::start()
{
	/* schedule a cycle to start things */
	ScheduleDelayed(_conversion_interval);
}

void LightwareGRF::RunImpl()
{

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

		if (PX4_OK != collect()) {
			PX4_DEBUG("collection error");

			if (++_consecutive_errors > 3) {
				_state = State::Configuring;
				_consecutive_errors = 0;
			}
		}


		ScheduleDelayed(_conversion_interval);
		break;
	}
}

void LightwareGRF::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void LightwareGRF::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for Lightware GRFxxx series LIDAR rangefinders: GRF250, GRF500.

Setup/usage information:
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lightware_grf_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x66);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int lightware_grf_i2c_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = LightwareGRF;
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
