/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @author Muhammad Rizky Millennianno <muhammad.rizky912@ui.ac.id>
 * @author Vishwakarma Aerial Dexterity Group Universitas Indonesia <rotary.auav@gmail.com>
 *
 * Driver for the Benewake TF-Luna lidar range finder.
 * Default I2C address 0x66 is used.
 */

#include "tfluna_i2c.hpp"

TFLuna::TFLuna(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	// TFLuna typical range 0-8 meters with 2 degrees FOV
	_px4_rangefinder.set_min_distance(0.2f);
	_px4_rangefinder.set_max_distance(8.f);
	_px4_rangefinder.set_fov(math::radians(2.f));

	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_TFLUNA);
}

TFLuna::~TFLuna()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TFLuna::init()
{
	// synchronize TFLuna FPS with default
	uint8_t fps_reg[2];
	bool fps_mismatch = false;
	int ret = read_register(TFLUNA_FPS_LOW, fps_reg[0]);
	ret = read_register(TFLUNA_FPS_HIGH, fps_reg[1]);
	if (ret != PX4_OK) {
		PX4_ERR("Failed to get sensor FPS");
	}
	else {
		if ((fps_reg[0] != TFLUNA_BASE_SAMPLE_HZ[0]) && (fps_reg[1] != TFLUNA_BASE_SAMPLE_HZ[1])) {
			fps_mismatch = true;
			ret = write_register(TFLUNA_FPS_LOW, TFLUNA_BASE_SAMPLE_HZ[0]);
			ret = write_register(TFLUNA_FPS_HIGH, TFLUNA_BASE_SAMPLE_HZ[1]);
		}
	}

	// synchronize TFLuna trigger mode with default
	uint8_t mode_reg;
	ret = read_register(TFLUNA_TRIG_MODE, mode_reg);
	if (ret != PX4_OK) {
		PX4_ERR("Failed to get sensor trigger mode");
	}
	else {
		if (mode_reg != TFLUNA_CONTINOUS) {
			ret = write_register(TFLUNA_TRIG_MODE, TFLUNA_CONTINOUS);
		}
	}

	// save settings to TFLuna sensor
	if ((mode_reg != TFLUNA_CONTINOUS) || fps_mismatch) {
		ret = write_register(TFLUNA_SAVE_SETTINGS, 0x01);
		if(ret == PX4_OK) {
			PX4_INFO("Synchronizing sensor setting success");
			mode_reg = TFLUNA_CONTINOUS;
			fps_reg[0] = TFLUNA_BASE_SAMPLE_HZ[0];
			fps_reg[1] = TFLUNA_BASE_SAMPLE_HZ[1];
		}
		else PX4_ERR("Failed to sync sensor setting");
	}

	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	ScheduleDelayed(TFLUNA_BASE_SAMPLE_MS);
	return PX4_OK;
}

int TFLuna::probe()
{
	// enable TFLuna I2C if disabled
	uint8_t state_tfluna;
	int ret = read_register(TFLUNA_ENABLEDISABLE, state_tfluna);
	if (state_tfluna != 0x01) ret = write_register(TFLUNA_ENABLEDISABLE, 0x01);
	
	uint8_t product_code[14];

	// read TFLuna product code
	for (uint8_t i = 0; i < 14; ++i) {
		ret = read_register(0x10 + i, product_code[i]);
  		if(ret != PX4_OK) break;
  	}

	PX4_DEBUG("product: %s", product_code);

	if (ret == PX4_OK) {
		return PX4_OK;
	}

	// Device not found on any address.
	return -EIO;
}

int TFLuna::collect()
{
	// Read from the sensor.
	uint8_t read_reg[6]; // 0-1 dist, 2-3 signal, 4-5 temperature
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	/**
	* read TFLuna data from distance to temperature (register address
	*	is consecutively increasing)
	**/
	for (uint8_t reg = TFLUNA_DIST_LOW; reg <= TFLUNA_TEMP_HIGH; reg++) {
		if(read_register(reg, read_reg[reg]) != PX4_OK) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return PX4_ERROR;
		}
	}

	perf_end(_sample_perf);

	uint16_t signal = (read_reg[2] << 8) | read_reg[3];
	int temp = (read_reg[4] << 8) | read_reg[5];
	float temp_celsius = float(temp/8.f - 256.f);
	uint16_t distance_cm = (read_reg[0] << 8) | read_reg[1];
	float distance_m = float(distance_cm / 100.f);

	PX4_DEBUG("signal: %i", signal);
	PX4_DEBUG("distance: %i", distance_cm);
	PX4_DEBUG("temp: %f", temp);

	if ((signal < UNDEREXPOSE_SENSOR_THRESHOLD) || 
		(signal == OVEREXPOSE_SENSOR_THRESHOLD)) {
		PX4_ERR("sensor light overexposed or underexposed");
		return PX4_ERROR;
	}

	if (temp_celsius > OVERHEAT_SENSOR_THRESHOLD) {
		PX4_ERR("sensor overheated");
		write_register(TFLUNA_ENABLEDISABLE, 0x00); // disable the sensor
		return PX4_ERROR;
	}

	_px4_rangefinder.update(timestamp_sample, distance_m);
	return PX4_OK;
}

int TFLuna::reset_sensor()
{
	int ret = write_register(TFLUNA_SOFT_RESET, 0x02);
	if (ret == PX4_OK) {
		px4_usleep(50_ms); // wait for reset to complete when register success
	}
	return ret;
}

void TFLuna::RunImpl()
{
	if (PX4_OK != collect()) {
		PX4_DEBUG("collection error");

		if (++_consecutive_errors > 3) {
			reset_sensor();
			_consecutive_errors = 0;
		}
	}

	ScheduleDelayed(TFLUNA_BASE_SAMPLE_MS);
}

/* Registers API */
int TFLuna::write_register(const uint8_t& reg, const uint8_t& val)
{
	const uint8_t cmd[2] {reg, val};
	return transfer(&cmd[0], 2, nullptr, 0);
}

int TFLuna::read_register(const uint8_t& reg, uint8_t& val)
{
	// Write register address to the sensor.
	if (transfer(&reg, 1, nullptr, 0) != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	// Read from the sensor.
	if (transfer(nullptr, 0, &val, 1) != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	return PX4_OK;
}

/* Interface to PX4 methods */
void TFLuna::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void TFLuna::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description

		I2C bus driver for Benewake TFLuna laser rangefinder.

		The sensor/driver must be enabled using the parameter SENS_EN_TFLUNA.

		Setup/usage information: https://docs.px4.io/main/en/sensor/tfluna.html
		)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("tfluna_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(TFLUNA_BASEADDR);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tfluna_i2c_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = TFLuna;
	BusCLIArguments cli{true, false};
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = TFLUNA_BASEADDR;

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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TFLUNA);

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
