/****************************************************************************
 *
 *
 * Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
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
 * @file ASP5033.cpp
 *
 *@author Denislav Petrov <denislavamitoba@gmail.com>
 */

#include "ASP5033.hpp"
#include <parameters/param.h>

ASP5033::ASP5033(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

ASP5033::~ASP5033()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);
}

int ASP5033::probe()
{
	int ret = sensor_id_check();
	return ret;
}

int ASP5033::sensor_id_check()
{
	uint8_t id[1];
	uint8_t cmd_1 = REG_ID_SET_ASP5033;
	uint8_t cmd_2 = REG_WHOAMI_RECHECK_ID_ASP5033;
	uint8_t cmd_3 = REG_ID_ASP5033;
	uint8_t cmd_1_2[2];
	cmd_1_2[0] = static_cast<uint8_t>(cmd_1);
	cmd_1_2[1] = static_cast<uint8_t>(cmd_2);


	if ((transfer(&cmd_1, 1, &id[0], sizeof(id)) != PX4_OK) || (*id != REG_WHOAMI_DEFAULT_ID_ASP5033)) { return 0; }

	if (transfer(&cmd_1_2[0], 2, nullptr, 0) != PX4_OK) { return 0; }

	if ((transfer(&cmd_3, 1, &id[0], sizeof(id)) != PX4_OK) || (*id != REG_WHOAMI_RECHECK_ID_ASP5033)) { return 0; }

	return 1;
}

int ASP5033::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	ScheduleNow();
	return ret;
}


/**
 * @brief calculation of the differential pressure in this way:
 * it collect all measured pressure and store it into press_sum,
 * count the value of collected times-press_count, then divide both
 * and get the actual value of differential pressure - _pressure
 *
 * @return true if pressure is valid and no errors, false if not
 */
bool ASP5033::get_differential_pressure()
{
	if (hrt_elapsed_time(&last_sample_time) > 200_ms) {
		return false;
	}

	if (press_count == 0) {
		return false;
	}

	//calculation differential pressure
	_pressure = press_sum / press_count;

	press_sum = 0.;
	press_count = 0;
	return true;
}


void ASP5033::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);
}

void ASP5033::RunImpl()
{
	int ret = PX4_ERROR;

	// collection phase
	if (_collect_phase) {
		// perform collection
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
			/* restart the measurement state machine */
			_collect_phase = false;
			_sensor_ok = false;
			ScheduleNow();
			return;
		}

		// next phase is measurement
		_collect_phase = false;

		// is there a collect->measure gap?
		if (_measure_interval > CONVERSION_INTERVAL) {

			// schedule a fresh cycle call when we are ready to measure again
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

	// next phase is collection
	_collect_phase = true;

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}


int ASP5033::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd_1 = CMD_MEASURE_ASP5033;
	uint8_t cmd_2 = REG_CMD_ASP5033;;

	//write to driver to start
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(cmd_2);
	cmd[1] = static_cast<uint8_t>(cmd_1);
	int ret = transfer(&cmd[0], 2, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int ASP5033::collect()
{
	perf_begin(_sample_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// Read pressure and temperature as one block
	uint8_t val[5] {0, 0, 0, 0, 0};
	uint8_t cmd = REG_PRESS_DATA_ASP5033;
	transfer(&cmd, 1, &val[0], sizeof(val));

	//Pressure is a signed 24-bit value
	int32_t press = (val[0] << 24) | (val[1] << 16) | (val[2] << 8);
	// convert back to 24 bit
	press >>= 8;

	// k is a shift based on the pressure range of the device. See
	// table in the datasheet
	constexpr uint8_t k = 7;
	constexpr float press_scale = 1.0f / (1U << k); //= 1.0f / (1U << k);
	press_sum += press * press_scale;
	press_count++;

	// temperature is 16 bit signed in units of 1/256 C
	const int16_t temp = (val[3] << 8) | val[4];
	constexpr float temp_scale = 1.0f / 256;
	_temperature = temp * temp_scale;
	last_sample_time = hrt_absolute_time();
	bool status = get_differential_pressure();

	if (status == true && (int)_temperature != 0) {
		// publish values
		differential_pressure_s differential_pressure{};
		differential_pressure.timestamp_sample = timestamp_sample;
		differential_pressure.device_id = get_device_id();
		differential_pressure.differential_pressure_pa = _pressure;
		int32_t differential_press_rev = 0;
		param_get(param_find("SENS_DPRES_REV"), &differential_press_rev);

		//If differential pressure reverse param set, swap positive and negative
		if (differential_press_rev == 1) {
			differential_pressure.differential_pressure_pa = -1.0f * _pressure;
		}


		differential_pressure.temperature = _temperature ;
		differential_pressure.error_count = perf_event_count(_comms_errors);
		differential_pressure.timestamp = timestamp_sample;
		_differential_pressure_pub.publish(differential_pressure);

	}

	perf_end(_sample_perf);

	return PX4_OK;
}
