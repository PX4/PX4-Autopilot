/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file icp10100.cpp
 *
 * Driver for the ICP10100 barometric pressure sensor connected via I2C
 *
 * Refer to: https://github.com/astuder/icp-101xx.git
 */

#include "icp10100.h"

// constants for presure calculation
const float _pcal[3] = { 45000.0, 80000.0, 105000.0 };
const float _lut_lower = 3.5 * 0x100000;	// 1<<20
const float _lut_upper = 11.5 * 0x100000;	// 1<<20
const float _quadr_factor = 1 / 16777216.0;
const float _offst_factor = 2048.0;
static int8_t cal_crc(uint8_t seed, uint8_t data);


ICP10100::ICP10100(const I2CSPIDriverConfig &config, IICP10100 *interface) :
	I2CSPIDriver(config),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

ICP10100::~ICP10100()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
ICP10100::init()
{
	if (_interface->send_command(ICP_CMD_SOFT_RESET) != OK) {
		PX4_WARN("failed to reset baro during init");
		return -EIO;
	}

	// software reset time to be greater than 170us
	usleep(250);

	// verify that the sensor is responding
	if (!is_connected()) {
		PX4_WARN("id of your baro is not: 0x%02x", ICP10xxx_CHIP_ID);
		return -EIO;
	}

	// read sensor calibration data
	if (!get_calibration()) {
		PX4_WARN("failed to get baro cal init");
		return -EIO;
	}

	if (!set_sensor_settings()) {
		PX4_WARN("failed to set sensor settings");
		return -EIO;
	}

	start();

	return OK;
}

void
ICP10100::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_perf);
	perf_print_counter(_comms_errors);
}

void
ICP10100::start()
{
	_collect_phase = false;

	// wait a bit longer for the first measurement, as otherwise the first readout might fail
	ScheduleOnInterval(_measure_interval, _measure_interval * 3);
}

void
ICP10100::RunImpl()
{
	if (_collect_phase) {
		collect();
	}

	measure();
}

int
ICP10100::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measurement */
	if (!get_measurement_time(VERY_ACCURATE)) {
		PX4_DEBUG("failed to set operating mode");
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

static int8_t cal_crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x31;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((seed & 0x80) ^ (data & 0x80)) {
			var2 = 1;

		} else {
			var2 = 0;
		}

		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}

int
ICP10100::collect()
{
	_collect_phase = false;

	/* enable pressure and temperature */
	uint8_t res_buf[9] {};

	perf_begin(_sample_perf);

	// /* this should be fairly close to the end of the conversion, so the best approximation of the time */
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (!get_sensor_data(res_buf)) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	_px4_baro.set_error_count(perf_event_count(_comms_errors));

	float temperature = _temperature_C;
	float pressure = _pressure_Pa; // to Pascal
	pressure = pressure / 100.0f; // to mbar

	_px4_baro.set_temperature(temperature);
	_px4_baro.update(timestamp_sample, pressure);

	perf_end(_sample_perf);

	return OK;
}

bool
ICP10100::get_sensor_data(uint8_t *comp_data)
{
	if (_interface->read_measurement_results(comp_data, 9) == OK) {
		_raw_t = (comp_data[0] << 8) | comp_data[1];
		uint32_t L_res_buf3 = comp_data[3];	// expand result bytes to 32bit to fix issues on 8-bit MCUs
		uint32_t L_res_buf4 = comp_data[4];
		uint32_t L_res_buf6 = comp_data[6];
		_raw_p = (L_res_buf3 << 16) | (L_res_buf4 << 8) | L_res_buf6;
		_calculate();
		return true;
	}

	return false;
}

bool
ICP10100::set_sensor_settings()
{
	_measure_interval = get_measurement_time(VERY_ACCURATE);

	if (_measure_interval == 0) {
		PX4_WARN("unsupported oversampling selected");
		return false;
	}

	return true;
}


bool
ICP10100::is_connected()
{
	uint8_t id_buf[2];

	if (_interface->read_response(ICP_CMD_READ_ID, id_buf, 2) == OK) {
		uint16_t id = (id_buf[0] << 8) | id_buf[1];

		if ((id & 0x03f) == 0x08) {
			return true;
		}
	}

	return false;
}

uint32_t
ICP10100::get_measurement_time(ICP10100::mmode mode)
{

	/*
	  From ds-000186-icp-101xx-v1.0.pdf, page 6, table 1

	  Sensor                  Measurement       Max Time
	  Mode                    Time (Forced)
	  Low Power (LP)             1.6 ms          1.8 ms
	  Normal (N)                 5.6 ms          6.3 ms
	  Low Noise (LN)             20.8 ms         23.8 ms
	  Ultra Low Noise(ULN)       83.2 ms         94.5 ms
	*/
	uint16_t cmd;
	uint32_t meas_time_us = 0; // unsupported value by default

	switch (mode) {
	case ICP10100::FAST:
		cmd = ICP_CMD_MEAS_LP;
		meas_time_us = 3000;
		break;

	case ICP10100::ACCURATE:
		cmd = ICP_CMD_MEAS_LN;
		meas_time_us = 24000;
		break;

	case ICP10100::VERY_ACCURATE:
		cmd = ICP_CMD_MEAS_ULN;
		meas_time_us = 95000;
		break;

	case ICP10100::NORMAL:
	default:
		cmd = ICP_CMD_MEAS_N;
		meas_time_us = 7000;
		break;
	}

	if (_interface->send_command(cmd) != OK) {
		meas_time_us = 0;
	}

	return meas_time_us;
}

void
ICP10100::_calculate(void)
{
	// calculate temperature
	_temperature_C = -45.f + 175.f / 65536.f * _raw_t;

	// calculate pressure
	float t = (float)(_raw_t - 32768);
	float s1 = _lut_lower + (float)(_scal[0] * t * t) * _quadr_factor;
	float s2 = _offst_factor * _scal[3] + (float)(_scal[1] * t * t) * _quadr_factor;
	float s3 = _lut_upper + (float)(_scal[2] * t * t) * _quadr_factor;
	float c = (s1 * s2 * (_pcal[0] - _pcal[1]) +
		   s2 * s3 * (_pcal[1] - _pcal[2]) +
		   s3 * s1 * (_pcal[2] - _pcal[0])) /
		  (s3 * (_pcal[0] - _pcal[1]) +
		   s1 * (_pcal[1] - _pcal[2]) +
		   s2 * (_pcal[2] - _pcal[0]));
	float a = (_pcal[0] * s1 - _pcal[1] * s2 - (_pcal[1] - _pcal[0]) * c) / (s1 - s2);
	float b = (_pcal[0] - a) * (s1 + c);
	_pressure_Pa = a + b / (c + _raw_p);
}

bool
ICP10100::get_calibration()
{
	uint8_t addr_otp_cmd[3] = {0x00, 0x66, 0x9c};
	uint8_t otp_buf[3];

	if (_interface->send_command(ICP_CMD_SET_ADDR, addr_otp_cmd, 3) != OK) {
		return false;
	}

	for (uint8_t i = 0; i < 4; i++) {
		if (_interface->read_response(ICP_CMD_READ_OTP, otp_buf, 3) != OK) {
			return false;
		}

		uint8_t crc = 0xFF;

		for (int j = 0; j < 2; j++) {
			crc = (uint8_t)cal_crc(crc, otp_buf[j]);
		}

		if (crc != otp_buf[2]) {
			return false;
		}

		_scal[i] = (otp_buf[0] << 8) | otp_buf[1];
	}

	return true;
}
