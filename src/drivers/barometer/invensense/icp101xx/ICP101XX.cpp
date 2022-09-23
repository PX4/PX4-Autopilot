/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include "ICP101XX.hpp"

using namespace time_literals;

ICP101XX::ICP101XX(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

ICP101XX::~ICP101XX()
{
	perf_free(_reset_perf);
	perf_free(_sample_perf);
	perf_free(_bad_transfer_perf);
}

int
ICP101XX::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool
ICP101XX::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void
ICP101XX::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_reset_perf);
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfer_perf);
}

int
ICP101XX::probe()
{
	uint16_t ID = 0;
	read_response(Cmd::READ_ID, (uint8_t *)&ID, 2);
	uint8_t PROD_ID = (ID >> 8) & 0x3f;

	if (PROD_ID != Product_ID) {
		DEVICE_DEBUG("unexpected PROD_ID 0x%02x", PROD_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void
ICP101XX::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// Software Reset
		send_command(Cmd::SOFT_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(100_ms); // Power On Reset: max 100ms
		break;

	case STATE::WAIT_FOR_RESET: {
			// check product id
			uint16_t ID = 0;
			read_response(Cmd::READ_ID, (uint8_t *)&ID, 2);
			uint8_t PROD_ID = (ID >> 8) & 0x3f; // Product ID Bits 5:0

			if (PROD_ID == Product_ID) {
				// if reset succeeded then read otp
				_state = STATE::READ_OTP;
				ScheduleDelayed(10_ms); // Time to coefficients are available.

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("Reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(100_ms);

				} else {
					PX4_DEBUG("Reset not complete, check again in 10 ms");
					ScheduleDelayed(10_ms);
				}
			}
		}
		break;

	case STATE::READ_OTP: {
			// read otp
			uint8_t addr_otp_cmd[3] = {0x00, 0x66, 0x9c};
			uint8_t otp_buf[3];
			uint8_t crc;
			bool success = true;

			send_command(Cmd::SET_ADDR, addr_otp_cmd, 3);

			for (uint8_t i = 0; i < 4; i++) {
				read_response(Cmd::READ_OTP, otp_buf, 3);

				crc = 0xFF;

				for (int j = 0; j < 2; j++) {
					crc = (uint8_t)cal_crc(crc, otp_buf[j]);
				}

				if (crc != otp_buf[2]) {
					success = false;
					break;
				}

				_scal[i] = (otp_buf[0] << 8) | otp_buf[1];
			}

			if (success) {
				_state = STATE::MEASURE;

			} else {
				_state = STATE::RESET;
			}

			ScheduleDelayed(10_ms);
		}
		break;

	case STATE::MEASURE:
		if (Measure()) {
			// if configure succeeded then start measurement cycle
			_state = STATE::READ;
			perf_begin(_sample_perf);
			ScheduleDelayed(_measure_interval);

		} else {
			// MEASURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Measure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Measure failed, retrying");
			}

			ScheduleDelayed(_measure_interval);
		}

		break;

	case STATE::READ: {
			uint8_t comp_data[9] {};
			bool success = false;

			if (read_measure_results(comp_data, 9) == PX4_OK) {
				perf_end(_sample_perf);

				uint16_t _raw_t = (comp_data[0] << 8) | comp_data[1];
				uint32_t L_res_buf3 = comp_data[3];	// expand result bytes to 32bit to fix issues on 8-bit MCUs
				uint32_t L_res_buf4 = comp_data[4];
				uint32_t L_res_buf6 = comp_data[6];
				uint32_t _raw_p = (L_res_buf3 << 16) | (L_res_buf4 << 8) | L_res_buf6;

				// constants for presure calculation
				static constexpr float _pcal[3] = { 45000.0, 80000.0, 105000.0 };
				static constexpr float _lut_lower = 3.5 * 0x100000;	// 1<<20
				static constexpr float _lut_upper = 11.5 * 0x100000;	// 1<<20
				static constexpr float _quadr_factor = 1 / 16777216.0;
				static constexpr float _offst_factor = 2048.0;

				// calculate temperature
				float _temperature_C = -45.f + 175.f / 65536.f * _raw_t;

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
				float _pressure_Pa = a + b / (c + _raw_p);

				float temperature = _temperature_C;
				float pressure = _pressure_Pa;

				// publish
				sensor_baro_s sensor_baro{};
				sensor_baro.timestamp_sample = now;
				sensor_baro.device_id = get_device_id();
				sensor_baro.pressure = pressure;
				sensor_baro.temperature = temperature;
				sensor_baro.error_count = perf_event_count(_bad_transfer_perf);
				sensor_baro.timestamp = hrt_absolute_time();
				_sensor_baro_pub.publish(sensor_baro);

				success = true;

				if (_failure_count > 0) {
					_failure_count--;
				}

				_state = STATE::MEASURE;

			} else {
				perf_count(_bad_transfer_perf);
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			ScheduleDelayed(1000_ms / 8 - _measure_interval); // 8Hz
		}

		break;
	}
}

bool
ICP101XX::Measure()
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
	Cmd cmd;

	switch (_mode) {
	case MODE::FAST:
		cmd = Cmd::MEAS_LP;
		_measure_interval = 2_ms;
		break;

	case MODE::ACCURATE:
		cmd = Cmd::MEAS_LN;
		_measure_interval = 24_ms;
		break;

	case MODE::VERY_ACCURATE:
		cmd = Cmd::MEAS_ULN;
		_measure_interval = 95_ms;
		break;

	case MODE::NORMAL:
	default:
		cmd = Cmd::MEAS_N;
		_measure_interval = 7_ms;
		break;
	}

	if (send_command(cmd) != PX4_OK) {
		return false;
	}

	return true;
}

int8_t
ICP101XX::cal_crc(uint8_t seed, uint8_t data)
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
ICP101XX::read_measure_results(uint8_t *buf, uint8_t len)
{
	return transfer(nullptr, 0, buf, len);
}

int
ICP101XX::read_response(Cmd cmd, uint8_t *buf, uint8_t len)
{
	uint8_t buff[2];
	buff[0] = ((uint16_t)cmd >> 8) & 0xff;
	buff[1] = (uint16_t)cmd & 0xff;
	return transfer(&buff[0], 2, buf, len);
}

int
ICP101XX::send_command(Cmd cmd)
{
	uint8_t buf[2];
	buf[0] = ((uint16_t)cmd >> 8) & 0xff;
	buf[1] = (uint16_t)cmd & 0xff;
	return transfer(buf, sizeof(buf), nullptr, 0);
}

int
ICP101XX::send_command(Cmd cmd, uint8_t *data, uint8_t len)
{
	uint8_t buf[5];
	buf[0] = ((uint16_t)cmd >> 8) & 0xff;
	buf[1] = (uint16_t)cmd & 0xff;
	memcpy(&buf[2], data, len);
	return transfer(&buf[0], len + 2, nullptr, 0);
}
