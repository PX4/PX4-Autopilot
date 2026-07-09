/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ADIS16607.hpp"
#include <endian.h>

using namespace time_literals;

ADIS16607::ADIS16607(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ADIS16607::~ADIS16607()
{
	perf_free(_reset_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
}

int ADIS16607::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ADIS16607::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ADIS16607::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void ADIS16607::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
}

int ADIS16607::probe()
{
	// Power-On Start-Up Time 50 ms
	if (hrt_absolute_time() < 50_ms) {
		PX4_WARN("required Power-On Start-Up Time 50 ms");
	}

	// lock the device to half duplex SPI mode
	RegisterWrite(Register::SPI_HALFDUPLEX_KEY, SPI_HALFDUPLEX_KEY_VALUE);

	const uint16_t device_id = RegisterRead(Register::DEV_ID);

	if (device_id != DEVICE_IDENTIFICATION) {
		PX4_ERR("unexpected DEV_ID 0x%02x", device_id);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ADIS16607::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		RegisterWrite(Register::SOFT_RESET, SOFT_RESET_BIT::RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(50_ms); // 50 ms Reset Recovery Time
		break;

	case STATE::WAIT_FOR_RESET:
		// lock the device to half duplex SPI mode
		RegisterWrite(Register::SPI_HALFDUPLEX_KEY, SPI_HALFDUPLEX_KEY_VALUE);
		// These bits are cleared when read
		RegisterRead(Register::DIAG_STAT);

		if (_self_test_passed) {
			if ((RegisterRead(Register::DEV_ID) == DEVICE_IDENTIFICATION)) {
				// if reset succeeded then configure
				_state = STATE::CONFIGURE;
				ScheduleNow();

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_WARN("Reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(100_ms);

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}

		} else {
			RegisterWrite(Register::MSC_CTRL, MSC_CTRL_BIT::Self_test_1);
			_state = STATE::SELF_TEST_CHECK;
			ScheduleDelayed(50_ms); // Self Test Time
		}

		break;

	case STATE::SELF_TEST_CHECK: {
			// read DIAG_STAT to check result
			const uint16_t diag_stat = RegisterRead(Register::DIAG_STAT);

			if (diag_stat != 0) {
				PX4_ERR("self test failed, resetting. DIAG_STAT: %#X", diag_stat);
				_state = STATE::RESET;
				ScheduleDelayed(3_s);

			} else {
				PX4_DEBUG("self test passed");
				_self_test_passed = true;
				_state = STATE::RESET;
				ScheduleNow();
			}
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::FIFO_READ;
			FIFOReset();

			// The DR interrupt configuration does not support FIFO triggering
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_WARN("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_WARN("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;
			uint8_t samples = 0;

			//  Read status
			const uint16_t diag_stat = RegisterRead(Register::DIAG_STAT);

			if ((diag_stat & ~DIAG_STAT_BIT::FIFO_Threshold_Met) != 0) {
				// Reset the sensor if any error other than FIFO overflow occurs.
				Reset();
				return;
			}

			// check current FIFO count
			const uint16_t fifo_count_bytes = RegisterRead(Register::FIFO_WORD_CNT) * 2;

			if (fifo_count_bytes >= FIFO::SIZE) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (fifo_count_bytes == 0) {
				perf_count(_fifo_empty_perf);

			} else {
				// FIFO count (size in bytes)
				samples = (fifo_count_bytes / sizeof(FIFO::DATA));

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);
					samples = 0;
				}
			}

			bool success = false;

			if (samples >= 1) {
				if (FIFORead(timestamp_sample, samples)) {
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}
		}

		break;
	}
}

void ADIS16607::ConfigureSampleRate(int sample_rate)
{
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES));

	_fifo_empty_interval_us = _fifo_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_samples);
}

void ADIS16607::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of word
	const uint16_t fifo_watermark_threshold = samples * (sizeof(FIFO::DATA) / sizeof(uint16_t));

	for (auto &r : _register_cfg) {
		if (r.reg == Register::USER_FIFO_CFG) {
			r.set_bits = r.set_bits | fifo_watermark_threshold;
		}
	}
}

bool ADIS16607::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// The updateFIFO() function only supports 16-bit resolution configuration
	// accel: ±40 g, 781.25 LSB/g (16-bit format)
	_px4_accel.set_range(40.f * CONSTANTS_ONE_G);
	_px4_accel.set_scale(CONSTANTS_ONE_G / 781.25f); // scaling 781.25 LSB/g -> m/s^2 per LSB

	// gyro: ±2000 °/sec, 15.625 LSB/°/sec (16-bit format)
	_px4_gyro.set_range(math::radians(2000.f));
	_px4_gyro.set_scale(math::radians(1.f / 15.625f)); // scaling 15.625 LSB/°/sec -> rad/s per LSB

	return success;
}

bool ADIS16607::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint16_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint16_t ADIS16607::RegisterRead(Register reg)
{
	uint8_t cmd[4];
	cmd[0] = (static_cast<uint8_t>(reg) | DIR_READ);

	transfer(&cmd[0], &cmd[0], 4);

	return (uint16_t)((cmd[2] << 8) | cmd[3]);
}

void ADIS16607::RegisterWrite(Register reg, uint16_t value)
{
	uint8_t cmd[3];
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = (value >> 8) & 0xFF;
	cmd[2] = value & 0xFF;

	transfer(&cmd[0], nullptr, 3);
}

void ADIS16607::RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits)
{
	const uint16_t orig_val = RegisterRead(reg);

	uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

void ADIS16607::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// Clear FIFO data
	for (auto &r : _register_cfg) {
		if (r.reg == Register::USER_FIFO_CFG) {
			RegisterWrite(r.reg, r.set_bits | USER_FIFO_CFG_BIT::CLEAR_FIFOB);
		}
	}
}

bool ADIS16607::FIFORead(const hrt_abstime &timestamp_sample, const uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 2, (static_cast<uint8_t>(FIFO_MAX_SAMPLES) * sizeof(FIFO::DATA) + 2));

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	if (ProcessTemperature(buffer.f, samples)) {
		ProcessAccel(timestamp_sample, buffer.f, samples);
		ProcessGyro(timestamp_sample, buffer.f, samples);
		return true;
	}

	return false;
}

void ADIS16607::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = FIFO_SAMPLE_DT;

	for (uint8_t i = 0; i < samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = be16toh(fifo[i].ACCEL_DATA_X);
		accel.y[i] = -1 * static_cast<int16_t>(be16toh(fifo[i].ACCEL_DATA_Y));
		accel.z[i] = -1 * static_cast<int16_t>(be16toh(fifo[i].ACCEL_DATA_Z));

	}

	_px4_accel.set_error_count(perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}
}

void ADIS16607::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_SAMPLE_DT;

	for (uint8_t i = 0; i < samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = be16toh(fifo[i].GYRO_DATA_X);
		gyro.y[i] = -1 * static_cast<int16_t>(be16toh(fifo[i].GYRO_DATA_Y));
		gyro.z[i] = -1 * static_cast<int16_t>(be16toh(fifo[i].GYRO_DATA_Z));
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		_px4_gyro.updateFIFO(gyro);
	}
}

bool ADIS16607::ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples)
{
	int16_t temperature[FIFO_MAX_SAMPLES];
	float temperature_sum{0};

	int valid_samples = 0;

	for (int i = 0; i < samples; i++) {
		const int16_t t = (int16_t)be16toh(fifo[i].TEMP);

		// sample invalid if -32768
		if (t != -32768) {
			temperature_sum += t;
			temperature[valid_samples] = t;
			valid_samples++;
		}
	}

	if (valid_samples > 0) {
		const float temperature_avg = temperature_sum / valid_samples;

		for (int i = 0; i < valid_samples; i++) {
			// temperature changing wildly is an indication of a transfer error
			if (fabsf(temperature[i] - temperature_avg) > TEMPERATURE_WILDLY_THRESHOLD) {
				perf_count(_bad_transfer_perf);
				return false;
			}
		}

		// use average temperature reading
		const float temp_c = (temperature_avg * TEMPERATURE_SCALE_FACTOR) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(temp_c)) {
			_px4_accel.set_temperature(temp_c);
			_px4_gyro.set_temperature(temp_c);
			return true;

		} else {
			perf_count(_bad_transfer_perf);
		}
	}

	return false;
}
