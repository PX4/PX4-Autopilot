/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#include "LSM303D.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

LSM303D::LSM303D(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_mag(get_device_id(), config.rotation)
{
	ConfigureSampleRate(_px4_accel.get_max_rate_hz());
}

LSM303D::~LSM303D()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
}

int LSM303D::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool LSM303D::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void LSM303D::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
}

int LSM303D::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void LSM303D::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// CTRL0
		RegisterSetBits(Register::CTRL0, CTRL0_BIT::BOOT);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(150_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::WHO_AM_I) == WHOAMI) {
			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(10_ms);

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

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			bool success = false;

			// always check FIFO status and count
			const uint8_t FIFO_SRC = RegisterRead(Register::FIFO_SRC);
			const uint8_t samples = (FIFO_SRC & FIFO_SRC_BIT::FSS); // FSS4-FSS0 FIFO stored data level

			if ((FIFO_SRC & FIFO_SRC_BIT::OVRN) || (samples > FIFO_MAX_SAMPLES)) {
				// not necessarily an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				FIFOReset();

			} else if ((FIFO_SRC & FIFO_SRC_BIT::EMPTY) || (samples == 0)) {
				perf_count(_fifo_empty_perf);

			} else {
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

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}

			} else {
				// periodically update temperature (50 Hz)
				if (now >= _temperature_and_magnetometer_update_timestamp + 20_ms) {
					UpdateTemperatureAndMagnetometer();
					_temperature_and_magnetometer_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void LSM303D::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = ST_LSM303D::LA_ODR; // default to max ODR
	}

	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / ACCEL_RATE), (float)FIFO_MAX_SAMPLES));

	_fifo_samples = math::max(_fifo_samples, (uint32_t)2); // require at least 2 FIFO samples per iteration

	// recompute FIFO empty interval (us) with actual accel sample limit
	_fifo_empty_interval_us = _fifo_samples * (1e6f / ACCEL_RATE);
}

bool LSM303D::Configure()
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

	// Linear acceleration sensitivity ±16g (0.732 mg/LSB)
	_px4_accel.set_scale(CONSTANTS_ONE_G * 0.732f * 1e-3f);
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);

	// Magnetic sensitivity ±12gauss (0.479 mgauss/LSB)
	_px4_mag.set_scale(0.479f * 1e-3f);

	return success;
}

bool LSM303D::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

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

uint8_t LSM303D::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM303D::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM303D::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool LSM303D::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	ProcessAccel(timestamp_sample, buffer.f, samples);

	return true;
}

void LSM303D::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_CTRL_REG: switch to bypass mode to restart data collection
	RegisterClearBits(Register::FIFO_CTRL, FIFO_CTRL_BIT::Bypass_mode);

	// FIFO_CTRL_REG: mode + watermark
	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CTRL) {
			RegisterSetAndClearBits(Register::FIFO_CTRL, r.set_bits, r.clear_bits);
		}
	}
}

void LSM303D::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		int16_t accel_x = combine(fifo[i].OUT_X_H_A, fifo[i].OUT_X_L_A);
		int16_t accel_y = combine(fifo[i].OUT_Y_H_A, fifo[i].OUT_Y_L_A);
		int16_t accel_z = combine(fifo[i].OUT_Z_H_A, fifo[i].OUT_Z_L_A);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel_x;
		accel.y[i] = math::negate(accel_y);
		accel.z[i] = math::negate(accel_z);
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_accel.updateFIFO(accel);
}

bool LSM303D::UpdateTemperatureAndMagnetometer()
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	// read current temperature
	struct TransferBuffer {
		uint8_t cmd;
		uint8_t TEMP_OUT_L; // 0x05
		uint8_t TEMP_OUT_H; // 0x06
		uint8_t STATUS_M;   // 0x07
		uint8_t OUT_X_L_M;  // 0x08
		uint8_t OUT_X_H_M;  // 0x09
		uint8_t OUT_Y_L_M;  // 0x0A
		uint8_t OUT_Y_H_M;  // 0x0B
		uint8_t OUT_Z_L_M;  // 0x0C
		uint8_t OUT_Z_H_M;  // 0x0D
	} buffer{};

	buffer.cmd = static_cast<uint8_t>(Register::TEMP_OUT_L) | DIR_READ | AUTO_INCREMENT;

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	if (buffer.STATUS_M & STATUS_M_BIT::ZYXMDA) {
		// 8 LSB/°C, 25 °C offset
		// Temperature data is stored inside TEMP_OUT_L (05h), TEMP_OUT_H (06h) as 2’s
		// complement data in 12-bit format, right justified.
		uint16_t TEMP_OUT = combine(buffer.TEMP_OUT_H, buffer.TEMP_OUT_L);

		// shift to align sign bit, cast to int16_t, then shift back
		int16_t temperature_raw = static_cast<int16_t>(TEMP_OUT << 4) >> 4;

		const float temperature_c = (temperature_raw / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(temperature_c)
		    && (temperature_c >= TEMPERATURE_SENSOR_MIN)
		    && (temperature_c <= TEMPERATURE_SENSOR_MAX)) {

			_px4_accel.set_temperature(temperature_c);
			_px4_mag.set_temperature(temperature_c);

		} else {
			return false;
		}

		int16_t mag_x = combine(buffer.OUT_X_H_M, buffer.OUT_X_L_M);
		int16_t mag_y = combine(buffer.OUT_Y_H_M, buffer.OUT_Y_L_M);
		int16_t mag_z = combine(buffer.OUT_Z_H_M, buffer.OUT_Z_L_M);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		mag_y = math::negate(mag_y);
		mag_z = math::negate(mag_z);

		_px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
		_px4_mag.update(time_now_us, mag_x, mag_y, mag_z);

		return true;
	}

	return false;
}
