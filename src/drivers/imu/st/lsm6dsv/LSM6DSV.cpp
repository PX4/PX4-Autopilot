/****************************************************************************
 *
 *   Copyright (c) 2024-2026 PX4 Development Team. All rights reserved.
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

#include "LSM6DSV.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

LSM6DSV::LSM6DSV(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

LSM6DSV::~LSM6DSV()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int LSM6DSV::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool LSM6DSV::Reset()
{
	DataReadyInterruptDisable();
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void LSM6DSV::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void LSM6DSV::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz), %ld samples per cycle",
		 _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us, (long)_fifo_gyro_samples);
	PX4_INFO("Sensor ODR: %u Hz (HAODR mode1), FIFO sample dt: %.0f us",
		 (unsigned)GYRO_ODR, (double)FIFO_SAMPLE_DT);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int LSM6DSV::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHO_AM_I_ID) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void LSM6DSV::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// Software reset
		RegisterWrite(Register::CTRL3, CTRL3_BIT::SW_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::WHO_AM_I) == WHO_AM_I_ID) {
			// Set IF_INC immediately to enable multi-byte reads
			RegisterWrite(Register::CTRL3, CTRL3_BIT::IF_INC | CTRL3_BIT::BDU);

			_state = STATE::CONFIGURE;
			ScheduleDelayed(10_ms);

		} else {
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
			_state = STATE::FIFO_RESET;
			ScheduleDelayed(1_ms);

		} else {
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_RESET:
		_state = STATE::FIFO_READ;
		FIFOReset();

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;
			bool success = false;

			if (_data_ready_interrupt_enabled) {
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			// Read FIFO status (atomic multi-byte read to avoid race between STATUS1 and STATUS2)
			struct FIFOStatusTransfer {
				uint8_t cmd{static_cast<uint8_t>(Register::FIFO_STATUS1) | DIR_READ};
				uint8_t STATUS1{0};
				uint8_t STATUS2{0};
			} fifo_status{};

			if (transfer((uint8_t *)&fifo_status, (uint8_t *)&fifo_status, sizeof(fifo_status)) != PX4_OK) {
				perf_count(_bad_transfer_perf);

			} else if (fifo_status.STATUS2 & static_cast<uint8_t>(FIFO_STATUS2_BIT::FIFO_OVR_LATCHED)) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else {
				// FIFO unread word count: 9-bit field (FIFO_STATUS2 bit0 is bit8)
				// Each sample period produces 2 words (1 gyro word + 1 accel word)
				uint16_t fifo_words = fifo_status.STATUS1;

				if (fifo_status.STATUS2 & static_cast<uint8_t>(FIFO_STATUS2_BIT::DIFF_FIFO_8)) {
					fifo_words |= (1u << 8);
				}

				// Convert word count to sample periods for comparisons against _fifo_gyro_samples / FIFO_MAX_SAMPLES
				const uint16_t sample_periods = fifo_words / 2;

				if (sample_periods == 0) {
					perf_count(_fifo_empty_perf);

				} else if (sample_periods > static_cast<uint16_t>(FIFO_MAX_SAMPLES)) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else {

					// tolerate minor jitter, leave sample to next iteration if behind by only 1
					if (sample_periods == static_cast<uint16_t>(_fifo_gyro_samples) + 1) {
						timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
						fifo_words -= 2;
					}

					if (FIFORead(timestamp_sample, fifo_words)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			// periodically check configuration registers
			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					perf_count(_bad_register_perf);
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void LSM6DSV::ConfigureSampleRate(int sample_rate)
{
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

bool LSM6DSV::Configure()
{
	// First enable HAODR mode, then configure ODR registers
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// Verify all registers
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// Gyroscope: ±2000 dps, 70 mdps/LSB (ST datasheet)
	_px4_gyro.set_scale(math::radians(70.f / 1000.f));
	_px4_gyro.set_range(math::radians(2000.f));

	// Accelerometer: ±16g, 0.488 mg/LSB (ST datasheet)
	_px4_accel.set_scale(0.488f * (CONSTANTS_ONE_G / 1000.f));
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);

	return success;
}

bool LSM6DSV::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t LSM6DSV::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM6DSV::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM6DSV::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool LSM6DSV::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = FIFO_SAMPLE_DT;

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	// Read FIFO word by word: each word is 7 bytes (tag + 6 data)
	for (uint16_t i = 0; i < samples; i++) {
		// Read tag + data in one transfer (1 cmd byte + 7 data bytes)
		struct FIFOWordTransfer {
			uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA_OUT_TAG) | DIR_READ};
			uint8_t TAG{0};
			uint8_t DATA_X_L{0};
			uint8_t DATA_X_H{0};
			uint8_t DATA_Y_L{0};
			uint8_t DATA_Y_H{0};
			uint8_t DATA_Z_L{0};
			uint8_t DATA_Z_H{0};
		} buffer{};

		if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			continue;
		}

		// Decode tag from upper 5 bits
		const uint8_t tag_id = buffer.TAG >> 3;

		const int16_t data_x = combine(buffer.DATA_X_H, buffer.DATA_X_L);
		const int16_t data_y = combine(buffer.DATA_Y_H, buffer.DATA_Y_L);
		const int16_t data_z = combine(buffer.DATA_Z_H, buffer.DATA_Z_L);

		if (tag_id == static_cast<uint8_t>(FifoTag::GYRO_NC)) {
			if (gyro.samples < (sizeof(gyro.x) / sizeof(gyro.x[0]))) {
				gyro.x[gyro.samples] = data_x;
				gyro.y[gyro.samples] = data_y;
				gyro.z[gyro.samples] = data_z;
				gyro.samples++;
			}

		} else if (tag_id == static_cast<uint8_t>(FifoTag::ACCEL_NC)) {
			if (accel.samples < (sizeof(accel.x) / sizeof(accel.x[0]))) {
				accel.x[accel.samples] = data_x;
				accel.y[accel.samples] = data_y;
				accel.z[accel.samples] = data_z;
				accel.samples++;
			}

		} else if (tag_id == static_cast<uint8_t>(FifoTag::TEMPERATURE)) {
			const int16_t temp_raw = combine(buffer.DATA_X_H, buffer.DATA_X_L);
			const float temperature = (temp_raw / 256.0f) + 25.0f;

			if (PX4_ISFINITE(temperature)) {
				_px4_accel.set_temperature(temperature);
				_px4_gyro.set_temperature(temperature);
			}
		}

		// Other tags (TIMESTAMP, etc.) are silently ignored
	}

	// Publish
	if (gyro.samples > 0) {
		_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
					  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));
		_px4_gyro.updateFIFO(gyro);
	}

	if (accel.samples > 0) {
		_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
					   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));
		_px4_accel.updateFIFO(accel);
	}

	return (accel.samples > 0) && (gyro.samples > 0);
}

void LSM6DSV::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// Switch to Bypass mode to flush FIFO
	RegisterWrite(Register::FIFO_CTRL4, FIFO_CTRL4_BIT::FIFO_MODE_BYPASS);

	// Re-enable Continuous mode
	RegisterWrite(Register::FIFO_CTRL4, FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS);

	_drdy_timestamp_sample.store(0);
}

void LSM6DSV::UpdateTemperature()
{
	struct TransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::OUT_TEMP_L) | DIR_READ};
		uint8_t OUT_TEMP_L{0};
		uint8_t OUT_TEMP_H{0};
	} buffer{};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	// 256 LSB/°C, zero = 25°C
	const int16_t OUT_TEMP = combine(buffer.OUT_TEMP_H, buffer.OUT_TEMP_L);
	const float temperature = (OUT_TEMP / 256.0f) + 25.0f;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

int LSM6DSV::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<LSM6DSV *>(arg)->DataReady();
	return 0;
}

void LSM6DSV::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool LSM6DSV::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// INT1 defaults to active-high (H_LACTIVE=0), use rising edge
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &DataReadyInterruptCallback, this) == 0;
}

bool LSM6DSV::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

void LSM6DSV::ConfigureFIFOWatermark(uint8_t samples)
{
	// accel + gyro = 2 FIFO words per sample period
	const uint8_t fifo_watermark = samples * 2;

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CTRL1) {
			r.set_bits = fifo_watermark;
			r.clear_bits = static_cast<uint8_t>(~fifo_watermark & 0xFF);
		}
	}
}
