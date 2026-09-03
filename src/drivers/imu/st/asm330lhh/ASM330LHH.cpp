/**
 * @file ASM330LHH.cpp
 *
 * Driver for the ST ASM330LHH automotive 6-axis IMU connected via SPI.
 *
 * Copyright (c) 2026, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#include "ASM330LHH.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ASM330LHH::ASM330LHH(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ASM330LHH::~ASM330LHH()
{
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int ASM330LHH::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ASM330LHH::Reset()
{
	DataReadyInterruptDisable();
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ASM330LHH::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ASM330LHH::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz), %ld samples per cycle",
		 _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us, (long)_fifo_gyro_samples);
	PX4_INFO("Sensor ODR: %u Hz, FIFO sample dt: %.1f us, %u words/period",
		 (unsigned)GYRO_ODR, (double)FIFO_SAMPLE_DT, (unsigned)FIFO::MAX_WORDS_PER_PERIOD);

	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int ASM330LHH::probe()
{
	// The first SPI transaction after power-up can return garbage while the device's shared
	// I2C/SPI interface latches onto SPI (selected by the first CS falling edge), so retry the
	// WHO_AM_I read a few times before giving up.
	uint8_t whoami = 0;

	for (int attempt = 0; attempt < 3; attempt++) {
		whoami = RegisterRead(Register::WHO_AM_I);

		if (whoami == WHO_AM_I_ID) {
			return PX4_OK;
		}

		px4_usleep(1000);
	}

	DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
	return PX4_ERROR;
}

void ASM330LHH::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// Software reset
		RegisterWrite(Register::CTRL3_C, CTRL3_C_BIT::SW_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::WHO_AM_I) == WHO_AM_I_ID) {
			// Set IF_INC immediately to enable multi-byte reads
			RegisterWrite(Register::CTRL3_C, CTRL3_C_BIT::IF_INC | CTRL3_C_BIT::BDU);

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

				// Backup schedule in case the interrupt stops arriving
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			// Read FIFO status as one multi-byte transfer to avoid a race between STATUS1 and STATUS2
			struct FIFOStatusTransfer {
				uint8_t cmd{static_cast<uint8_t>(Register::FIFO_STATUS1) | DIR_READ};
				uint8_t STATUS1{0};
				uint8_t STATUS2{0};
			} fifo_status{};

			if (transfer((uint8_t *)&fifo_status, (uint8_t *)&fifo_status, sizeof(fifo_status)) != PX4_OK) {
				perf_count(_bad_transfer_perf);

			} else if (fifo_status.STATUS2 & static_cast<uint8_t>(FIFO_STATUS2_BIT::FIFO_OVR_LATCHED)) {
				// latched: catches an overrun that has already cleared itself
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else {
				// FIFO unread word count: 10-bit field (FIFO_STATUS2 bits 1:0 are bits 9:8)
				uint16_t fifo_words = fifo_status.STATUS1;

				if (fifo_status.STATUS2 & static_cast<uint8_t>(FIFO_STATUS2_BIT::DIFF_FIFO_8)) {
					fifo_words |= (1u << 8);
				}

				if (fifo_status.STATUS2 & static_cast<uint8_t>(FIFO_STATUS2_BIT::DIFF_FIFO_9)) {
					fifo_words |= (1u << 9);
				}

				// Drain whole sample periods only. A period that was still being batched when the
				// status was read stays in the FIFO for the next cycle, so a batch never carries a
				// partial period and the per-channel sample counts always agree.
				uint16_t sample_periods = fifo_words / static_cast<uint16_t>(FIFO::MAX_WORDS_PER_PERIOD);

				if (sample_periods == 0) {
					perf_count(_fifo_empty_perf);

				} else if (sample_periods > static_cast<uint16_t>(FIFO_MAX_SAMPLES)) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else {
					// Tolerate minor jitter: if one more period is waiting than expected, leave it to
					// the next iteration and back-date the batch by the period it no longer carries.
					//
					// Only while that still advances timestamp_sample. When the drain interval is a
					// single sample period - _fifo_gyro_samples == 1 - subtracting a whole period cancels
					// the whole interval, so the adjusted stamp lands on the previous one or, with
					// negative scheduling jitter, before it. vehicle_imu rejects both as a timestamp error
					// and drops the batch. Publishing the extra period instead is harmless: it is one
					// period of data, and the FIFO is drained either way.
					if (sample_periods == static_cast<uint16_t>(_fifo_gyro_samples) + 1) {
						const hrt_abstime adjusted = timestamp_sample - static_cast<int>(FIFO_SAMPLE_DT);

						if (adjusted > _last_timestamp_sample) {
							timestamp_sample = adjusted;
							sample_periods--;
						}
					}

					if (FIFORead(timestamp_sample, sample_periods * static_cast<uint16_t>(FIFO::MAX_WORDS_PER_PERIOD))) {
						success = true;
						_last_timestamp_sample = timestamp_sample;

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

			// periodically update temperature (~1 Hz)
			if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
				UpdateTemperature();
				_temperature_update_timestamp = now;
			}
		}

		break;
	}
}

void ASM330LHH::ConfigureSampleRate(int sample_rate)
{
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / FIFO_SAMPLE_DT, (float)FIFO_MAX_SAMPLES));

	_fifo_empty_interval_us = _fifo_gyro_samples * FIFO_SAMPLE_DT;

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ASM330LHH::ConfigureFIFOWatermark(uint8_t samples)
{
	// MAX_WORDS_PER_PERIOD FIFO words per sample period (gyro + accel).
	// WTM is the 8-bit FIFO_CTRL1 field; samples is capped at FIFO_MAX_SAMPLES (32) so this fits
	// and FIFO_CTRL2's WTM8 stays 0.
	const uint8_t fifo_watermark = samples * static_cast<uint8_t>(FIFO::MAX_WORDS_PER_PERIOD);

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CTRL1) {
			r.set_bits = fifo_watermark;
			r.clear_bits = static_cast<uint8_t>(~fifo_watermark & 0xFF);
		}
	}
}

bool ASM330LHH::Configure()
{
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

	// Scale and range are set once here and never touched again, so every published batch carries
	// the same scale factor.

	// Gyroscope: +/-2000 dps, 70 mdps/LSB (DS12232 table "Mechanical characteristics")
	_px4_gyro.set_scale(math::radians(70.f / 1000.f));
	_px4_gyro.set_range(math::radians(2000.f));

	// Accelerometer: +/-16 g, 0.488 mg/LSB
	_px4_accel.set_scale(0.488f * (CONSTANTS_ONE_G / 1000.f));
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);

	return success;
}

bool ASM330LHH::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t ASM330LHH::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ASM330LHH::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ASM330LHH::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool ASM330LHH::FIFORead(const hrt_abstime &timestamp_sample, uint16_t words)
{
	// Drain the requested words in one burst. RunImpl() hands over whole sample periods, at most
	// FIFO_MAX_SAMPLES of them; clamp defensively so transfer_size can never run past the buffer.
	const uint16_t words_to_read = math::min<uint16_t>(words, FIFO_MAX_WORDS);
	const size_t transfer_size = words_to_read * FIFO::WORD_SIZE + 1;

	// _fifo_buffer is reused across cycles. transfer() clobbers the command byte with the byte
	// clocked in alongside it, so restore it; the rest needs no clearing because only the words
	// covered by transfer_size are ever parsed.
	_fifo_buffer.cmd = static_cast<uint8_t>(Register::FIFO_DATA_OUT_TAG) | DIR_READ;

	if (transfer((uint8_t *)&_fifo_buffer, (uint8_t *)&_fifo_buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = FIFO_SAMPLE_DT;

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	// set if the tag stream carries more samples of a channel than the drain should have produced
	bool tag_mismatch = false;

	for (uint16_t i = 0; i < words_to_read; i++) {
		const FIFOWord &word = _fifo_buffer.words[i];

		// TAG_SENSOR is the upper 5 bits of the tag byte
		const uint8_t tag_id = word.TAG >> 3;

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		const int16_t data_x = combine(word.DATA_X_H, word.DATA_X_L);
		const int16_t y = combine(word.DATA_Y_H, word.DATA_Y_L);
		const int16_t z = combine(word.DATA_Z_H, word.DATA_Z_L);
		const int16_t data_y = (y == INT16_MIN) ? INT16_MAX : -y;
		const int16_t data_z = (z == INT16_MIN) ? INT16_MAX : -z;

		if (tag_id == static_cast<uint8_t>(FifoTag::GYRO_NC)) {
			if (gyro.samples >= FIFO_MAX_SAMPLES) {
				tag_mismatch = true;
				break;
			}

			gyro.x[gyro.samples] = data_x;
			gyro.y[gyro.samples] = data_y;
			gyro.z[gyro.samples] = data_z;
			gyro.samples++;

		} else if (tag_id == static_cast<uint8_t>(FifoTag::ACCEL_NC)) {
			if (accel.samples >= FIFO_MAX_SAMPLES) {
				tag_mismatch = true;
				break;
			}

			accel.x[accel.samples] = data_x;
			accel.y[accel.samples] = data_y;
			accel.z[accel.samples] = data_z;
			accel.samples++;
		}

		// Other tags are ignored. FIFO_CTRL4 leaves ODR_T_BATCH and DEC_TS_BATCH at 0, so neither
		// temperature nor timestamp is batched; temperature comes from UpdateTemperature().
	}

	if (tag_mismatch) {
		// the drain is bounded to whole sample periods, so a channel overrunning its share means
		// the tag stream no longer matches the configured batching - resync rather than publish it
		perf_count(_bad_transfer_perf);
		FIFOReset();
		return false;
	}

	const uint32_t error_count = perf_event_count(_bad_transfer_perf) +
				     perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf);

	if (gyro.samples > 0) {
		_px4_gyro.set_error_count(error_count);
		_px4_gyro.updateFIFO(gyro);
	}

	if (accel.samples > 0) {
		_px4_accel.set_error_count(error_count);
		_px4_accel.updateFIFO(accel);
	}

	return (gyro.samples > 0) && (accel.samples > 0);
}

void ASM330LHH::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// Switch to Bypass mode to flush FIFO
	RegisterWrite(Register::FIFO_CTRL4, FIFO_CTRL4_BIT::FIFO_MODE_BYPASS);

	// Re-enable Continuous mode
	RegisterWrite(Register::FIFO_CTRL4, FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS);

	_drdy_timestamp_sample.store(0);
}

void ASM330LHH::UpdateTemperature()
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

	// 256 LSB/degC, zero = 25 degC
	const int16_t OUT_TEMP = combine(buffer.OUT_TEMP_H, buffer.OUT_TEMP_L);
	const float temperature = (OUT_TEMP / 256.0f) + 25.0f;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

int ASM330LHH::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ASM330LHH *>(arg)->DataReady();
	return 0;
}

void ASM330LHH::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ASM330LHH::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// INT1 defaults to active-high (H_LACTIVE=0) and is pulsed (COUNTER_BDR_REG1.DATAREADY_PULSED),
	// so trigger on the rising edge
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &DataReadyInterruptCallback, this) == 0;
}

bool ASM330LHH::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}
