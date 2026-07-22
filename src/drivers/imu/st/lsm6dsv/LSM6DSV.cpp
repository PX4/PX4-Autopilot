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

static constexpr bool IsSupportedWhoAmI(uint8_t whoami)
{
	return (whoami == WHO_AM_I_ID) || (whoami == WHO_AM_I_DSK320X) || (whoami == WHO_AM_I_HIGHG);
}

// Returns true if any axis is at (or near) the int16 limit. Only meaningful for the low-g channel,
// whose ±16 g / 0.488 mg/LSB scaling actually reaches the rail (16 g = 32787 counts). Limits are
// pulled in by sqrt(2) plus a small margin so a sample that would clip only after rotation is also
// caught (the same approach used by the ICM45686 driver).
static constexpr bool SampleClips(int16_t x, int16_t y, int16_t z)
{
	constexpr int16_t hi = static_cast<int16_t>(INT16_MAX / 1.41421356f) - 100;
	constexpr int16_t lo = static_cast<int16_t>(INT16_MIN / 1.41421356f) + 100;
	return (x >= hi) || (x <= lo) || (y >= hi) || (y <= lo) || (z >= hi) || (z <= lo);
}

// High-g full-scale, pinned at each variant's top range. The high-g channel is only ever published
// while the low-g channel clips, i.e. above 16 g, where the finer resolution of a smaller range buys
// nothing — the fallback already steps in with the high-g channel's ±1.5 g typ zero-g offset
// uncalibrated. The top range is instead what keeps an impact's first peak unclipped, since that
// peak lands within a few ms of the low-g clipping and no runtime full-scale change could follow it.
// Sensitivities are taken verbatim from the datasheet.
//
// LSM6DSV80X (datasheet DS14764 Table 3): FS_XL_HG=010 -> ±80 g, 3.904 mg/LSB.
static constexpr HighGFullScale HIGH_G_FS_DSV80X{0x02, 80, 3.904f};

// LSM6DSV320X (datasheet DS14623 Table 150 full-scale + Table 3 sensitivity): FS_XL_HG=100 ->
// ±320 g, 10.417 mg/LSB. The two parts share WHO_AM_I 0x73 and are selected explicitly at start
// (see WHO_AM_I_HIGHG); codes above 010 are reserved on the 80X.
static constexpr HighGFullScale HIGH_G_FS_DSV320X{0x04, 320, 10.417f};

// Convert a high-g full-scale to PX4Accelerometer scale (m/s^2 per LSB) and range (m/s^2).
static void GetHighGScaleRange(const HighGFullScale &fs, float &scale, float &range)
{
	scale = fs.scale_mg_per_lsb * (CONSTANTS_ONE_G / 1000.f);
	range = static_cast<float>(fs.range_g) * CONSTANTS_ONE_G;
}

LSM6DSV::LSM6DSV(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation),
	_highg_variant_arg(config.custom1)
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

	// Update device IDs based on probed variant (for proper identification in logs)
	switch (_device_variant) {
	case DeviceVariant::LSM6DSV16X:
		// default — no change needed
		break;

	case DeviceVariant::LSM6DSV32X:
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSV32X);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSV32X);
		break;

	case DeviceVariant::LSM6DSK320X:
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSK320X);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSK320X);
		break;

	case DeviceVariant::LSM6DSV320X:
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSV320X);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSV320X);
		break;

	case DeviceVariant::LSM6DSV80X:
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSV80X);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ST_LSM6DSV80X);
		break;
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

	const char *variant_str = "unknown";

	switch (_device_variant) {
	case DeviceVariant::LSM6DSV16X:   variant_str = "LSM6DSV16X";   break;

	case DeviceVariant::LSM6DSV32X:   variant_str = "LSM6DSV32X";   break;

	case DeviceVariant::LSM6DSK320X:  variant_str = "LSM6DSK320X";  break;

	case DeviceVariant::LSM6DSV320X:  variant_str = "LSM6DSV320X";  break;

	case DeviceVariant::LSM6DSV80X:   variant_str = "LSM6DSV80X";   break;
	}

	PX4_INFO("Device variant: %s", variant_str);
	PX4_INFO("FIFO empty interval: %d us (%.1f Hz), %ld samples per cycle",
		 _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us, (long)_fifo_gyro_samples);
	PX4_INFO("Sensor ODR: %u Hz (HAODR), FIFO sample dt: %.1f us, %u words/period",
		 (unsigned)_sensor_odr, (double)_fifo_sample_dt, (unsigned)_fifo_words_per_period);

	if (_high_g_enabled) {
		PX4_INFO("High-g fallback: enabled, full-scale +/-%u g", (unsigned)_hg_fs->range_g);
	}

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int LSM6DSV::probe()
{
	// The first SPI transaction after power-up can return garbage while the device's shared
	// I2C/I3C/SPI interface latches onto SPI (selected by the first CS falling edge), so retry
	// the WHO_AM_I read a few times before giving up.
	uint8_t whoami = 0;

	for (int attempt = 0; attempt < 3; attempt++) {
		whoami = RegisterRead(Register::WHO_AM_I);

		if (IsSupportedWhoAmI(whoami)) {
			break;
		}

		px4_usleep(1000);
	}

	if (whoami == WHO_AM_I_DSK320X) {
		_device_variant = DeviceVariant::LSM6DSK320X;
		PX4_INFO("detected LSM6DSK320X");
		UpdateVariantRegisterConfig();
		return PX4_OK;
	}

	if (whoami == WHO_AM_I_HIGHG) {
		// The LSM6DSV80X and LSM6DSV320X share this WHO_AM_I and register map and cannot be told
		// apart over SPI, so the physically-installed part is selected explicitly at start with the
		// -T argument (config.custom1): 320 -> LSM6DSV320X, otherwise (incl. 80 / unset) -> 80X.
		if (_highg_variant_arg == 320) {
			_device_variant = DeviceVariant::LSM6DSV320X;
			PX4_INFO("LSM6DSV320X selected (WHO_AM_I 0x73)");

		} else {
			_device_variant = DeviceVariant::LSM6DSV80X;

			if (_highg_variant_arg != 80) {
				PX4_WARN("WHO_AM_I 0x73 is ambiguous (80X/320X); defaulting to LSM6DSV80X. Pass -T 80 or -T 320.");

			} else {
				PX4_INFO("LSM6DSV80X selected (WHO_AM_I 0x73)");
			}
		}

		UpdateVariantRegisterConfig();
		return PX4_OK;
	}

	if (whoami != WHO_AM_I_ID) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	// 16X vs 32X: read CTRL8 bit2 (hardware-reserved, differs per variant)
	const uint8_t ctrl8 = RegisterRead(Register::CTRL8);
	_device_variant = (ctrl8 & Bit2) ? DeviceVariant::LSM6DSV32X : DeviceVariant::LSM6DSV16X;

	PX4_INFO("detected %s", _device_variant == DeviceVariant::LSM6DSV32X ? "LSM6DSV32X" : "LSM6DSV16X");

	UpdateVariantRegisterConfig();
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
		if (IsSupportedWhoAmI(RegisterRead(Register::WHO_AM_I))) {
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
				// Each sample period produces _fifo_words_per_period words
				// (gyro + low-g, plus high-g on the LSM6DSV80X / LSM6DSV320X)
				uint16_t fifo_words = fifo_status.STATUS1;

				if (fifo_status.STATUS2 & static_cast<uint8_t>(FIFO_STATUS2_BIT::DIFF_FIFO_8)) {
					fifo_words |= (1u << 8);
				}

				// Convert word count to sample periods for comparisons against _fifo_gyro_samples / FIFO_MAX_SAMPLES
				const uint16_t sample_periods = fifo_words / _fifo_words_per_period;

				if (sample_periods == 0) {
					perf_count(_fifo_empty_perf);

				} else if (sample_periods > static_cast<uint16_t>(FIFO_MAX_SAMPLES)) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else {

					// tolerate minor jitter, leave sample to next iteration if behind by only 1
					if (sample_periods == static_cast<uint16_t>(_fifo_gyro_samples) + 1) {
						timestamp_sample -= static_cast<int>(_fifo_sample_dt);
						fifo_words -= _fifo_words_per_period;
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
	const float min_interval = _fifo_sample_dt;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / _fifo_sample_dt, (float)FIFO_MAX_SAMPLES));

	_fifo_empty_interval_us = _fifo_gyro_samples * _fifo_sample_dt;

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void LSM6DSV::UpdateVariantRegisterConfig()
{
	// Select the high-g full-scale for this variant (nullptr = no high-g channel)
	switch (_device_variant) {
	case DeviceVariant::LSM6DSV80X:
		_hg_fs = &HIGH_G_FS_DSV80X;
		break;

	case DeviceVariant::LSM6DSV320X:
		_hg_fs = &HIGH_G_FS_DSV320X;
		break;

	default:
		_hg_fs = nullptr;
		break;
	}

	_high_g_enabled = (_hg_fs != nullptr);

	if (_high_g_enabled) {
		// 7.68 kHz (HAODR_SEL=00), with the high-g channel co-batched into the FIFO
		_sensor_odr = GYRO_ODR_HIGHG;
		_fifo_words_per_period = 3; // gyro + low-g + high-g

	} else {
		_sensor_odr = GYRO_ODR;
		_fifo_words_per_period = 2; // gyro + low-g
	}

	_fifo_sample_dt = 1e6f / (float)_sensor_odr;

	for (auto &r : _register_cfg) {
		switch (r.reg) {
		case Register::CTRL6: // gyroscope full-scale
			switch (_device_variant) {
			case DeviceVariant::LSM6DSV80X:
			case DeviceVariant::LSM6DSV320X:
				r.set_bits = CTRL6_BIT::FS_G_4000DPS_HIGHG; // ±4000 dps (CTRL6 bit3 = 1)
				break;

			case DeviceVariant::LSM6DSK320X:
				r.set_bits = CTRL6_BIT::FS_G_2000DPS_DSK320X;
				break;

			default:
				r.set_bits = CTRL6_BIT::FS_G_2000DPS;
				break;
			}

			break;

		case Register::CTRL8: // low-g accelerometer full-scale (±16 g) + LPF2 bandwidth
			r.set_bits = (_device_variant == DeviceVariant::LSM6DSV32X)
				     ? static_cast<uint8_t>(CTRL8_BIT::FS_XL_16G_DSV32X | CTRL8_BIT::LPF2_BW_ODR_DIV_10)
				     : static_cast<uint8_t>(CTRL8_BIT::FS_XL_16G | CTRL8_BIT::LPF2_BW_ODR_DIV_10);
			break;

		case Register::HAODR_CFG: // HAODR ODR set selection
			if (_high_g_enabled) {
				r.set_bits = 0;                              // HAODR_SEL=00 (1920/3840/7680 Hz set)
				r.clear_bits = HAODR_CFG_BIT::HAODR_SEL_MASK;

			} else {
				r.set_bits = HAODR_CFG_BIT::HAODR_MODE1;     // HAODR_SEL=01 (2000 Hz set)
				r.clear_bits = 0;
			}

			break;

		case Register::CTRL1: // accelerometer ODR + high-accuracy ODR mode
			r.set_bits = _high_g_enabled
				     ? static_cast<uint8_t>(HAODR_SEL0_ODR_7680HZ | CTRL1_BIT::CTRL1_MODE_HAODR)
				     : static_cast<uint8_t>(HAODR_MODE1_ODR_2000HZ | CTRL1_BIT::CTRL1_MODE_HAODR);
			break;

		case Register::CTRL2: // gyroscope ODR + high-accuracy ODR mode
			r.set_bits = _high_g_enabled
				     ? static_cast<uint8_t>(HAODR_SEL0_ODR_7680HZ | CTRL2_BIT::CTRL2_MODE_HAODR)
				     : static_cast<uint8_t>(HAODR_MODE1_ODR_2000HZ | CTRL2_BIT::CTRL2_MODE_HAODR);
			break;

		case Register::FIFO_CTRL3: // FIFO batch data rate (accel + gyro)
			r.set_bits = _high_g_enabled
				     ? static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_GY_7680 | FIFO_CTRL3_BIT::BDR_XL_7680)
				     : static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_GY_HAODR | FIFO_CTRL3_BIT::BDR_XL_HAODR);
			break;

		case Register::COUNTER_BDR_REG1: // high-g FIFO batching
			r.set_bits = _high_g_enabled ? static_cast<uint8_t>(COUNTER_BDR_REG1_BIT::XL_HG_BATCH_EN) : 0;
			break;

		case Register::CTRL1_XL_HG: // high-g ODR + full-scale
			if (_high_g_enabled) {
				const uint8_t val = static_cast<uint8_t>(CTRL1_XL_HG_BIT::ODR_XL_HG_7680 | _hg_fs->fs_code);
				r.set_bits = val;
				r.clear_bits = static_cast<uint8_t>(~val); // the whole register is ours, so check every bit

			} else {
				r.set_bits = 0;
				r.clear_bits = 0;
			}

			break;

		default:
			break;
		}
	}

	// recompute FIFO timing / watermark for the (possibly changed) ODR and word count
	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
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

	if (_high_g_enabled) {
		// Gyroscope: ±4000 dps, 140 mdps/LSB (ST datasheet) — LSM6DSV80X / LSM6DSV320X
		_px4_gyro.set_scale(math::radians(140.f / 1000.f));
		_px4_gyro.set_range(math::radians(4000.f));

	} else {
		// Gyroscope: ±2000 dps, 70 mdps/LSB (ST datasheet)
		_px4_gyro.set_scale(math::radians(70.f / 1000.f));
		_px4_gyro.set_range(math::radians(2000.f));
	}

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

bool LSM6DSV::FIFORead(const hrt_abstime &timestamp_sample, uint16_t words)
{
	// Drain the whole FIFO in one burst. RunImpl() already bounds the word count; clamp defensively
	// so transfer_size can never run past the buffer.
	const uint16_t words_to_read = math::min(words, FIFO_MAX_WORDS);
	const size_t transfer_size = words_to_read * FIFO::WORD_SIZE + 1;

	FIFOTransferBuffer buffer{};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = _fifo_sample_dt;

	sensor_accel_fifo_s accel{}; // low-g channel (±16 g)
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = _fifo_sample_dt;

	sensor_accel_fifo_s accel_hg{}; // high-g channel (LSM6DSV80X / LSM6DSV320X)
	accel_hg.timestamp_sample = timestamp_sample;
	accel_hg.samples = 0;
	accel_hg.dt = _fifo_sample_dt;

	for (uint16_t i = 0; i < words_to_read; i++) {
		const FIFOWord &word = buffer.words[i];

		// Decode tag from upper 5 bits
		const uint8_t tag_id = word.TAG >> 3;

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		const int16_t data_x = combine(word.DATA_X_H, word.DATA_X_L);
		const int16_t y = combine(word.DATA_Y_H, word.DATA_Y_L);
		const int16_t z = combine(word.DATA_Z_H, word.DATA_Z_L);
		const int16_t data_y = (y == INT16_MIN) ? INT16_MAX : -y;
		const int16_t data_z = (z == INT16_MIN) ? INT16_MAX : -z;

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

		} else if (tag_id == static_cast<uint8_t>(FifoTag::ACCEL_HG)) {
			if (accel_hg.samples < (sizeof(accel_hg.x) / sizeof(accel_hg.x[0]))) {
				accel_hg.x[accel_hg.samples] = data_x;
				accel_hg.y[accel_hg.samples] = data_y;
				accel_hg.z[accel_hg.samples] = data_z;
				accel_hg.samples++;
			}

		} else if (tag_id == static_cast<uint8_t>(FifoTag::TEMPERATURE)) {
			const int16_t temp_raw = combine(word.DATA_X_H, word.DATA_X_L);
			const float temperature = (temp_raw / 256.0f) + 25.0f;

			if (PX4_ISFINITE(temperature)) {
				_px4_accel.set_temperature(temperature);
				_px4_gyro.set_temperature(temperature);
			}
		}

		// Other tags (TIMESTAMP, etc.) are silently ignored
	}

	const uint32_t error_count = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				     perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf);

	// Evaluate clipping on the raw (pre-rotation) low-g samples before any publish rotates them in place.
	bool low_g_clipping = false;

	if (_high_g_enabled) {
		for (uint8_t n = 0; n < accel.samples; n++) {
			if (SampleClips(accel.x[n], accel.y[n], accel.z[n])) {
				low_g_clipping = true;
				break;
			}
		}
	}

	// Publish gyro
	if (gyro.samples > 0) {
		_px4_gyro.set_error_count(error_count);
		_px4_gyro.updateFIFO(gyro);
	}

	// Publish accelerometer: fall back to the high-g channel while the low-g channel is clipping.
	bool accel_published = false;

	if (_high_g_enabled && low_g_clipping && (accel_hg.samples > 0)) {
		float scale, range;
		GetHighGScaleRange(*_hg_fs, scale, range);
		_px4_accel.set_range(range);
		_px4_accel.set_scale(scale);
		_px4_accel.set_error_count(error_count);
		_px4_accel.updateFIFO(accel_hg);
		accel_published = true;

	} else if (accel.samples > 0) {
		// low-g: ±16 g, 0.488 mg/LSB (ST datasheet)
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		_px4_accel.set_scale(0.488f * (CONSTANTS_ONE_G / 1000.f));
		_px4_accel.set_error_count(error_count);
		_px4_accel.updateFIFO(accel);
		accel_published = true;
	}

	return accel_published && (gyro.samples > 0);
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
	// _fifo_words_per_period FIFO words per sample period (gyro + low-g, plus high-g on the 80X).
	// WTM is the 8-bit FIFO_CTRL1 field; samples is capped at FIFO_MAX_SAMPLES (32) so this fits.
	const uint8_t fifo_watermark = samples * _fifo_words_per_period;

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CTRL1) {
			r.set_bits = fifo_watermark;
			r.clear_bits = static_cast<uint8_t>(~fifo_watermark & 0xFF);
		}
	}
}
