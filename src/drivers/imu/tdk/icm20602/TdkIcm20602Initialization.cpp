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

#include "TdkIcm20602.hpp"

#include <lib/mathlib/mathlib.h>

#include <cmath>
#include <cstring>

using namespace time_literals;
using namespace frequency_literals;
using namespace tdk_direct_registers;

// Startup and recovery use the board's size optimization; acquisition remains separately optimized.

TdkIcm20602::TdkIcm20602(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_register_frequency(imu::spiConfigFrequency(spiModel(), config.custom2)),
	_data_frequency(imu::spiDataFrequency(spiModel(), config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
	buildRegisterConfig();
}

int TdkIcm20602::init()
{
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

int TdkIcm20602::probe()
{

	const uint8_t whoami = registerRead(Register::WHO_AM_I);

	if (_transfer_failed || whoami != kWhoAmI) {
		DEVICE_DEBUG("%s: unexpected WHO_AM_I 0x%02x", spiModel().name, whoami);

		return PX4_ERROR;
	}

	return PX4_OK;
}

void TdkIcm20602::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

bool TdkIcm20602::reset(uint32_t delay_us)
{
	_state = State::kReset;
	dataReadyInterruptDisable();
	ScheduleClear();

	if (delay_us > 0) {
		ScheduleDelayed(delay_us);

	} else {
		ScheduleNow();
	}

	return true;
}

bool TdkIcm20602::resetComplete()
{
	if (registerRead(Register::WHO_AM_I) != kWhoAmI) {
		return false;
	}

	if (registerRead(Register::PWR_MGMT_1) != kResetPwrValue) {
		return false;
	}

	return (registerRead(Register::CONFIG) == 0x80) && !_transfer_failed;
}

bool TdkIcm20602::configure()
{
	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		registerSetAndClearBits(_register_cfg[i].reg, _register_cfg[i].set_bits, _register_cfg[i].clear_bits);
	}

	bool success = true;

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		success &= registerCheck(_register_cfg[i]);
	}

	configureAccel();
	configureGyro();

	return success && !_transfer_failed;
}

void TdkIcm20602::configureSampleRate(int sample_rate)
{
	// Drain complete accel repetition groups without changing the native gyro sample rate.
	const float min_interval = kFifoSampleDt * (kSamplesPerTransfer);

	_fifo_empty_interval_us =
		math::max(
			roundf((1e6f / static_cast<float>(math::max(sample_rate, 1))) / min_interval) * min_interval,
			min_interval);

	_fifo_gyro_samples =
		roundf(
			math::min(
				static_cast<float>(_fifo_empty_interval_us) / (1e6f / kGyroRate),
				static_cast<float>(kFifoMaxSamples)));

	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / kGyroRate);
}

void TdkIcm20602::configureAccel()
{
	const uint8_t accel_fs = registerRead(Register::ACCEL_CONFIG) & (Bit4 | Bit3);

	switch (accel_fs) {
	case static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_2G): {
			_px4_accel.set_scale(CONSTANTS_ONE_G / 16384.f);
			_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
			break;
		}

	case static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_4G): {
			_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);
			_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
			break;
		}

	case static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_8G): {
			_px4_accel.set_scale(CONSTANTS_ONE_G / 4096.f);
			_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
			break;
		}

	case static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G): {
			_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
			_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
			break;
		}
	}
}

void TdkIcm20602::configureGyro()
{
	const uint8_t gyro_fs   = registerRead(Register::GYRO_CONFIG) & (Bit4 | Bit3);
	float         range_dps = 0.f;

	switch (gyro_fs) {
	case static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_250_DPS): {
			range_dps = 250.f;
			break;
		}

	case static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_500_DPS): {
			range_dps = 500.f;
			break;
		}

	case static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_1000_DPS): {
			range_dps = 1000.f;
			break;
		}

	case static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_2000_DPS): {
			range_dps = 2000.f;
			break;
		}
	}

	_px4_gyro.set_scale(math::radians(range_dps / 32768.f));
	_px4_gyro.set_range(math::radians(range_dps));
}

bool TdkIcm20602::wakeAndResetSignalPath()
{
	registerWrite(
		Register::PWR_MGMT_1,
		static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0));

	registerWrite(Register::I2C_IF, Bit6);
	registerWrite(
		Register::SIGNAL_PATH_RESET,
		static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::ACCEL_RESET)
		| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::TEMP_RESET));
	registerSetAndClearBits(Register::USER_CTRL, static_cast<uint8_t>(USER_CTRL_BIT::SIG_COND_RST), 0);

	return !_transfer_failed;
}

void TdkIcm20602::runInitialization(hrt_abstime now)
{
	switch (_state) {
	case State::kReset: {
			_factory_read_failed = false;
			_checked_register    = 0;
			buildRegisterConfig();
			deviceReset();

			if (_transfer_failed) {
				ScheduleDelayed(100_ms);
				break;
			}

			_reset_timestamp = now;
			_failure_count   = 0;
			_state           = State::kWaitForReset;
			ScheduleDelayed(2_ms);
			break;
		}

	case State::kWaitForReset: {
			if (resetComplete()) {
				// Learn each device's factory offsets after reset, then include them in periodic register checks.
				_factory_read_failed |= !storeCheckedRegisterValue(Register::XG_OFFS_TC_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::XG_OFFS_TC_L);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::YG_OFFS_TC_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::YG_OFFS_TC_L);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::ZG_OFFS_TC_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::ZG_OFFS_TC_L);

				_factory_read_failed |= !storeCheckedRegisterValue(Register::XA_OFFSET_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::XA_OFFSET_L);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::YA_OFFSET_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::YA_OFFSET_L);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::ZA_OFFSET_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::ZA_OFFSET_L);

				if (_transfer_failed || _factory_read_failed) {
					_state = State::kReset;
					ScheduleDelayed(100_ms);
					break;
				}

				if (!wakeAndResetSignalPath()) {
					reset(100_ms);
					break;
				}

				_state = State::kConfigure;
				ScheduleDelayed(35_ms);

			} else if (hrt_elapsed_time(&_reset_timestamp) > 1_s) {
				PX4_DEBUG("%s reset failed, retrying", spiModel().name);
				_state = State::kReset;
				ScheduleDelayed(100_ms);

			} else {
				ScheduleDelayed(10_ms);
			}

			break;
		}

	case State::kConfigure: {
			if (configure() && !_transfer_failed) {
				// Flush and restore the configured FIFO sources before enabling callbacks.
				if (!fifoReset()) {
					reset(100_ms);
					break;
				}

				_state = State::kFifoRead;
				_data_ready_interrupt_enabled = dataReadyInterruptConfigure();

				if (_data_ready_interrupt_enabled) {
					ScheduleDelayed(100_ms);

				} else {
					ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
				}

			} else {
				_state = hrt_elapsed_time(&_reset_timestamp) > 1_s ? State::kReset : State::kConfigure;
				ScheduleDelayed(100_ms);
			}

			break;
		}

	case State::kFifoRead: {
			break; // Handled by the acquisition fast path in RunImpl().
		}
	}
}

void TdkIcm20602::buildRegisterConfig()
{

	constexpr uint8_t fifo_enable = (static_cast<uint8_t>(FIFO_EN_BIT::GYRO_FIFO_EN) | static_cast<uint8_t>(FIFO_EN_BIT::ACCEL_FIFO_EN));

	// Restore fixed masks before relearning device-specific factory offsets.
	static constexpr RegisterConfig defaults[] {
		{
			Register::CONFIG,
			static_cast<uint8_t>(CONFIG_BIT::FIFO_MODE)
			| static_cast<uint8_t>(CONFIG_BIT::DLPF_CFG_BYPASS_DLPF_8KHZ)
		},

		{
			Register::GYRO_CONFIG,
			static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_2000_DPS),
			static_cast<uint8_t>(GYRO_CONFIG_BIT::FCHOICE_B_8KHZ_BYPASS_DLPF)
		},
		{
			Register::ACCEL_CONFIG,
			static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G)
		},

		{
			Register::ACCEL_CONFIG2,
			static_cast<uint8_t>(ACCEL_CONFIG2_BIT::ACCEL_FCHOICE_B),
			0
		},

		// Keep the packet layout limited to the configured inertial and temperature channels.
		{
			Register::FIFO_EN,
			fifo_enable,
			0
		},

		{
			Register::INT_PIN_CFG,
			static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_LEVEL)
			| static_cast<uint8_t>(INT_PIN_CFG_BIT::LATCH_INT_EN)
			| static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_RD_CLEAR)
		},
		{
			Register::INT_ENABLE,
			0,
			static_cast<uint8_t>(INT_ENABLE_BIT::DATA_RDY_INT_EN)
		},
		{Register::FIFO_WM_TH1, 0},
		{Register::FIFO_WM_TH2, 0},

		{
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN)
		},
		{Register::I2C_IF, Bit6},

		{
			Register::PWR_MGMT_1,
			static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0),
			static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP)
		},

		// Filled from checked factory reads after reset; zero masks initially impose no value.
		{Register::XG_OFFS_TC_H, 0},
		{Register::XG_OFFS_TC_L, 0},
		{Register::YG_OFFS_TC_H, 0},
		{Register::YG_OFFS_TC_L, 0},
		{Register::ZG_OFFS_TC_H, 0},
		{Register::ZG_OFFS_TC_L, 0},

		// Filled from checked factory reads after reset; zero masks initially impose no value.
		{Register::XA_OFFSET_H, 0},
		{Register::XA_OFFSET_L, 0},
		{Register::YA_OFFSET_H, 0},
		{Register::YA_OFFSET_L, 0},
		{Register::ZA_OFFSET_H, 0},
		{Register::ZA_OFFSET_L, 0},
	};

	static_assert(sizeof(defaults) == sizeof(_register_cfg), "Complete fixed register configuration required");

	memcpy(_register_cfg, defaults, sizeof(defaults));
	configureFifoWatermark();
}

void TdkIcm20602::configureFifoWatermark()
{

	const uint16_t threshold = _fifo_gyro_samples * kFifoPacketSize;

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		if (_register_cfg[i].reg == Register::CONFIG) {
			_register_cfg[i].clear_bits |= Bit7;

		} else if (_register_cfg[i].reg == Register::FIFO_WM_TH1) {
			// FIFO_WM_TH[9:8] only; leave the reserved high bits untouched.
			_register_cfg[i].set_bits   = (threshold >> 8) & 0x03;
			_register_cfg[i].clear_bits = 0x03 & ~_register_cfg[i].set_bits;

		} else if (_register_cfg[i].reg == Register::FIFO_WM_TH2) {
			_register_cfg[i].set_bits   = threshold & 0xff;
			_register_cfg[i].clear_bits = static_cast<uint8_t>(~_register_cfg[i].set_bits);
		}
	}
}

bool TdkIcm20602::fifoReset()
{
	_fifo_perf.reset.count();
	registerWrite(Register::FIFO_EN, 0);
	registerSetAndClearBits(Register::USER_CTRL, static_cast<uint8_t>(USER_CTRL_BIT::FIFO_RST), static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN));
	_drdy_timestamp_sample.store(0);

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		if (_register_cfg[i].reg == Register::FIFO_EN || _register_cfg[i].reg == Register::USER_CTRL) {
			registerSetAndClearBits(_register_cfg[i].reg, _register_cfg[i].set_bits, _register_cfg[i].clear_bits);

			// A successful transfer does not prove that the channel restore was accepted.
			if (!registerCheck(_register_cfg[i])) {
				if (!_transfer_failed) {
					_transfer_perf.bad_register.count();
				}

				return false;
			}
		}
	}

	return !_transfer_failed;
}
