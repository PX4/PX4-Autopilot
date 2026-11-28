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

#include "TdkMpu6000.hpp"

#include <lib/mathlib/mathlib.h>

#include <cmath>
#include <cstring>

using namespace time_literals;
using namespace frequency_literals;
using namespace tdk_direct_registers;

// Startup and recovery use the board's size optimization; acquisition remains separately optimized.

TdkMpu6000::TdkMpu6000(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_register_frequency(imu::spiConfigFrequency(spiModel(), config.custom2)),
	_data_frequency(imu::spiDataFrequency(spiModel(), config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());

}

int TdkMpu6000::init()
{
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

int TdkMpu6000::probe()
{

	const uint8_t whoami = registerRead(Register::WHO_AM_I);

	if (_transfer_failed || whoami != kWhoAmI) {
		DEVICE_DEBUG("%s: unexpected WHO_AM_I 0x%02x", spiModel().name, whoami);

		return PX4_ERROR;
	}

	return PX4_OK;
}

void TdkMpu6000::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

bool TdkMpu6000::reset(uint32_t delay_us)
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

bool TdkMpu6000::resetComplete()
{
	if (registerRead(Register::WHO_AM_I) != kWhoAmI) {
		return false;
	}

	if (registerRead(Register::PWR_MGMT_1) != kResetPwrValue) {
		return false;
	}

	return !_transfer_failed;
}

void TdkMpu6000::runInitialization(hrt_abstime now)
{
	switch (_state) {
	case State::kReset: {
			_checked_register    = 0;

			deviceReset();

			if (_transfer_failed) {
				ScheduleDelayed(100_ms);
				break;
			}

			_reset_timestamp = now;
			_failure_count   = 0;
			_state           = State::kWaitForReset;
			ScheduleDelayed(100_ms);
			break;
		}

	case State::kWaitForReset: {
			if (resetComplete()) {
				if (_transfer_failed) {
					_state = State::kReset;
					ScheduleDelayed(100_ms);
					break;
				}

				if (!wakeAndResetSignalPath()) {
					reset(100_ms);
					break;
				}

				_state = State::kConfigure;
				ScheduleDelayed(100_ms);

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

bool TdkMpu6000::configure()
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

void TdkMpu6000::configureSampleRate(int sample_rate)
{
	// MPU6000 drains gyro at up to 2 kHz while tracking its 1 kHz accel phase across reads.
	// The accel repetition divider must not impose a slower gyro publication interval.
	const float min_interval = kFifoSampleDt * (4);

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

void TdkMpu6000::configureAccel()
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

void TdkMpu6000::configureGyro()
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

bool TdkMpu6000::wakeAndResetSignalPath()
{
	registerWrite(
		Register::PWR_MGMT_1,
		static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0));

	registerWrite(
		Register::SIGNAL_PATH_RESET,
		static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::GYRO_RESET)
		| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::ACCEL_RESET)
		| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::TEMP_RESET));
	registerWrite(
		Register::USER_CTRL,
		static_cast<uint8_t>(USER_CTRL_BIT::SIG_COND_RST)
		| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS));

	return !_transfer_failed;
}

bool TdkMpu6000::fifoReset()
{
	_fifo_perf.reset.count();
	registerWrite(Register::FIFO_EN, 0);
	registerSetAndClearBits(Register::USER_CTRL, static_cast<uint8_t>(USER_CTRL_BIT::FIFO_RST), static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN));
	_drdy_count.store(0);
	_fifo_accel_samples_count = 0;
	memset(_last_accel, 0, sizeof(_last_accel));
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
