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

#include "TdkIcm45686.hpp"

#include <lib/mathlib/mathlib.h>

#include <cmath>
#include <cstring>

using namespace time_literals;
using namespace frequency_literals;

// Startup and recovery use the board's size optimization; acquisition remains separately optimized.

TdkIcm45686::TdkIcm45686(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_register_frequency(imu::spiConfigFrequency(spiModel(), config.custom2)),
	_data_frequency(imu::spiDataFrequency(spiModel(), config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_enable_clock_input(config.custom1 != 0),
	// All implemented CLKIN paths specify nominal ODR at 32 kHz, not 32.768 kHz.
	_sample_dt_us((1e6f / kOutputDataRateHz)
		      * (_enable_clock_input ? 32_kHz / static_cast<float>(config.custom1) : 1.f)),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
	_px4_accel.set_range(kAccelRangeG * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(kGyroRangeDps));

	_px4_accel.set_scale(CONSTANTS_ONE_G * kAccelRangeG / 32768.f);
	_px4_gyro.set_scale(math::radians(kGyroRangeDps / 32768.f));
}

int TdkIcm45686::init()
{
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

int TdkIcm45686::probe()
{
	for (int attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;

		const uint8_t whoami = registerRead(AddressSpace::kBank0, kWhoAmIRegister);

		if (!_transfer_failed && whoami == kWhoAmI) {
			return PX4_OK;
		}

		DEVICE_DEBUG("%s unexpected WHO_AM_I 0x%02x", spiModel().name, whoami);
	}

	return PX4_ERROR;
}

void TdkIcm45686::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

bool TdkIcm45686::reset()
{
	_state               = State::kReset;
	_fifo_state_ready_at = 0;
	dataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();

	return true;
}

bool TdkIcm45686::resetComplete()
{

	if (registerRead(AddressSpace::kBank0, kWhoAmIRegister) != kWhoAmI) {
		return false;
	}

	if ((registerRead(AddressSpace::kBank0, kResetReg) & kResetBit) != 0) {
		return false;
	}

	return !_transfer_failed;
}

bool TdkIcm45686::configure()
{
	// Static filters, interface and interrupt settings must not change while
	// measuring. FIFO mode stays at reset bypass until every setting is verified.
	if ((registerRead(AddressSpace::kBank0, kPowerReg) & kPwrLowNoise) != 0 || _transfer_failed) {
		return false;
	}

	using namespace tdk_icm45686_registers;

	// FIFO_CONFIG2's watermark comparator may only change in bypass mode.
	const uint8_t mode = registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG0));

	if ((mode & static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_CLEAR)) != 0 || _transfer_failed) {
		return false;
	}

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		if (!deferredConfiguration(_register_cfg[i])) {
			registerSetAndClearBits(_register_cfg[i]);
		}
	}

	// The high-byte write commits the watermark, including when it is zero.
	registerWrite(AddressSpace::kBank0,
		      static_cast<uint16_t>(tdk_icm45686_registers::Register::BANK_0::FIFO_CONFIG1_1), _fifo_gyro_samples >> 8);

	bool success = _register_cfg_count > 0;

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		if (!deferredConfiguration(_register_cfg[i])) {
			success &= registerCheck(_register_cfg[i]);
		}
	}

	return success && !_transfer_failed;
}

void TdkIcm45686::configureSampleRate(int sample_rate)
{
	const float sample_dt          = _sample_dt_us;
	const float requested_interval = 1e6f / math::max(sample_rate, 1);

	// Round to complete sample periods, with a minimum of one period.
	_fifo_empty_interval_us =
		static_cast<uint16_t>(
			math::max(
				roundf(requested_interval / sample_dt) * sample_dt,
				sample_dt));

	// Bound each read by both the publication buffer and the device FIFO.
	const uint8_t fifo_limit =
		math::min(
			static_cast<unsigned>(kFifoMaxSamples),
			static_cast<unsigned>(kFifoCapacity / kPacketSize));

	_fifo_gyro_samples =
		static_cast<uint8_t>(
			math::constrain(
				roundf(_fifo_empty_interval_us / sample_dt),
				1.f,
				static_cast<float>(fifo_limit)));
	_fifo_empty_interval_us = static_cast<uint16_t>(roundf(_fifo_gyro_samples * sample_dt));

	// The FIFO watermark is expressed in records.
	_register_cfg_count = tdk_icm45686_config::load(
				      _fifo_gyro_samples,
				      _enable_clock_input,
				      _register_cfg);

	if (_register_cfg_count > tdk_icm45686_config::kRegisterCount) {
		_register_cfg_count = 0;
	}
}

void TdkIcm45686::startFifoRead()
{
	fifoReset();

	bool configured = _register_cfg_count > 0;

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		configured &= registerCheck(_register_cfg[i]);
	}

	if (!configured || _transfer_failed) {
		_state = State::kReset;
		ScheduleDelayed(100_ms);

		return;
	}

	_state                        = State::kFifoRead;
	_data_ready_interrupt_enabled = dataReadyInterruptConfigure();

	if (_data_ready_interrupt_enabled) {
		ScheduleDelayed(100_ms);

	} else {
		ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
	}
}

bool TdkIcm45686::deferredConfiguration(const RegisterConfig &config) const
{
	using namespace tdk_icm45686_registers;

	return config.space == AddressSpace::kBank0
	       && (config.reg == kPowerReg
		   || config.reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG0)
		   || config.reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG3));
}

void TdkIcm45686::fifoReset()
{
	_fifo_perf.reset.count();

	using namespace tdk_icm45686_registers;

	// Disable every FIFO source before entering bypass mode and restoring the configured depth.
	registerSetAndClearBits({
		AddressSpace::kBank0, static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG3), 0,
		static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_ES1_EN)
		| static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_ES0_EN)
		| static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_HIRES_EN)
		| static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_GYRO_EN)
		| static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_ACCEL_EN)
		| static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_IF_EN)
	});
	registerSetAndClearBits({
		AddressSpace::kBank0, static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG0),
		static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_SET),
		static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_CLEAR)
	});
	registerSetAndClearBits({
		AddressSpace::kBank0, static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG0),
		static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET), 0
	});

	// Restore stop-on-full mode, then enable only the inertial channels and FIFO interface.
	registerSetAndClearBits({
		AddressSpace::kBank0, static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG0),
		static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET),
		static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_CLEAR)
	});
	registerSetAndClearBits({
		AddressSpace::kBank0, static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG3),
		static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_GYRO_EN)
		| static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_ACCEL_EN)
		| static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_IF_EN), 0
	});

	_drdy_timestamp_sample.store(0);
}

void TdkIcm45686::runInitialization(hrt_abstime now)
{
	switch (_state) {
	case State::kReset: {
			memset(_checked_register, 0, sizeof(_checked_register));
			registerWrite(AddressSpace::kBank0, kResetReg, kResetBit);

			if (_transfer_failed) {
				ScheduleDelayed(100_ms);
				break;
			}

			_reset_timestamp     = now;
			_failure_count       = 0;
			_state               = State::kWaitForReset;
			ScheduleDelayed(1_ms);
			break;
		}

	case State::kWaitForReset: {
			if (resetComplete()) {

				if (_transfer_failed) {
					_state = State::kReset;
					ScheduleDelayed(100_ms);
					break;
				}

				_state = State::kConfigure;
				ScheduleDelayed(1_ms);

			} else if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("%s reset failed, retrying", spiModel().name);
				_state = State::kReset;
				ScheduleDelayed(100_ms);

			} else {
				ScheduleDelayed(10_ms);
			}

			break;
		}

	case State::kConfigure: {
			if (configure()) {
				registerWrite(AddressSpace::kBank0, kPowerReg, kPwrLowNoise);

				if (!_transfer_failed) {
					// Preserve the conservative measurement startup guard before enabling FIFO.
					// This is a host timing bound, not a replacement for packet validity checks.
					_state               = State::kFifoEnable;
					_fifo_state_ready_at = hrt_absolute_time() + 50_ms;
					ScheduleDelayed(50_ms);

				} else {
					_state = State::kReset;
					ScheduleDelayed(100_ms);
				}

			} else {
				_state = hrt_elapsed_time(&_reset_timestamp) > 1000_ms ? State::kReset : State::kConfigure;
				ScheduleDelayed(100_ms);
			}

			break;
		}

	case State::kFifoEnable: {
			// A pending ScheduleNow request must not shorten the measurement startup guard.
			if (now < _fifo_state_ready_at) {
				ScheduleDelayed(static_cast<uint32_t>(_fifo_state_ready_at - now));
				break;
			}

			for (uint8_t i = 0; i < _register_cfg_count; ++i) {
				const RegisterConfig &cfg = _register_cfg[i];

				if (deferredConfiguration(cfg) && cfg.reg != kPowerReg) {
					registerSetAndClearBits(cfg);
				}
			}

			if (_transfer_failed) {
				_state = State::kReset;
				ScheduleDelayed(100_ms);

			} else {
				// Let initial ODR-change records enter the FIFO before the guarded startup flush.
				_state               = State::kFifoReset;
				_fifo_state_ready_at = hrt_absolute_time() + 1_ms;
				ScheduleDelayed(1_ms);

			}

			break;
		}

	case State::kFifoReset: {
			if (now < _fifo_state_ready_at) {
				ScheduleDelayed(static_cast<uint32_t>(_fifo_state_ready_at - now));
				break;
			}

			startFifoRead();
			break;
		}

	case State::kFifoRead: {
			break; // Handled by the acquisition fast path in RunImpl().
		}
	}
}
