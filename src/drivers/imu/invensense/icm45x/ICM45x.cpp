/****************************************************************************
 *
 *   Copyright (c) 2023-2026 PX4 Development Team. All rights reserved.
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
 * @file ICM45x.cpp
 * @brief Implementation of the ICM45x Gen-4 SPI IMU family driver.
 */

#include "ICM45x.hpp"
#include "ICM45xRegisters.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/imu/ByteCursor.hpp>
#include <lib/drivers/imu/FifoBatch.hpp>
#include <lib/drivers/imu/FifoFrames.hpp>
#include <lib/drivers/imu/TdkFifoPacket.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

#include <cinttypes>
#include <climits>
#include <cmath>
#include <cstring>

using namespace time_literals;

namespace
{

constexpr uint16_t combineUnsigned(uint8_t msb, uint8_t lsb)
{
	return (static_cast<uint16_t>(msb) << 8) | lsb;
}

} // namespace

ICM45x::ICM45x(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_endpoint_guard(config),
	_profile(*static_cast<const ICM45xOptions *>(config.custom_data)->profile),
	_int2(static_cast<const ICM45xOptions *>(config.custom_data)->int2),
	_spi_mode(config.spi_mode),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_enable_clock_input(config.custom1 != 0 && _profile.clock_input),
	// All implemented CLKIN paths specify nominal ODR at 32 kHz, not 32.768 kHz.
	_sample_dt_us((1e6f / _profile.output_data_rate_hz)
		      * (_enable_clock_input ? (32 * imu::kHz) / static_cast<float>(config.custom1) : 1.f)),
	_fifo_interrupt_config0(
		isIcm56686()
		? static_cast<uint8_t>(_int2 ? icm45x_registers::icm56686::BANK_0::INT2_CONFIG0
				       : icm45x_registers::icm56686::BANK_0::INT1_CONFIG0)
		: static_cast<uint8_t>(icm45x_registers::icm45686::BANK_0::INT1_CONFIG0)),
	_fifo_interrupt_config2(
		isIcm56686()
		? static_cast<uint8_t>(_int2 ? icm45x_registers::icm56686::BANK_0::INT2_CONFIG2
				       : icm45x_registers::icm56686::BANK_0::INT1_CONFIG2)
		: static_cast<uint8_t>(icm45x_registers::icm45686::BANK_0::INT1_CONFIG2)),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());

	_px4_accel.set_range(_profile.accel_range_g * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(_profile.gyro_range_dps));
	_px4_accel.set_scale(_profile.accel_range_g * CONSTANTS_ONE_G / 32768.f);
	_px4_gyro.set_scale(math::radians(_profile.gyro_range_dps / 32768.f));
}

ICM45x::~ICM45x()
{
	dataReadyInterruptDisable();
}

int ICM45x::init()
{
	if (!_endpoint_guard.acquire()) {
		return PX4_ERROR;
	}

	if (!_dma_buffer || _bank0_config_count == 0 || _ireg_config_count == 0) {
		PX4_ERR("%s invalid register configuration", _profile.device.name);
		return PX4_ERROR;
	}

	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

bool ICM45x::reset()
{
	dataReadyInterruptDisable();
	_drdy_timestamp_sample.store(0);
	_drdy_backlog_drain = false;
	_state              = State::kReset;
	_state_ready_at     = 0;
	ScheduleClear();
	ScheduleNow();

	return true;
}

void ICM45x::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM45x::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("variant: %s", _profile.device.name);

	imu::printSpiStatus(_profile.device,
			    _spi_mode,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO interval: %" PRIu32 " us (%.1f Hz), %u samples",
		 _fifo_empty_interval_us,
		 1e6 / _fifo_empty_interval_us,
		 _fifo_gyro_samples);
	PX4_INFO("DRDY interrupt: %s%s",
		 _data_ready_interrupt_enabled ? "enabled" : "disabled",
		 _int2 ? " (INT2)" : "");

	if (_profile.clock_input) {
		PX4_INFO("clock input: %s", _enable_clock_input ? "enabled" : "disabled");
	}

	_transfer_perf.print();
	_fifo_perf.print();
	_drdy_missed_perf.print();
}

int ICM45x::probe()
{
	for (int attempt = 0; attempt < 3; ++attempt) {
		uint8_t whoami = 0;

		if (registerRead(icm45x_registers::common::WHO_AM_I, whoami) && whoami == _profile.whoami) {
			return PX4_OK;
		}

		DEVICE_DEBUG("%s unexpected WHO_AM_I 0x%02x", _profile.device.name, whoami);
	}

	return PX4_ERROR;
}

void ICM45x::configureSampleRate(int sample_rate)
{
	const float minimum_interval = _sample_dt_us;
	_fifo_empty_interval_us = static_cast<uint32_t>(
					  math::max(roundf((1e6f / math::max(sample_rate, 1)) / minimum_interval) * minimum_interval,
							  minimum_interval));

	const uint8_t fifo_limit = math::min(
					   static_cast<unsigned>(kFifoMaxSamples),
					   static_cast<unsigned>(_profile.fifo_capacity / _profile.packet_size));

	_fifo_gyro_samples = static_cast<uint8_t>(
				     math::constrain(
					     roundf(_fifo_empty_interval_us / minimum_interval),
					     1.f,
					     static_cast<float>(fifo_limit)));
	_fifo_empty_interval_us = static_cast<uint32_t>(roundf(_fifo_gyro_samples * minimum_interval));

	const icm45x_config::Context context {
		_fifo_gyro_samples,
		_enable_clock_input,
		_int2,
		_drdy_gpio != 0,
	};

	_bank0_config_count = icm45x_config::loadBank0(_profile.variant, context, _bank0_config);

	if (_bank0_config_count > icm45x_config::kMaxBank0Configs) {
		_bank0_config_count = 0;
	}

	_ireg_config_count = icm45x_config::loadIreg(_profile.variant, context, _ireg_config);

	if (_ireg_config_count > icm45x_config::kMaxIregConfigs) {
		_ireg_config_count = 0;
	}
}

void ICM45x::RunImpl()
{
	const hrt_abstime drdy_timestamp =
		(_state == State::kFifoRead && _data_ready_interrupt_enabled)
		? _drdy_timestamp_sample.fetch_and(0)
		: 0;
	const hrt_abstime now = hrt_absolute_time();

	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;
		const bool  backlog_drain    = _drdy_backlog_drain;
		_drdy_backlog_drain          = false;

		if (_data_ready_interrupt_enabled) {
			const uint32_t drdy_window_us = isIcm56686()
							? (_fifo_empty_interval_us * 2)
							: _fifo_empty_interval_us;

			if (drdy_timestamp != 0 && now >= drdy_timestamp && (now - drdy_timestamp) < drdy_window_us) {
				timestamp_sample = drdy_timestamp;

			} else if (!backlog_drain) {
				_drdy_missed_perf.count();
			}

			ScheduleDelayed(_fifo_empty_interval_us * 2);
		}

		const bool success = fifoRead(timestamp_sample);

		if (success) {
			if (_failure_count > 0) {
				--_failure_count;
			}

		} else if (++_failure_count > 10) {
			reset();
			return;
		}

		if (_profile.periodic_register_check) {
			if (hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				if (registerCheck(_bank0_config[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % _bank0_config_count;

				} else {
					_transfer_perf.bad_register.count();
					reset();
				}
			}

		} else if (success && !_profile.fifo_temperature
			   && hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
			updateTemperature();
			_temperature_update_timestamp = now;
		}

		return;
	}

	using namespace icm45x_registers;

	switch (_state) {
	case State::kReset: {
			dataReadyInterruptDisable();
			_checked_register = 0;

			if (isIcm56686()) {
				using icm56686::BANK_0;

				// Capture board pad/interface overrides only once prior to the first soft reset,
				// so a retry after timeout never overwrites saved settings with post-reset OTP defaults.
				if (!_overrides_saved) {
					if (!registerRead(static_cast<uint8_t>(BANK_0::INTF_CONFIG1_OVRD), _intf_config1_override)
					    || !registerRead(static_cast<uint8_t>(BANK_0::DRIVE_CONFIG0), _drive_config0)) {
						ScheduleDelayed(10_ms);
						break;
					}

					_overrides_saved = true;
				}

				if (!registerWrite(static_cast<uint8_t>(BANK_0::INT1_CONFIG0),
						   icm56686::INT_CONFIG0_BIT::INT1_STATUS_EN_RESET_DONE)
				    || !registerWrite(common::REG_MISC2, common::REG_MISC2_SOFT_RST)) {
					ScheduleDelayed(10_ms);
					break;
				}

				_reset_timestamp = now;
				_failure_count   = 0;
				_state           = State::kWaitForReset;
				ScheduleDelayed(10_ms);
				break;
			}

			if (!registerWrite(common::REG_MISC2, common::REG_MISC2_SOFT_RST)) {
				ScheduleDelayed(10_ms);
				break;
			}

			_reset_timestamp = now;
			_failure_count   = 0;
			_state           = State::kWaitForReset;
			ScheduleDelayed(1_ms);
			break;
		}

	case State::kWaitForReset: {
			if (isIcm56686()) {
				using icm56686::BANK_0;

				uint8_t whoami = 0;
				uint8_t status = 0;

				// INT1_STATUS_RESET_DONE clears on read; read it only after restoring overrides and matching WHO_AM_I.
				if (registerWrite(static_cast<uint8_t>(BANK_0::DRIVE_CONFIG0), _drive_config0)
				    && registerWrite(static_cast<uint8_t>(BANK_0::INTF_CONFIG1_OVRD), _intf_config1_override)
				    && registerRead(common::WHO_AM_I, whoami)
				    && whoami == _profile.whoami
				    && registerRead(static_cast<uint8_t>(BANK_0::INT1_STATUS0), status)
				    && (status & icm56686::INT1_STATUS0_BIT::INT1_STATUS_RESET_DONE) != 0) {
					_state          = State::kConfigure;
					_state_ready_at = 0;
					ScheduleNow();

				} else if (hrt_elapsed_time(&_reset_timestamp) > 100_ms) {
					PX4_DEBUG("%s reset failed, retrying", _profile.device.name);
					_state = State::kReset;
					ScheduleDelayed(100_ms);

				} else {
					ScheduleDelayed(10_ms);
				}

				break;
			}

			uint8_t whoami    = 0;
			uint8_t reg_misc2 = 0;

			if (registerRead(common::WHO_AM_I, whoami)
			    && whoami == _profile.whoami
			    && registerRead(common::REG_MISC2, reg_misc2)
			    && (reg_misc2 & common::REG_MISC2_SOFT_RST) == 0) {
				_state          = State::kConfigure;
				_state_ready_at = 0;
				ScheduleNow();

			} else if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("%s reset failed, retrying", _profile.device.name);
				_state = State::kReset;
				ScheduleDelayed(100_ms);

			} else {
				ScheduleDelayed(10_ms);
			}

			break;
		}

	case State::kConfigure: {
			if (now < _state_ready_at) {
				ScheduleDelayed(static_cast<uint32_t>(_state_ready_at - now));
				break;
			}

			const bool configured = isIcm56686() ? configureIcm56686() : configureIcm45686();

			if (configured) {
				// DS-000489 Table 1: 35 ms gyro startup time, 10 ms accel from sleep to valid data.
				_state          = State::kFifoReset;
				_state_ready_at = hrt_absolute_time() + 50_ms;
				ScheduleDelayed(50_ms);

			} else if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("%s configure failed, resetting", _profile.device.name);
				_state = State::kReset;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("%s configure failed, retrying", _profile.device.name);
				ScheduleDelayed(100_ms);
			}

			break;
		}

	case State::kFifoReset: {
			if (now < _state_ready_at) {
				ScheduleDelayed(static_cast<uint32_t>(_state_ready_at - now));
				break;
			}

			if (!fifoReset()) {
				_state = State::kReset;
				ScheduleDelayed(100_ms);
				break;
			}

			_state = State::kFifoRead;

			if (dataReadyInterruptConfigure()) {
				ScheduleDelayed(100_ms);

			} else {
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			break;
		}

	case State::kFifoRead: {
			break;
		}
	}
}

bool ICM45x::configureIcm45686()
{
	using namespace icm45x_registers::icm45686;

	bool success = _bank0_config_count > 0 && _ireg_config_count > 0;

	// DS-000489 Sections 15.6, 16.23: disable all INT1 sources (including reset-default RESET_DONE) before changing
	// INT1_CONFIG2, and keep sensors off while programming non-dynamic IREG and BANK_0 registers.
	success &= registerWrite(static_cast<uint8_t>(BANK_0::INT1_CONFIG0), 0);
	success &= registerWrite(static_cast<uint8_t>(BANK_0::PWR_MGMT0), 0);

	// DS-000489 Section 16.51: request the internal relaxation oscillator so MCLK-domain IREGs respond while sensors are off.
	success &= registerSetAndClearBits(
			   static_cast<uint8_t>(BANK_0::REG_MISC1),
			   REG_MISC1_BIT::OSC_ID_OVRD_RC_SET,
			   REG_MISC1_BIT::OSC_ID_OVRD_RC_CLEAR);
	px4_udelay(10);

	success &= registerConfigure(static_cast<uint8_t>(BANK_0::INT1_CONFIG2));

	for (uint8_t i = 0; i < _ireg_config_count; ++i) {
		success &= iregSetAndClearBits(_ireg_config[i].reg, _ireg_config[i].set_bits, _ireg_config[i].clear_bits);
	}

	success &= fifoBypassIcm45686();

	success &= registerConfigure(static_cast<uint8_t>(BANK_0::GYRO_CONFIG0));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::ACCEL_CONFIG0));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG4));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::RTC_CONFIG));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::IOC_PAD_SCENARIO_OVRD));

	// Configure non-dynamic FIFO_GYRO_EN and FIFO_ACCEL_EN (clearing FIFO_HIRES_EN and FIFO_IF_EN) while sensors are off.
	success &= registerWrite(
			   static_cast<uint8_t>(BANK_0::FIFO_CONFIG3),
			   FIFO_CONFIG3_BIT::FIFO_GYRO_EN
			   | FIFO_CONFIG3_BIT::FIFO_ACCEL_EN);

	// Release MCLK override and enable accel/gyro in low-noise mode; kConfigure waits 50 ms (>= 35 ms Table 1).
	success &= registerSetAndClearBits(
			   static_cast<uint8_t>(BANK_0::REG_MISC1),
			   0,
			   REG_MISC1_BIT::OSC_ID_OVRD_MASK);
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::PWR_MGMT0));

	return success;
}

bool ICM45x::fifoBypassIcm45686()
{
	using namespace icm45x_registers::icm45686;

	// Only clear dynamic FIFO_IF_EN here so runtime FIFO resets never toggle non-dynamic FIFO_GYRO_EN/FIFO_ACCEL_EN.
	bool success = registerSetAndClearBits(
			       static_cast<uint8_t>(BANK_0::FIFO_CONFIG3),
			       0,
			       FIFO_CONFIG3_BIT::FIFO_IF_EN);
	success &= registerSetAndClearBits(
			   static_cast<uint8_t>(BANK_0::FIFO_CONFIG0),
			   FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_SET,
			   FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_CLEAR);
	success &= registerSetAndClearBits(
			   static_cast<uint8_t>(BANK_0::FIFO_CONFIG0),
			   FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET,
			   FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_CLEAR);

	return success;
}

bool ICM45x::fifoResetIcm45686()
{
	using namespace icm45x_registers::icm45686;

	_fifo_perf.reset.count();
	_drdy_timestamp_sample.store(0);
	_drdy_backlog_drain = false;

	bool success = fifoBypassIcm45686();

	// Watermark latches only when FIFO_CONFIG1_1 (FIFO_WM[15:8]) is written, even when 0x00.
	success &= registerForceWrite(static_cast<uint8_t>(BANK_0::FIFO_CONFIG1_0));
	success &= registerForceWrite(static_cast<uint8_t>(BANK_0::FIFO_CONFIG1_1));

	// FIFO_WR_WM_GT_TH may only be written while FIFO is in bypass mode.
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG2));

	success &= fifoFlush(static_cast<uint8_t>(BANK_0::FIFO_CONFIG2), FIFO_CONFIG2_BIT::FIFO_FLUSH);

	// Enable STOP_ON_FULL first, SREG-FIFO interface (FIFO_IF_EN) next, and INT1_CONFIG0 last.
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG0));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG3));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::INT1_CONFIG0));

	for (uint8_t i = 0; i < _bank0_config_count; ++i) {
		success &= registerCheck(_bank0_config[i]);
	}

	return success;
}

bool ICM45x::configureIcm56686()
{
	using namespace icm45x_registers::icm56686;

	bool success = _bank0_config_count > 0 && _ireg_config_count > 0;

	// Registers without dynamic-change support must be configured with both sensors off.
	success &= registerWrite(static_cast<uint8_t>(BANK_0::INT1_CONFIG0), 0);

	if (_fifo_interrupt_config0 != static_cast<uint8_t>(BANK_0::INT1_CONFIG0)) {
		success &= registerWrite(_fifo_interrupt_config0, 0);
	}

	success &= registerWrite(static_cast<uint8_t>(BANK_0::PWR_MGMT0), 0);
	success &= registerWrite(static_cast<uint8_t>(BANK_0::PWR_MGMT_AUX1), 0);

	success &= fifoBypassIcm56686();

	success &= registerConfigure(static_cast<uint8_t>(BANK_0::ACCEL_CONFIG0));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::GYRO_CONFIG0));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG4));

	// Configure non-dynamic FIFO_HIRES_EN, FIFO_GYRO_EN, FIFO_ACCEL_EN while sensors are off.
	success &= registerWrite(static_cast<uint8_t>(BANK_0::FIFO_CONFIG3),
				 FIFO_CONFIG3_BIT::FIFO_HIRES_EN
				 | FIFO_CONFIG3_BIT::FIFO_GYRO_EN
				 | FIFO_CONFIG3_BIT::FIFO_ACCEL_EN);

	for (uint8_t i = 0; i < _ireg_config_count; ++i) {
		success &= iregSetAndClearBits(_ireg_config[i].reg, _ireg_config[i].set_bits, _ireg_config[i].clear_bits);
	}

	success &= registerConfigure(static_cast<uint8_t>(BANK_0::PWR_MGMT0));

	return success;
}

bool ICM45x::fifoBypassIcm56686()
{
	using namespace icm45x_registers::icm56686;

	bool success = registerSetAndClearBits(
			       static_cast<uint8_t>(BANK_0::FIFO_CONFIG3),
			       0,
			       FIFO_CONFIG3_BIT::FIFO_IF_EN);
	success &= registerSetAndClearBits(
			   static_cast<uint8_t>(BANK_0::FIFO_CONFIG0),
			   FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS,
			   FIFO_CONFIG0_BIT::FIFO_MODE_MASK);
	success &= registerSetAndClearBits(
			   static_cast<uint8_t>(BANK_0::FIFO_CONFIG0),
			   FIFO_CONFIG0_BIT::FIFO_DEPTH_2K,
			   static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_DEPTH_MASK & ~FIFO_CONFIG0_BIT::FIFO_DEPTH_2K));

	return success;
}

bool ICM45x::fifoFlush(uint8_t fifo_config2_reg, uint8_t flush_bit)
{
	uint8_t fifo_config2 = 0;

	if (!registerRead(fifo_config2_reg, fifo_config2)
	    || !registerWrite(fifo_config2_reg, static_cast<uint8_t>(fifo_config2 | flush_bit))) {
		return false;
	}

	px4_udelay(10);

	for (int i = 0; i < 100; ++i) {
		if (!registerRead(fifo_config2_reg, fifo_config2)) {
			return false;
		}

		if ((fifo_config2 & flush_bit) == 0) {
			return true;
		}

		px4_udelay(10);
	}

	return false;
}

bool ICM45x::fifoResetIcm56686()
{
	using namespace icm45x_registers::icm56686;

	_fifo_perf.reset.count();
	_drdy_timestamp_sample.store(0);
	_drdy_backlog_drain = false;

	bool success = fifoBypassIcm56686();

	// Force-write both watermark bytes so FIFO_CONFIG1_1 latches the threshold.
	success &= registerForceWrite(static_cast<uint8_t>(BANK_0::FIFO_CONFIG1_0));
	success &= registerForceWrite(static_cast<uint8_t>(BANK_0::FIFO_CONFIG1_1));

	// FIFO_WR_WM_GT_TH may only be written while FIFO is in bypass mode.
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG2));

	success &= fifoFlush(static_cast<uint8_t>(BANK_0::FIFO_CONFIG2), FIFO_CONFIG2_BIT::FIFO_FLUSH);

	// Enable the FIFO first, the SREG-FIFO interface (FIFO_IF_EN) last.
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG0));
	success &= registerConfigure(static_cast<uint8_t>(BANK_0::FIFO_CONFIG3));
	success &= registerConfigure(_fifo_interrupt_config2);
	success &= registerConfigure(_fifo_interrupt_config0);

	for (uint8_t i = 0; i < _bank0_config_count; ++i) {
		success &= registerCheck(_bank0_config[i]);
	}

	return success;
}

bool ICM45x::fifoReset()
{
	return isIcm56686() ? fifoResetIcm56686() : fifoResetIcm45686();
}

bool ICM45x::registerRead(uint8_t reg, uint8_t &value)
{
	set_frequency(_register_frequency);

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(reg | icm45x_registers::DIR_READ);
	cmd[1] = 0;

	if (device::SPI::transfer(cmd, cmd, 2) != PX4_OK) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	value = cmd[1];
	return true;
}

bool ICM45x::registerWrite(uint8_t reg, uint8_t value)
{
	set_frequency(_register_frequency);

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = reg;
	cmd[1] = value;

	if (device::SPI::transfer(cmd, cmd, 2) != PX4_OK) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	return true;
}

bool ICM45x::registerSetAndClearBits(uint8_t reg, uint8_t set_bits, uint8_t clear_bits)
{
	uint8_t value = 0;

	if (!registerRead(reg, value)) {
		return false;
	}

	const uint8_t new_value = (value & ~clear_bits) | set_bits;
	return value == new_value || registerWrite(reg, new_value);
}

bool ICM45x::registerConfigure(const Bank0Config &register_config)
{
	return registerSetAndClearBits(register_config.reg, register_config.set_bits, register_config.clear_bits);
}

bool ICM45x::registerConfigure(uint8_t reg)
{
	for (uint8_t i = 0; i < _bank0_config_count; ++i) {
		if (_bank0_config[i].reg == reg) {
			return registerConfigure(_bank0_config[i]);
		}
	}

	return false;
}

bool ICM45x::registerForceWrite(uint8_t reg)
{
	for (uint8_t i = 0; i < _bank0_config_count; ++i) {
		if (_bank0_config[i].reg == reg) {
			return registerWrite(_bank0_config[i].reg, _bank0_config[i].set_bits);
		}
	}

	return false;
}

bool ICM45x::registerCheck(const Bank0Config &register_config)
{
	uint8_t value = 0;

	if (!registerRead(register_config.reg, value)) {
		return false;
	}

	const bool success = (register_config.set_bits == 0 || (value & register_config.set_bits) == register_config.set_bits)
			     && (register_config.clear_bits == 0 || (value & register_config.clear_bits) == 0);

	if (!success) {
		PX4_DEBUG("%s reg 0x%02x value 0x%02x (set 0x%02x clear 0x%02x)",
			  _profile.device.name,
			  register_config.reg,
			  value,
			  register_config.set_bits,
			  register_config.clear_bits);
	}

	return success;
}

bool ICM45x::waitForIregReady()
{
	using namespace icm45x_registers::common;

	// Honour the datasheet 4 us minimum gap before polling IREG_DONE.
	px4_udelay(4);

	for (int i = 0; i < 20; ++i) {
		uint8_t status = 0;

		if (!registerRead(REG_MISC2, status)) {
			return false;
		}

		if (status & REG_MISC2_IREG_DONE) {
			return true;
		}

		px4_udelay(10);
	}

	return false;
}

bool ICM45x::iregRead(uint16_t reg, uint8_t &value)
{
	using namespace icm45x_registers::common;

	if (!waitForIregReady()) {
		return false;
	}

	set_frequency(_register_frequency);

	uint8_t *address = _dma_buffer.data();
	address[0] = IREG_ADDR_15_8;
	address[1] = static_cast<uint8_t>(reg >> 8);
	address[2] = static_cast<uint8_t>(reg & 0xFF);

	if (device::SPI::transfer(address, address, 3) != PX4_OK) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	px4_udelay(10);

	if (!waitForIregReady() || !registerRead(IREG_DATA, value)) {
		return false;
	}

	// Reading IREG_DATA starts a prefetch of the next indirect address.
	px4_udelay(10);
	return waitForIregReady();
}

bool ICM45x::iregWrite(uint16_t reg, uint8_t value)
{
	using namespace icm45x_registers::common;

	if (!waitForIregReady()) {
		return false;
	}

	set_frequency(_register_frequency);

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = IREG_ADDR_15_8;
	cmd[1] = static_cast<uint8_t>(reg >> 8);
	cmd[2] = static_cast<uint8_t>(reg & 0xFF);
	cmd[3] = value;

	if (device::SPI::transfer(cmd, cmd, 4) != PX4_OK) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	px4_udelay(10);
	return waitForIregReady();
}

bool ICM45x::iregSetAndClearBits(uint16_t reg, uint8_t set_bits, uint8_t clear_bits)
{
	uint8_t value = 0;

	if (!iregRead(reg, value)) {
		return false;
	}

	const uint8_t new_value = (value & ~clear_bits) | set_bits;

	if (value != new_value && !iregWrite(reg, new_value)) {
		return false;
	}

	uint8_t check = 0;
	return iregRead(reg, check) && check == new_value;
}

uint16_t ICM45x::fifoReadCount()
{
	set_frequency(_data_frequency);

	uint8_t *fifo_count = _dma_buffer.data();
	const unsigned reads = _profile.double_read_fifo_count ? 2 : 1;

	for (unsigned i = 0; i < reads; ++i) {
		fifo_count[0] = static_cast<uint8_t>(_profile.fifo_count_reg | icm45x_registers::DIR_READ);
		fifo_count[1] = 0;
		fifo_count[2] = 0;

		if (device::SPI::transfer(fifo_count, fifo_count, 3) != PX4_OK) {
			_transfer_perf.bad_transfer.count();
			return 0;
		}
	}

	// Little-endian SREG_CTRL configuration returns FIFO_COUNT LSByte first.
	return combineUnsigned(fifo_count[2], fifo_count[1]);
}

bool ICM45x::fifoRead(const hrt_abstime &timestamp_sample)
{
	const uint16_t fifo_frames = fifoReadCount();

	if (fifo_frames == 0) {
		_fifo_perf.empty.count();
		return false;
	}

	const uint16_t overflow_threshold = static_cast<uint16_t>(_profile.fifo_capacity / _profile.packet_size);

	if (fifo_frames >= overflow_threshold) {
		_fifo_perf.overflow.count();

		if (!fifoReset()) {
			PX4_DEBUG("%s FIFO reset failed", _profile.device.name);
		}

		return false;
	}

	const uint8_t frames        = static_cast<uint8_t>(math::min<uint16_t>(kFifoMaxSamples, fifo_frames));
	const size_t  transfer_size = kFifoTransferPrefix + frames * _profile.packet_size;

	set_frequency(_data_frequency);
	uint8_t *fifo_transfer = _dma_buffer.data();

	if (transfer_size > _dma_buffer.capacity() || transfer_size > _profile.device.max_transfer_bytes) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	memset(fifo_transfer, 0, transfer_size);
	fifo_transfer[0] = static_cast<uint8_t>(_profile.fifo_data_reg | icm45x_registers::DIR_READ);

	if (device::SPI::transfer(fifo_transfer, fifo_transfer, transfer_size) != PX4_OK) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	// ICM45x uses pulsed INT_MODE = 0 with FIFO_WR_WM_GT_TH = 1, so each sample at or above the watermark
	// re-pulses INT and timestamps the newest FIFO frame; back-calculate when draining multi-batch backlogs.
	const hrt_abstime batch_timestamp = imu::adjustFifoBacklogTimestamp(
			timestamp_sample,
			false,
			fifo_frames,
			frames,
			_sample_dt_us);

	sensor_accel_fifo_s accel {};
	sensor_gyro_fifo_s  gyro  {};
	static_assert(sizeof(accel.x) == sizeof(gyro.x), "FIFO channel capacities must match");
	accel.timestamp_sample = batch_timestamp;
	gyro.timestamp_sample  = batch_timestamp;

	imu::FifoSampleStats temperatures;
	imu::TdkFifoSample   sample {};

	const uint8_t *data = &fifo_transfer[kFifoTransferPrefix];
	const auto decode = [this, &temperatures](imu::ByteCursor & cursor, imu::TdkFifoSample & decoded) {
		const uint8_t *packet = cursor.take(_profile.packet_size);

		if (!imu::decodeTdkFifoPacket(
			    packet,
			    _profile.packet_size,
			    true,
			    true,
			    false,
			    true,
			    decoded)) {
			return imu::FifoFrame::kInvalid;
		}

		if (_profile.fifo_temperature
		    && decoded.temperature != INT16_MIN
		    && !temperatures.add(decoded.temperature)) {
			return imu::FifoFrame::kInvalid;
		}

		if (decoded.accel_valid && decoded.gyro_valid) {
			return imu::FifoFrame::kBoth;
		}

		if (decoded.accel_valid) {
			return imu::FifoFrame::kAccel;
		}

		if (decoded.gyro_valid) {
			return imu::FifoFrame::kGyro;
		}

		return imu::FifoFrame::kMetadata;
	};

	const imu::FifoDecodeResult result = imu::decodeFifoFrames<imu::FifoAxisMapping::kFlipYZ>(
			data, frames * _profile.packet_size, accel, gyro, sample, decode);

	if (result.status == imu::FifoDecodeStatus::kEmpty) {
		return false;
	}

	if (result.status != imu::FifoDecodeStatus::kComplete) {
		_transfer_perf.bad_transfer.count();

		if (!fifoReset()) {
			PX4_DEBUG("%s FIFO reset failed", _profile.device.name);
		}

		return false;
	}

	if (_profile.fifo_temperature && !temperatures.empty() && !processTemperature(temperatures)) {
		return false;
	}

	accel.dt = _sample_dt_us;
	gyro.dt  = _sample_dt_us;

	const uint64_t errors = errorCount();

	if (gyro.samples > 0) {
		_px4_gyro.set_error_count(errors);
		_px4_gyro.updateFIFO(gyro);
	}

	if (accel.samples > 0) {
		_px4_accel.set_error_count(errors);
		_px4_accel.updateFIFO(accel);
	}

	if (fifo_frames > frames) {
		_drdy_backlog_drain = true;
		ScheduleNow();
	}

	return true;
}

bool ICM45x::processTemperature(const imu::FifoSampleStats &temperatures)
{
	if (temperatures.empty()) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	float average = 0.f;

	if (!temperatures.meanWithin(1000.f, average)) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	const float temperature = (average / _profile.temperature_sensitivity) + _profile.temperature_offset;

	if (!PX4_ISFINITE(temperature)) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	return true;
}

void ICM45x::updateTemperature()
{
	set_frequency(_data_frequency);

	uint8_t *temperature_buf = _dma_buffer.data();
	temperature_buf[0] = static_cast<uint8_t>(_profile.temperature_reg | icm45x_registers::DIR_READ);
	temperature_buf[1] = 0;
	temperature_buf[2] = 0;

	if (device::SPI::transfer(temperature_buf, temperature_buf, 3) != PX4_OK) {
		_transfer_perf.bad_transfer.count();
		return;
	}

	// Little-endian SREG_CTRL configuration returns TEMP_DATA LSByte first.
	const int16_t raw_temp = imu::readLe16(&temperature_buf[1]);
	const float   temp_c   = (raw_temp / _profile.temperature_sensitivity) + _profile.temperature_offset;

	if (PX4_ISFINITE(temp_c)) {
		_px4_accel.set_temperature(temp_c);
		_px4_gyro.set_temperature(temp_c);
	}
}

uint64_t ICM45x::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}

int ICM45x::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM45x *>(arg)->dataReady();
	return 0;
}

void ICM45x::dataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ICM45x::dataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		_data_ready_interrupt_enabled = false;
		return false;
	}

	// ICM-56686 uses active-high rising edge; ICM-45686 uses active-low falling edge.
	const bool rising_edge = isIcm56686();

	_data_ready_interrupt_enabled =
		px4_arch_gpiosetevent(_drdy_gpio, rising_edge, !rising_edge, true, &dataReadyInterruptCallback, this) == 0;

	return _data_ready_interrupt_enabled;
}

bool ICM45x::dataReadyInterruptDisable()
{
	if (!_data_ready_interrupt_enabled || _drdy_gpio == 0) {
		return false;
	}

	_data_ready_interrupt_enabled = false;

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}
