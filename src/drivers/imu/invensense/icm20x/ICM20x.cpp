/****************************************************************************
 *
 *   Copyright (c) 2020-2026 PX4 Development Team. All rights reserved.
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
 * @file ICM20x.cpp
 * @brief Implementation of the ICM20x banked-register SPI IMU family driver.
 */

#include "ICM20x.hpp"

#include <lib/drivers/imu/ByteCursor.hpp>
#include <lib/drivers/imu/FifoBatch.hpp>
#include <lib/drivers/imu/ImuSamplePhase.hpp>

#include <cinttypes>
#include <cmath>
#include <cstring>

using namespace time_literals;
using namespace icm20x_registers;

ICM20x::ICM20x(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_endpoint_guard(config),
	_profile(*static_cast<const Profile *>(config.custom_data)),
	_spi_mode(config.spi_mode),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
}

ICM20x::~ICM20x() = default;

int ICM20x::init()
{
	if (!_endpoint_guard.acquire()) {
		return PX4_ERROR;
	}

	if (!_dma_buffer) {
		return PX4_ERROR;
	}

	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

bool ICM20x::reset(uint32_t delay_us)
{
	_state = State::kReset;
	_config_check_failure_count = 0;

	ScheduleClear();

	if (delay_us > 0) {
		ScheduleDelayed(delay_us);

	} else {
		ScheduleNow();
	}

	return true;
}

void ICM20x::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM20x::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("type: %s (six-axis only, polling)", _profile.device.name);

	imu::printSpiStatus(_profile.device,
			    _spi_mode,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO empty interval: %" PRIu32 " us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	_transfer_perf.print();
	_fifo_perf.print();
}

int ICM20x::probe()
{
	for (int i = 0; i < 3; i++) {
		_transfer_failed = false;

		// Read identity from bank 0, independently of the cached bank or the state left by a reset.
		selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0, true);

		const uint8_t whoami = registerRead(Register::BANK_0::WHO_AM_I);

		if (!_transfer_failed && whoami == _profile.whoami) {
			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
	}

	return PX4_ERROR;
}

void ICM20x::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;

		// always check current FIFO count
		bool           success    = false;
		bool           phase_wait = false;
		const uint16_t fifo_count = fifoReadCount();

		if (fifo_count >= FIFO::SIZE) {
			_fifo_perf.overflow.count();

			if (!fifoReset()) {
				reset(100_ms);
				return;
			}

		} else if (fifo_count == 0) {
			if (!_transfer_failed) {
				_fifo_perf.empty.count();
			}

		} else {
			// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure
			uint16_t samples = fifo_count / sizeof(FIFO::DATA);

			if (samples > _fifo_gyro_samples) {
				// grab desired number of samples, but reschedule next cycle sooner
				const int extra_samples = samples - _fifo_gyro_samples;

				timestamp_sample = imu::adjustFifoBacklogTimestamp(
							   now,
							   false,
							   samples,
							   static_cast<uint16_t>(_fifo_gyro_samples),
							   _fifo_sample_dt_us);

				samples = _fifo_gyro_samples;

				if (_fifo_gyro_samples > extra_samples) {
					// reschedule to run when a total of _fifo_gyro_samples should be available in the FIFO
					const uint32_t reschedule_delay_us =
						static_cast<uint32_t>(roundf((_fifo_gyro_samples - extra_samples) * _fifo_sample_dt_us));
					ScheduleOnInterval(_fifo_empty_interval_us, reschedule_delay_us);

				} else {
					// otherwise reschedule to run immediately
					ScheduleOnInterval(_fifo_empty_interval_us);
				}

			} else if (samples < _fifo_gyro_samples) {
				// reschedule next cycle to catch the desired number of samples
				ScheduleOnInterval(_fifo_empty_interval_us,
						   static_cast<uint32_t>(roundf((_fifo_gyro_samples - samples) * _fifo_sample_dt_us)));
				phase_wait = !_transfer_failed
					     && samples > 0
					     && now >= _last_sample_timestamp
					     && (now - _last_sample_timestamp) < 50_ms;
			}

			if (samples == _fifo_gyro_samples) {
				if (fifoRead(timestamp_sample, static_cast<uint8_t>(samples))) {
					success                = true;
					_last_sample_timestamp = now;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}
			}
		}

		if (!success && !phase_wait) {
			// Preserve recovery scheduled by fifoRead() after an unsuccessful FIFO reset.
			if (_state == State::kReset) {
				return;
			}

			_failure_count++;

			// full reset if things are failing consistently
			if (_failure_count > 10) {
				reset();

				return;
			}
		}

		if ((!success && !phase_wait) || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
			// Clear any FIFO-path transfer failure so configuration check performs live SPI transactions.
			_transfer_failed = false;

			// Check Bank 2 first and Bank 0 second so the chip remains parked in USER_BANK_0 for FIFO reads.
			if (registerCheck(_register_bank2_config[_checked_register_bank2])
			    && registerCheck(_register_bank0_config[_checked_register_bank0])) {
				_config_check_failure_count  = 0;
				_last_config_check_timestamp = now;
				_checked_register_bank0      = (_checked_register_bank0 + 1) % kRegisterBank0ConfigCount;
				_checked_register_bank2      = (_checked_register_bank2 + 1) % kRegisterBank2ConfigCount;

			} else if (!_transfer_failed) {
				// register content mismatch, force reset
				_transfer_perf.bad_register.count();
				reset();

			} else if (++_config_check_failure_count > 10) {
				reset();
			}

		} else {
			// periodically update temperature (~1 Hz)
			if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
				updateTemperature();
				_temperature_update_timestamp = now;
			}
		}

		return;
	}

	switch (_state) {
	case State::kReset: {
			_fifo_sample_dt_us   = kFifoSampleDt;
			_register_bank_valid = false;
			// PWR_MGMT_1: Device Reset
			registerWrite(Register::BANK_0::PWR_MGMT_1, static_cast<uint8_t>(PWR_MGMT_1_BIT::DEVICE_RESET));
			_register_bank_valid = false;

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
			// The reset value is 0x00 for all registers other than the registers below
			if ((registerRead(Register::BANK_0::WHO_AM_I) == _profile.whoami)
			    && (registerRead(Register::BANK_0::PWR_MGMT_1) == 0x41)
			    && !_transfer_failed) {

				// Wakeup and reset
				registerWrite(Register::BANK_0::PWR_MGMT_1, static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0));

				const uint8_t user_ctrl = static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS)
							  | static_cast<uint8_t>(USER_CTRL_BIT::SRAM_RST);

				registerWrite(Register::BANK_0::USER_CTRL, user_ctrl);

				if (_transfer_failed) {
					reset(100_ms);
					break;
				}

				// if reset succeeded then configure
				_state = State::kConfigure;
				ScheduleDelayed(100_ms);

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("Reset failed, retrying");
					_state = State::kReset;
					ScheduleDelayed(100_ms);

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}

			break;
		}

	case State::kConfigure: {
			if (configure()) {
				if (!fifoReset()) {
					reset(100_ms);
					break;
				}

				// if configure succeeded then start reading from FIFO
				_state                 = State::kFifoRead;
				_last_sample_timestamp = now;

				// Both original ICM20x paths poll; keep that behavior without dead IRQ state.
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

			} else {
				// CONFIGURE not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("Configure failed, resetting");
					_state = State::kReset;

				} else {
					PX4_DEBUG("Configure failed, retrying");
				}

				ScheduleDelayed(100_ms);
			}

			break;
		}

	case State::kFifoRead: {
			break; // Handled by the acquisition fast path above.
		}
	}
}

void ICM20x::configureAccel()
{
	_px4_accel.set_scale(CONSTANTS_ONE_G / _profile.accel_lsb_per_g);
	_px4_accel.set_range(_profile.accel_range_g * CONSTANTS_ONE_G);
}

void ICM20x::configureGyro()
{
	_px4_gyro.set_scale(math::radians(_profile.gyro_range_dps / 32768.f));
	_px4_gyro.set_range(math::radians(_profile.gyro_range_dps));
}

void ICM20x::configureSampleRate(int sample_rate)
{
	// Round to the nearest complete group of repeated accel samples.
	const float min_interval       = _fifo_sample_dt_us * kSamplesPerTransfer;
	const float requested_interval = math::max(
			roundf((1e6f / static_cast<float>(math::max(sample_rate, 1))) / min_interval) * min_interval,
			min_interval);

	_fifo_gyro_samples = static_cast<int32_t>(
				     math::constrain(
					     roundf(requested_interval / _fifo_sample_dt_us),
					     static_cast<float>(kSamplesPerTransfer),
					     static_cast<float>(kFifoMaxSamples)));

	// Recompute the interval in microseconds using the actual gyro sample limit.
	_fifo_empty_interval_us = static_cast<uint32_t>(roundf(_fifo_gyro_samples * _fifo_sample_dt_us));
}

void ICM20x::selectRegisterBank(RegisterBank bank, bool force)
{
	set_frequency(_register_frequency);

	if (!_register_bank_valid
	    || bank != _last_register_bank
	    || force) {
		// Update the cached bank only after a successful selection transaction.
		uint8_t *cmd_bank_sel = _dma_buffer.data();
		cmd_bank_sel[0] = static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL);
		cmd_bank_sel[1] = static_cast<uint8_t>(bank);

		if (transfer(cmd_bank_sel, cmd_bank_sel, 2) != PX4_OK) {
			_register_bank_valid = false;
			_transfer_failed     = true;
			_transfer_perf.bad_transfer.count();

			return;
		}

		_register_bank_valid = true;
		_last_register_bank  = bank;
	}
}

bool ICM20x::configure()
{
	// first set and clear all configured register bits
	for (const auto &register_config : _register_bank0_config) {
		registerSetAndClearBits(register_config.reg, register_config.set_bits, register_config.clear_bits);
	}

	for (const auto &register_config : _register_bank2_config) {
		registerSetAndClearBits(register_config.reg, register_config.set_bits, register_config.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &register_config : _register_bank0_config) {
		if (!registerCheck(register_config)) {
			success = false;
		}
	}

	for (const auto &register_config : _register_bank2_config) {
		if (!registerCheck(register_config)) {
			success = false;
		}
	}

	const uint8_t pll1 = registerRead(Bank1::TIMEBASE_CORRECTION_PLL);
	const uint8_t pll2 = registerRead(Bank1::TIMEBASE_CORRECTION_PLL);

	if (_transfer_failed || pll1 != pll2) {
		return false;
	}

	const float pll_scale = 1.f + 0.00079f * static_cast<float>(static_cast<int8_t>(pll1));
	_fifo_sample_dt_us    = (pll_scale > 0.5f && pll_scale < 1.5f) ? (kFifoSampleDt / pll_scale) : kFifoSampleDt;
	configureSampleRate(_px4_gyro.get_max_rate_hz());

	selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	configureAccel();
	configureGyro();

	return success && !_transfer_failed;
}

template <typename T>
bool ICM20x::registerCheck(const T &register_config)
{
	for (unsigned attempt = 0; attempt < 2; ++attempt) {
		const uint8_t reg_value = registerRead(register_config.reg, attempt > 0);

		if (_transfer_failed) {
			return false;
		}

		const bool set_ok   = !register_config.set_bits
				      || ((reg_value & register_config.set_bits) == register_config.set_bits);
		const bool clear_ok = !register_config.clear_bits
				      || ((reg_value & register_config.clear_bits) == 0);

		if (set_ok && clear_ok) {
			return true;
		}

		if (attempt > 0) {
			if (!set_ok) {
				PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)",
					  static_cast<uint8_t>(register_config.reg),
					  reg_value,
					  register_config.set_bits);
			}

			if (!clear_ok) {
				PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)",
					  static_cast<uint8_t>(register_config.reg),
					  reg_value,
					  register_config.clear_bits);
			}
		}
	}

	return false;
}

template <typename T>
uint8_t ICM20x::registerRead(T reg, bool force_bank)
{
	if (_transfer_failed) {
		return 0;
	}

	selectRegisterBank(reg, force_bank);

	if (_transfer_failed || !_register_bank_valid) {
		return 0;
	}

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(static_cast<uint8_t>(reg) | DIR_READ);
	cmd[1] = 0;

	if (transfer(cmd, cmd, 2) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}

	return cmd[1];
}

template <typename T>
void ICM20x::registerWrite(T reg, uint8_t value)
{
	if (_transfer_failed) {
		return;
	}

	selectRegisterBank(reg);

	if (_transfer_failed || !_register_bank_valid) {
		return;
	}

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = value;

	if (transfer(cmd, cmd, 2) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}
}

template <typename T>
void ICM20x::registerSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = registerRead(reg);
	const uint8_t val      = static_cast<uint8_t>((orig_val & ~clearbits) | setbits);

	if (!_transfer_failed && orig_val != val) {
		registerWrite(reg, val);
	}
}

uint16_t ICM20x::fifoReadCount()
{
	if (_transfer_failed) {
		return 0;
	}

	selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	if (_transfer_failed || !_register_bank_valid) {
		return 0;
	}

	uint8_t *fifo_count_buf = _dma_buffer.data();
	fifo_count_buf[0] = static_cast<uint8_t>(static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH) | DIR_READ);
	fifo_count_buf[1] = 0;
	fifo_count_buf[2] = 0;

	set_frequency(_data_frequency);

	if (transfer(fifo_count_buf, fifo_count_buf, 3) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return 0;
	}

	return static_cast<uint16_t>((static_cast<uint16_t>(fifo_count_buf[1]) << 8) | fifo_count_buf[2]);
}

bool ICM20x::fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	const size_t transfer_size = 3 + static_cast<size_t>(samples) * sizeof(FIFO::DATA);

	if (_transfer_failed
	    || samples == 0
	    || samples > kFifoMaxSamples
	    || transfer_size > _dma_buffer.capacity()
	    || transfer_size > _profile.device.max_transfer_bytes) {
		return false;
	}

	selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	if (_transfer_failed || !_register_bank_valid) {
		return false;
	}

	uint8_t *buffer = _dma_buffer.data();
	memset(buffer, 0, transfer_size);
	buffer[0] = static_cast<uint8_t>(Bank0::FIFO_COUNTH) | DIR_READ;

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, transfer_size) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const uint16_t fifo_count_bytes = static_cast<uint16_t>((static_cast<uint16_t>(buffer[1]) << 8) | buffer[2]);

	if (fifo_count_bytes >= FIFO::SIZE) {
		_fifo_perf.overflow.count();

		if (!fifoReset()) {
			reset(100_ms);
		}

		return false;
	}

	const uint16_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	if (fifo_count_samples == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	// The transfer includes a fresh count snapshot; never decode beyond either snapshot or the requested payload.
	const uint8_t valid_samples = static_cast<uint8_t>(math::min(static_cast<uint16_t>(samples), fifo_count_samples));
	const auto   *fifo          = reinterpret_cast<const FIFO::DATA *>(buffer + 3);

	sensor_gyro_fifo_s  gyro  {};
	sensor_accel_fifo_s accel {};

	if (!decodeGyro(timestamp_sample, fifo, valid_samples, gyro)) {
		return false;
	}

	if (!decodeAccel(timestamp_sample, fifo, valid_samples, accel)) {
		if (!fifoReset()) {
			reset(100_ms);
		}

		return false;
	}

	const uint64_t errors = errorCount();

	_px4_gyro.set_error_count(errors);
	_px4_gyro.updateFIFO(gyro);

	_px4_accel.set_error_count(errors);

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}

	return true;
}

bool ICM20x::fifoReset()
{
	_fifo_perf.reset.count();

	// Stop sensor writes before resetting the FIFO pointers. USER_CTRL.FIFO_EN only
	// gates serial access; FIFO_EN_2 gates the sensor DMA writers (DS-000192, bank 0).
	// Resetting a live writer can leave the next burst starting partway through a frame.
	registerWrite(Register::BANK_0::FIFO_EN_2, 0);

	const uint8_t disabled = registerRead(Register::BANK_0::FIFO_EN_2);

	if (_transfer_failed) {
		return false;
	}

	if (disabled != 0) {
		_transfer_perf.bad_register.count();

		return false;
	}

	registerSetBits(Register::BANK_0::FIFO_RST, static_cast<uint8_t>(FIFO_RST_BIT::FIFO_RESET));
	registerClearBits(Register::BANK_0::FIFO_RST, static_cast<uint8_t>(FIFO_RST_BIT::FIFO_RESET));

	// Confirm deassertion before accepting any records from the new FIFO epoch.
	const uint8_t value = registerRead(Register::BANK_0::FIFO_RST);

	if (_transfer_failed) {
		return false;
	}

	if (value & static_cast<uint8_t>(FIFO_RST_BIT::FIFO_RESET)) {
		_transfer_perf.bad_register.count();

		return false;
	}

	registerWrite(Register::BANK_0::FIFO_EN_2, kFifoEn2Configured);

	const uint8_t restored = registerRead(Register::BANK_0::FIFO_EN_2);

	if (_transfer_failed) {
		return false;
	}

	if (restored != kFifoEn2Configured) {
		_transfer_perf.bad_register.count();

		return false;
	}

	return true;
}

bool ICM20x::decodeAccel(
	const hrt_abstime &timestamp_sample,
	const FIFO::DATA fifo[],
	uint8_t samples,
	sensor_accel_fifo_s &accel)
{
	accel.timestamp_sample = timestamp_sample;
	accel.dt               = _fifo_sample_dt_us * kSamplesPerTransfer;

	uint8_t first_sample = 1;

	// Accel values repeat across gyro packets. Determine the phase before selecting distinct accel samples.
	if (!imu::repeatedAccelPhase(reinterpret_cast<const uint8_t *>(fifo),
				     static_cast<size_t>(samples) * sizeof(FIFO::DATA),
				     sizeof(FIFO::DATA),
				     first_sample)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const auto decode = [](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);

		return imu::FifoSampleResult::kAppend;
	};

	if (!imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(reinterpret_cast<const uint8_t *>(fifo),
			static_cast<size_t>(samples) * sizeof(FIFO::DATA),
			sizeof(FIFO::DATA),
			first_sample,
			kSamplesPerTransfer,
			accel,
			decode)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}

bool ICM20x::decodeGyro(
	const hrt_abstime &timestamp_sample,
	const FIFO::DATA fifo[],
	uint8_t samples,
	sensor_gyro_fifo_s &gyro)
{
	gyro.timestamp_sample = timestamp_sample;
	gyro.dt               = _fifo_sample_dt_us;

	const auto decode = [](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet + offsetof(FIFO::DATA, GYRO_XOUT_H), axes);

		return imu::FifoSampleResult::kAppend;
	};

	if (!imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(reinterpret_cast<const uint8_t *>(fifo),
			static_cast<size_t>(samples) * sizeof(FIFO::DATA),
			sizeof(FIFO::DATA),
			0,
			1,
			gyro,
			decode)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}

void ICM20x::updateTemperature()
{
	if (_transfer_failed) {
		return;
	}

	selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	if (_transfer_failed || !_register_bank_valid) {
		return;
	}

	uint8_t *temperature_buf = _dma_buffer.data();
	temperature_buf[0] = static_cast<uint8_t>(static_cast<uint8_t>(Register::BANK_0::TEMP_OUT_H) | DIR_READ);
	temperature_buf[1] = 0;
	temperature_buf[2] = 0;

	set_frequency(_data_frequency);

	if (transfer(temperature_buf, temperature_buf, 3) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return;
	}

	const int16_t temperature_raw = imu::readBe16(&temperature_buf[1]);
	const float   temperature     = (temperature_raw / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

uint64_t ICM20x::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}
