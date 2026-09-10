/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "TdkIcm20x.hpp"
#include "../../common/ByteCursor.hpp"
#include "../TdkSamplePhase.hpp"
#include "../../common/FifoBatch.hpp"

#include <drivers/drv_sensor.h>

#include <climits>
#include <cstring>

using namespace time_literals;
using namespace tdk_icm20x_registers;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

TdkIcm20x::TdkIcm20x(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_profile(*static_cast<const Profile *>(config.custom_data)),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
}

TdkIcm20x::~TdkIcm20x() = default;

int TdkIcm20x::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? 0 : -1;
}

bool TdkIcm20x::reset(uint32_t delay_us)
{
	_state = State::kReset;

	ScheduleClear();

	if (delay_us > 0) {
		ScheduleDelayed(delay_us);

	} else {
		ScheduleNow();
	}

	return true;
}

void TdkIcm20x::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void TdkIcm20x::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("type: %s (six-axis only, polling)", _profile.device.name);

	imu::printSpiStatus(_profile.device,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	_transfer_perf.print();
	_fifo_perf.print();
}

int TdkIcm20x::probe()
{
	for (int i = 0; i < 3; i++) {
		_transfer_failed = false;

		// Read identity from bank 0, independently of the cached bank or the state left by a reset.
		selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0, true);

		uint8_t whoami = registerRead(Register::BANK_0::WHO_AM_I);

		if (!_transfer_failed && whoami == _profile.whoami) {
			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
	}

	return PX4_ERROR;
}

void TdkIcm20x::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;

		// always check current FIFO count
		bool           success    = false;
		const uint16_t fifo_count = fifoReadCount();

		if (fifo_count >= FIFO::SIZE) {
			_fifo_perf.overflow.count();

			if (!fifoReset()) {
				reset(100_ms);
				return;
			}

		} else if (fifo_count == 0) {
			_fifo_perf.empty.count();

		} else {
			// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure
			uint16_t samples = fifo_count / sizeof(FIFO::DATA);

			if (samples > _fifo_gyro_samples) {
				// grab desired number of samples, but reschedule next cycle sooner
				int extra_samples = samples - _fifo_gyro_samples;

				samples = _fifo_gyro_samples;

				if (_fifo_gyro_samples > extra_samples) {
					// reschedule to run when a total of _fifo_gyro_samples should be available in the FIFO
					const uint32_t reschedule_delay_us = (_fifo_gyro_samples - extra_samples) * static_cast<int>(kFifoSampleDt);
					ScheduleOnInterval(_fifo_empty_interval_us, reschedule_delay_us);

				} else {
					// otherwise reschedule to run immediately
					ScheduleOnInterval(_fifo_empty_interval_us);
				}

			} else if (samples < _fifo_gyro_samples) {
				// reschedule next cycle to catch the desired number of samples
				ScheduleOnInterval(_fifo_empty_interval_us, (_fifo_gyro_samples - samples) * static_cast<int>(kFifoSampleDt));
			}

			if (samples == _fifo_gyro_samples) {
				if (fifoRead(timestamp_sample, samples)) {
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}
			}
		}

		if (!success) {
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

		if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
			// check configuration registers periodically or immediately following any failure

			if (registerCheck(_register_bank0_cfg[_checked_register_bank0])
			    && registerCheck(_register_bank2_cfg[_checked_register_bank2])) {
				_last_config_check_timestamp = now;
				_checked_register_bank0      = (_checked_register_bank0 + 1) % kRegisterBank0ConfigCount;
				_checked_register_bank2      = (_checked_register_bank2 + 1) % kRegisterBank2ConfigCount;

			} else {
				// register check failed, force reset
				_transfer_perf.bad_register.count();
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
			_register_bank_valid = false;
			// PWR_MGMT_1: Device Reset
			registerWrite(Register::BANK_0::PWR_MGMT_1, static_cast<uint8_t>(PWR_MGMT_1_BIT::DEVICE_RESET));
			_register_bank_valid = false;

			if (_transfer_failed) {
				ScheduleDelayed(100_ms);
				break;
			}

			_reset_timestamp     = now;
			_failure_count       = 0;
			_state               = State::kWaitForReset;
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

				uint8_t user_ctrl = static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS) | static_cast<uint8_t>(USER_CTRL_BIT::SRAM_RST);

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
				_state = State::kFifoRead;

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

void TdkIcm20x::configureAccel()
{
	_px4_accel.set_scale(CONSTANTS_ONE_G / _profile.accel_lsb_per_g);
	_px4_accel.set_range(_profile.accel_range_g * CONSTANTS_ONE_G);
}

void TdkIcm20x::configureGyro()
{
	_px4_gyro.set_scale(math::radians(_profile.gyro_range_dps / 32768.f));
	_px4_gyro.set_range(math::radians(_profile.gyro_range_dps));
}

void TdkIcm20x::configureSampleRate(int sample_rate)
{
	// Round to the nearest complete group of repeated accel samples.
	const float min_interval = kFifoSampleDt * kSamplesPerTransfer;

	_fifo_empty_interval_us =
		math::max(
			roundf((1e6f / static_cast<float>(sample_rate)) / min_interval) * min_interval,
			min_interval);

	_fifo_gyro_samples =
		roundf(
			math::min(
				static_cast<float>(_fifo_empty_interval_us) / (1e6f / kGyroRate),
				static_cast<float>(kFifoMaxSamples)));

	// Recompute the interval in microseconds using the actual gyro sample limit.
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / kGyroRate);
}

void TdkIcm20x::selectRegisterBank(RegisterBank bank, bool force)
{
	set_frequency(_register_frequency);

	if (!_register_bank_valid
	    || bank != _last_register_bank
	    || force) {
		// Update the cached bank only after a successful selection transaction.
		uint8_t cmd_bank_sel[2] {};

		cmd_bank_sel[0] = static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL);
		cmd_bank_sel[1] = static_cast<uint8_t>(bank);

		if (transfer(cmd_bank_sel, cmd_bank_sel, sizeof(cmd_bank_sel)) != PX4_OK) {
			_register_bank_valid = false;
			_transfer_failed     = true;
			_transfer_perf.bad_transfer.count();

			return;
		}

		_register_bank_valid = true;

		_last_register_bank = bank;
	}
}

bool TdkIcm20x::configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		registerSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		registerSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!registerCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		if (!registerCheck(reg_cfg)) {
			success = false;
		}
	}

	configureAccel();
	configureGyro();

	return success && !_transfer_failed;
}

template <typename T>
bool TdkIcm20x::registerCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = registerRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)",
			  static_cast<uint8_t>(reg_cfg.reg),
			  reg_value,
			  reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)",
			  static_cast<uint8_t>(reg_cfg.reg),
			  reg_value,
			  reg_cfg.clear_bits);
		success = false;
	}

	return success && !_transfer_failed;
}

template <typename T>
uint8_t TdkIcm20x::registerRead(T reg)
{
	if (_transfer_failed) {
		return 0;
	}

	uint8_t cmd[2] {};

	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	selectRegisterBank(reg);

	if (_transfer_failed) {
		return 0;
	}

	if (transfer(cmd, cmd, sizeof(cmd)) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}

	return cmd[1];
}

template <typename T>
void TdkIcm20x::registerWrite(T reg, uint8_t value)
{
	if (_transfer_failed) {
		return;
	}

	uint8_t cmd[2] { static_cast<uint8_t>(reg), value };

	selectRegisterBank(reg);

	if (_transfer_failed) {
		return;
	}

	if (transfer(cmd, cmd, sizeof(cmd)) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}
}

template <typename T>
void TdkIcm20x::registerSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = registerRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (!_transfer_failed && orig_val != val) {
		registerWrite(reg, val);
	}
}

uint16_t TdkIcm20x::fifoReadCount()
{
	selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	// read FIFO count
	uint8_t fifo_count_buf[3] {};

	fifo_count_buf[0] = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH) | DIR_READ;

	set_frequency(_data_frequency);

	if (!_register_bank_valid || transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool TdkIcm20x::fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	FifoTransferBuffer buffer {};
	const size_t transfer_size = samples * sizeof(FIFO::DATA) + 3;

	if (samples == 0
	    || samples > kFifoMaxSamples
	    || transfer_size > _profile.device.max_transfer_bytes) {
		return false;
	}

	set_frequency(_data_frequency);

	if (!_register_bank_valid || transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const uint16_t fifo_count_bytes = combine(buffer.FIFO_COUNTH, buffer.FIFO_COUNTL);

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
	const uint16_t valid_samples = math::min(static_cast<uint16_t>(samples), fifo_count_samples);

	if (valid_samples > 0) {
		if (processGyro(timestamp_sample, buffer.f, valid_samples)) {
			// Accel phase failure must not suppress the independent, valid gyro batch.
			return processAccel(timestamp_sample, buffer.f, valid_samples);
		}
	}

	return false;
}

bool TdkIcm20x::fifoReset()
{
	_fifo_perf.reset.count();

	// Pulse the FIFO reset bits without changing the configured channel selection.
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

	return true;
}

bool TdkIcm20x::processAccel(
	const hrt_abstime &timestamp_sample,
	const FIFO::DATA fifo[],
	const uint8_t samples)
{
	sensor_accel_fifo_s accel {};

	accel.timestamp_sample = timestamp_sample;
	accel.dt               = kFifoSampleDt * kSamplesPerTransfer;

	uint8_t first_sample = 1;

	// Accel values repeat across gyro packets. Determine the phase before selecting distinct accel samples.
	if (!tdk::repeatedAccelPhase(reinterpret_cast<const uint8_t *>(fifo), samples * sizeof(FIFO::DATA),
				     sizeof(FIFO::DATA), first_sample)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const auto decode = [](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);

		return imu::FifoSampleResult::kAppend;
	};

	if (!imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(reinterpret_cast<const uint8_t *>(fifo),
			samples * sizeof(FIFO::DATA), sizeof(FIFO::DATA), first_sample, kSamplesPerTransfer, accel, decode)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_accel.set_error_count(errorCount());

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}

	return true;
}

bool TdkIcm20x::processGyro(
	const hrt_abstime &timestamp_sample,
	const FIFO::DATA fifo[],
	const uint8_t samples)
{
	sensor_gyro_fifo_s gyro {};

	gyro.timestamp_sample = timestamp_sample;
	gyro.dt               = kFifoSampleDt;

	const auto decode = [](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet + offsetof(FIFO::DATA, GYRO_XOUT_H), axes);

		return imu::FifoSampleResult::kAppend;
	};

	if (!imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(reinterpret_cast<const uint8_t *>(fifo),
			samples * sizeof(FIFO::DATA), sizeof(FIFO::DATA), 0, 1, gyro, decode)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_gyro.set_error_count(errorCount());
	_px4_gyro.updateFIFO(gyro);

	return true;
}

void TdkIcm20x::updateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};

	temperature_buf[0] = static_cast<uint8_t>(Register::BANK_0::TEMP_OUT_H) | DIR_READ;
	selectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	set_frequency(_data_frequency);

	if (!_register_bank_valid || transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return;
	}

	const int16_t temperature_raw = combine(temperature_buf[1], temperature_buf[2]);
	const float   temperature     = (temperature_raw / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

uint64_t TdkIcm20x::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}
