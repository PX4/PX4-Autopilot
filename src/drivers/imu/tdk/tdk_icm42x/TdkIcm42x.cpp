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

#include "TdkIcm42x.hpp"
#include "TdkIcm42xRegisters.hpp"
#include "TdkHiresFifo.hpp"
#include "registers/TdkICM42670PRegisters.hpp"
#include "registers/TdkICM45686Registers.hpp"

#include <drivers/drv_sensor.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include "../../common/FifoFrames.hpp"

#include <climits>
#include <cmath>
#include <cstring>

using namespace time_literals;
using namespace frequency_literals;

namespace
{

constexpr uint8_t  kDirRead              { 0x80 };
constexpr uint8_t  kPwrLowNoise          { 0x0f };
constexpr uint8_t  kBankSelectRegister   { 0x76 };
constexpr uint8_t  kMclkReadyRegister    { 0x00 };
constexpr uint8_t  kMclkReadyBit         { 1u << 3 };
constexpr uint8_t  kMregWriteBlock       { 0x79 };
constexpr uint8_t  kMregWriteAddress     { 0x7a };
constexpr uint8_t  kMregWriteData        { 0x7b };
constexpr uint8_t  kMregReadBlock        { 0x7c };
constexpr uint8_t  kMregReadAddress      { 0x7d };
constexpr uint8_t  kMregReadData         { 0x7e };
constexpr uint8_t  kIregAddressHigh      { 0x7c };
constexpr uint8_t  kIregData             { 0x7e };
constexpr unsigned kMregDelayUs          { 10 };
constexpr unsigned kIregDelayUs          { 4 };
constexpr float    kFifoTimestampScaling { 16.f *(32.f / 30.f) };

constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

constexpr uint16_t combineUnsigned(uint8_t msb, uint8_t lsb)
{
	return (static_cast<uint16_t>(msb) << 8) | lsb;
}

} // namespace

TdkIcm42x::TdkIcm42x(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_profile(*static_cast<const Profile *>(config.custom_data)),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_enable_clock_input(config.custom1 != 0 && _profile.clock_input),
	// All implemented CLKIN paths specify nominal ODR at 32 kHz, not 32.768 kHz.
	_sample_dt_us((1e6f / _profile.output_data_rate_hz)
		      * (_enable_clock_input ? 32_kHz / static_cast<float>(config.custom1) : 1.f)),
	_timestamp_scale_us(_enable_clock_input ? 1e6f / config.custom1 : kFifoTimestampScaling),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
	_px4_accel.set_range(_profile.accel_range_g * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(_profile.gyro_range_dps));

	_px4_accel.set_scale(CONSTANTS_ONE_G * _profile.accel_range_g / 32768.f);
	_px4_gyro.set_scale(math::radians(_profile.gyro_range_dps / 32768.f));
}

TdkIcm42x::~TdkIcm42x() = default;

int TdkIcm42x::init()
{
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

int TdkIcm42x::probe()
{
	for (int attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;

		if (_profile.protocol == Protocol::kBanked) {
			selectRegisterBank(0, true);
		}

		const uint8_t whoami = registerRead(AddressSpace::kBank0, _profile.whoami_reg);

		if (!_transfer_failed && whoami == _profile.whoami) {
			return PX4_OK;
		}

		DEVICE_DEBUG("%s unexpected WHO_AM_I 0x%02x", _profile.device.name, whoami);
	}

	return PX4_ERROR;
}

bool TdkIcm42x::reset()
{
	_state               = State::kReset;
	_fifo_state_ready_at = 0;
	dataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();

	return true;
}

void TdkIcm42x::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void TdkIcm42x::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("variant: %s", _profile.device.name);

	imu::printSpiStatus(_profile.device,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO interval: %u us (%.1f Hz), %u samples",
		 _fifo_empty_interval_us,
		 1e6 / _fifo_empty_interval_us,
		 _fifo_gyro_samples);
	PX4_INFO("clock input: %s", _enable_clock_input ? "enabled" : "disabled");

	_transfer_perf.print();
	_fifo_perf.print();
	_drdy_missed_perf.print();
}

void TdkIcm42x::configureSampleRate(int sample_rate)
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
			static_cast<unsigned>(_profile.fifo_capacity / _profile.packet_size));

	_fifo_gyro_samples =
		static_cast<uint8_t>(
			math::constrain(
				roundf(_fifo_empty_interval_us / sample_dt),
				1.f,
				static_cast<float>(fifo_limit)));
	_fifo_empty_interval_us = static_cast<uint16_t>(roundf(_fifo_gyro_samples * sample_dt));

	// The configured watermark is expressed in records or bytes, depending on the model.
	const tdk_icm42x_config::Context context {
		static_cast<uint16_t>(_fifo_gyro_samples * (_profile.fifo_count_is_records ? 1 : _profile.packet_size)),
		_enable_clock_input,
	};

	_register_cfg_count = tdk_icm42x_config::load(_profile.variant, context, _register_cfg);

	if (_register_cfg_count > tdk_icm42x_config::kMaxRegisterConfigs) {
		_register_cfg_count = 0;
	}
}

bool TdkIcm42x::resetComplete()
{
	if (_profile.protocol == Protocol::kBanked) {
		selectRegisterBank(0, true);
	}

	if (registerRead(AddressSpace::kBank0, _profile.whoami_reg) != _profile.whoami) {
		return false;
	}

	if ((registerRead(AddressSpace::kBank0, _profile.reset_reg) & _profile.reset_bit) != 0) {
		return false;
	}

	return (_profile.reset_status_bit == 0
		|| (registerRead(AddressSpace::kBank0, _profile.reset_status_reg) & _profile.reset_status_bit)) && !_transfer_failed;
}

void TdkIcm42x::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;
		uint8_t     samples          = 0;
		hrt_abstime fresh_interrupt  = 0;

		const bool count_before_irq = _profile.variant == Variant::kIcm40609D;

		// ICM40609D verifies available records before using an interrupt's target count.
		const uint16_t initial_count = count_before_irq ? fifoReadCount() : 0;

		if (_data_ready_interrupt_enabled) {
			const hrt_abstime drdy_timestamp = _drdy_timestamp_sample.fetch_and(0);

			if (drdy_timestamp != 0 && (now - drdy_timestamp) < _fifo_empty_interval_us) {
				fresh_interrupt = drdy_timestamp;

				if (!count_before_irq) {
					timestamp_sample = drdy_timestamp;
					samples          = _fifo_gyro_samples;
				}

			} else {
				_drdy_missed_perf.count();
			}

			ScheduleDelayed(_fifo_empty_interval_us * 2);
		}

		if (samples == 0 || _profile.protocol == Protocol::kDirect456) {
			samples = 0;

			const uint16_t fifo_count = count_before_irq ? initial_count : fifoReadCount();
			const uint16_t fifo_limit = _profile.fifo_count_is_records
						    ? _profile.fifo_capacity / _profile.packet_size : _profile.fifo_capacity;

			if (fifo_count >= fifo_limit) {
				fifoReset();
				_fifo_perf.overflow.count();

			} else if (fifo_count == 0) {
				_fifo_perf.empty.count();

			} else {
				uint16_t count = _profile.fifo_count_is_records ? fifo_count : fifo_count / _profile.packet_size;

				// Apply the original one-extra-frame tolerance before the batch-capacity check.
				if (_profile.protocol != Protocol::kDirect456 && count == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(_sample_dt_us);
					--count;
				}

				if (count > kFifoMaxSamples) {
					fifoReset();
					_fifo_perf.overflow.count();

				} else {
					samples = static_cast<uint8_t>(count);
				}
			}
		}

		if (count_before_irq && samples > 0 && fresh_interrupt != 0) {
			timestamp_sample = fresh_interrupt;
			samples          = _fifo_gyro_samples;
		}

		bool success = samples > 0 && fifoRead(timestamp_sample, samples);

		if (success) {
			_failure_count = math::max(0, _failure_count - 1);

		} else if (++_failure_count > 10) {
			reset();

			return;
		}

		if (_register_cfg_count > 0
		    && (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms)) {
			if (checkConfiguration()) {
				_last_config_check_timestamp = now;

			} else {
				_transfer_perf.bad_register.count();
				reset();
			}

		} else if (!_profile.fifo_temperature && hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
			updateTemperature();
			_temperature_update_timestamp = now;
		}

		return;
	}

	switch (_state) {
	case State::kReset: {
			_register_bank_valid = false;
			memset(_checked_register, 0, sizeof(_checked_register));
			registerWrite(AddressSpace::kBank0, _profile.reset_reg, _profile.reset_bit);
			_register_bank_valid = false;

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
				// Configure with both measurement channels OFF. ICM42670P needs
				// IDLE = 1 to keep MCLK running for indirect MREG access.
				if (_profile.protocol == Protocol::kMreg) {
					constexpr uint8_t kIdle { 1u << 4 };

					registerWrite(AddressSpace::kBank0, _profile.power_reg, kIdle);
				}

				if (_transfer_failed) {
					_state = State::kReset;
					ScheduleDelayed(100_ms);
					break;
				}

				_state = State::kConfigure;
				ScheduleDelayed(1_ms);

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
			if (configure()) {
				registerWrite(AddressSpace::kBank0, _profile.power_reg, kPwrLowNoise);

				if (!_transfer_failed) {
					// No register writes for at least 200 us after OFF -> ON. This
					// host guard also exceeds the 45 ms minimum gyro ON time and
					// allows margin over ICM45686's typical 35 ms startup time.
					// Packet validity checks remain required; this is not a ready bit.
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

				if (deferredConfiguration(cfg) && cfg.reg != _profile.power_reg) {
					registerSetAndClearBits(cfg);
				}
			}

			if (_transfer_failed) {
				_state = State::kReset;
				ScheduleDelayed(100_ms);

			} else if ((_profile.protocol == Protocol::kBanked && _profile.packet_format == PacketFormat::kHighRes20)
				   || _profile.protocol == Protocol::kDirect456
				   || _profile.protocol == Protocol::kMreg) {
				// Preserve the original high-resolution banked and ICM45686 startup guard.
				// ICM42670P uses the same host guard; its original path did not wait here.
				// Let initial ODR-change records enter the FIFO before the startup flush.
				_state               = State::kFifoReset;
				_fifo_state_ready_at = hrt_absolute_time() + 1_ms;
				ScheduleDelayed(1_ms);

			} else {
				startFifoRead();
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
			break; // Handled by the acquisition fast path above.
		}
	}
}

void TdkIcm42x::startFifoRead()
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
	_data_ready_interrupt_enabled = _profile.data_ready_interrupt && dataReadyInterruptConfigure();

	if (_data_ready_interrupt_enabled) {
		ScheduleDelayed(100_ms);

	} else {
		ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
	}
}

bool TdkIcm42x::checkConfiguration()
{
	constexpr AddressSpace spaces[] {
		AddressSpace::kBank0,
		AddressSpace::kBank1,
		AddressSpace::kBank2,
		AddressSpace::kMreg1,
		AddressSpace::kIreg,
	};

	static_assert(sizeof(spaces) / sizeof(spaces[0]) == sizeof(_checked_register), "Register check cursor count");

	// One register per address space per pass preserves the original bank/filter surveillance cadence.
	for (unsigned space = 0; space < sizeof(_checked_register); ++space) {
		uint8_t index = _checked_register[space];

		for (uint8_t scanned = 0; scanned < _register_cfg_count; ++scanned) {
			const RegisterConfig &cfg = _register_cfg[index];

			index = index + 1 < _register_cfg_count ? index + 1 : 0;

			if (cfg.space == spaces[space]) {
				if (!registerCheck(cfg)) {
					return false;
				}

				_checked_register[space] = index;
				break;
			}
		}
	}

	return !_transfer_failed;
}

bool TdkIcm42x::deferredConfiguration(const RegisterConfig &config) const
{
	if (config.space != AddressSpace::kBank0) {
		return false;
	}

	if (config.reg == _profile.power_reg) {
		return true;
	}

	if (_profile.protocol == Protocol::kDirect456) {
		using namespace tdk_icm45686_registers;

		return config.reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG0)
		       || config.reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG3);
	}

	const uint16_t mode_register = _profile.protocol == Protocol::kMreg
				       ? static_cast<uint16_t>(tdk_icm42670p_registers::Register::BANK_0::FIFO_CONFIG1)
				       : static_cast<uint16_t>(tdk_icm42x_registers::Register::BANK_0::FIFO_CONFIG);

	return config.reg == mode_register;
}

bool TdkIcm42x::configure()
{
	// Static filters, interface and interrupt settings must not change while
	// measuring. FIFO mode stays at reset bypass until every setting is verified.
	if ((registerRead(AddressSpace::kBank0, _profile.power_reg) & kPwrLowNoise) != 0 || _transfer_failed) {
		return false;
	}

	if (_profile.protocol == Protocol::kDirect456) {
		using namespace tdk_icm45686_registers;

		// FIFO_CONFIG2's watermark comparator may only change in bypass mode.
		const uint8_t mode = registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG0));

		if ((mode & static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_CLEAR)) != 0 || _transfer_failed) {
			return false;
		}
	}

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		if (!deferredConfiguration(_register_cfg[i])) {
			registerSetAndClearBits(_register_cfg[i]);
		}
	}

	// The high-byte write commits the watermark, including when it is zero.
	if (_profile.protocol == Protocol::kDirect456) {
		registerWrite(AddressSpace::kBank0,
			      static_cast<uint16_t>(tdk_icm45686_registers::Register::BANK_0::FIFO_CONFIG1_1), _fifo_gyro_samples >> 8);
	}

	bool success = _register_cfg_count > 0;

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		if (!deferredConfiguration(_register_cfg[i])) {
			success &= registerCheck(_register_cfg[i]);
		}
	}

	return success && !_transfer_failed;
}

void TdkIcm42x::selectRegisterBank(uint8_t bank, bool force)
{
	set_frequency(_register_frequency);

	if (_profile.protocol == Protocol::kBanked
	    && (force || !_register_bank_valid || bank != _last_register_bank)) {
		uint8_t cmd[2] { kBankSelectRegister, bank };

		_register_bank_valid = transferChecked(cmd, sizeof(cmd));

		if (_register_bank_valid) {
			_last_register_bank = bank;
		}
	}
}

uint8_t TdkIcm42x::registerRead(AddressSpace space, uint16_t reg)
{
	set_frequency(_register_frequency);

	if (space == AddressSpace::kIreg) {
		uint8_t cmd[3] { kIregAddressHigh, static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg) };

		if (!transferChecked(cmd, sizeof(cmd))) {
			return 0;
		}

		px4_udelay(kIregDelayUs);

		const uint8_t value = registerRead(AddressSpace::kBank0, kIregData);

		px4_udelay(kIregDelayUs);

		return value;
	}

	if (space == AddressSpace::kMreg1) {
		if ((registerRead(AddressSpace::kBank0, kMclkReadyRegister) & kMclkReadyBit) == 0 || _transfer_failed) {
			_transfer_failed = true;

			return 0;
		}

		registerWrite(AddressSpace::kBank0, kMregReadBlock, 0);
		registerWrite(AddressSpace::kBank0, kMregReadAddress, reg);
		px4_udelay(kMregDelayUs);

		const uint8_t value = registerRead(AddressSpace::kBank0, kMregReadData);

		px4_udelay(kMregDelayUs);

		return value;
	}

	selectRegisterBank(static_cast<uint8_t>(space));

	uint8_t cmd[2] { static_cast<uint8_t>(reg | kDirRead), 0 };

	if (!_transfer_failed) {
		transferChecked(cmd, sizeof(cmd));
	}

	return cmd[1];
}

void TdkIcm42x::registerWrite(AddressSpace space, uint16_t reg, uint8_t value)
{
	set_frequency(_register_frequency);

	if (space == AddressSpace::kIreg) {
		uint8_t cmd[4] { kIregAddressHigh, static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg), value };

		transferChecked(cmd, sizeof(cmd));
		px4_udelay(kIregDelayUs);

		return;
	}

	if (space == AddressSpace::kMreg1) {
		if ((registerRead(AddressSpace::kBank0, kMclkReadyRegister) & kMclkReadyBit) == 0 || _transfer_failed) {
			_transfer_failed = true;

			return;
		}

		registerWrite(AddressSpace::kBank0, kMregWriteBlock, 0);
		registerWrite(AddressSpace::kBank0, kMregWriteAddress, reg);
		registerWrite(AddressSpace::kBank0, kMregWriteData, value);
		px4_udelay(kMregDelayUs);

		return;
	}

	selectRegisterBank(static_cast<uint8_t>(space));

	uint8_t cmd[2] { static_cast<uint8_t>(reg), value };

	if (!_transfer_failed) {
		transferChecked(cmd, sizeof(cmd));
	}
}

void TdkIcm42x::registerSetAndClearBits(const RegisterConfig &reg_cfg)
{
	const uint8_t original = registerRead(reg_cfg.space, reg_cfg.reg);
	const uint8_t value    = (original & ~reg_cfg.clear_bits) | reg_cfg.set_bits;

	if (!_transfer_failed && original != value) {
		registerWrite(reg_cfg.space, reg_cfg.reg, value);
	}
}

bool TdkIcm42x::registerCheck(const RegisterConfig &reg_cfg)
{
	const uint8_t value = registerRead(reg_cfg.space, reg_cfg.reg);
	const bool success = !_transfer_failed && (reg_cfg.set_bits == 0 || (value & reg_cfg.set_bits) == reg_cfg.set_bits)
			     && (reg_cfg.clear_bits == 0 || (value & reg_cfg.clear_bits) == 0);

	if (!success) {
		PX4_DEBUG("%s reg %u:0x%02x value 0x%02x (set 0x%02x clear 0x%02x)",
			  _profile.device.name,
			  static_cast<unsigned>(reg_cfg.space),
			  reg_cfg.reg,
			  value,
			  reg_cfg.set_bits,
			  reg_cfg.clear_bits);
	}

	return success;
}

uint16_t TdkIcm42x::fifoReadCount()
{
	selectRegisterBank(0);
	set_frequency(_data_frequency);

	uint8_t cmd[3] {};
	// Retain the existing ICM45686 workaround for a stale first count read.
	const unsigned reads = _profile.protocol == Protocol::kDirect456 ? 2 : 1;

	for (unsigned i = 0; i < reads; ++i) {
		cmd[0] = _profile.fifo_count_reg | kDirRead;

		if (!transferChecked(cmd, sizeof(cmd))) {
			return 0;
		}
	}

	return _profile.fifo_count_little_endian ? combineUnsigned(cmd[2], cmd[1]) : combineUnsigned(cmd[1], cmd[2]);
}

bool TdkIcm42x::fifoRead(const hrt_abstime &timestamp_sample, uint8_t requested_samples)
{
	selectRegisterBank(0);
	set_frequency(_data_frequency);
	_fifo_transfer[0] = static_cast<uint8_t>((_profile.protocol == Protocol::kDirect456
			    ? _profile.fifo_data_reg : _profile.int_status_reg) | kDirRead);
	const size_t transfer_size = _profile.fifo_transfer_prefix + requested_samples * _profile.packet_size;

	if (requested_samples == 0
	    || requested_samples > kFifoMaxSamples
	    || transfer_size > _profile.device.max_transfer_bytes
	    || !transferChecked(_fifo_transfer, transfer_size)) {
		return false;
	}

	if (_profile.protocol != Protocol::kDirect456 && (_fifo_transfer[1] & _profile.fifo_full_bit)) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	uint8_t available_samples = requested_samples;

	// Banked/MREG transfers include a count snapshot before the payload; ICM45686 reads the payload directly.
	// Compare counts and capacity in the model's native units before converting to complete packets.
	if (_profile.protocol != Protocol::kDirect456) {
		const uint8_t  count_offset   = _profile.fifo_transfer_prefix - 2;
		const uint16_t embedded_count = combineUnsigned(_fifo_transfer[count_offset], _fifo_transfer[count_offset + 1]);
		const uint16_t capacity = _profile.fifo_count_is_records ? _profile.fifo_capacity / _profile.packet_size : _profile.fifo_capacity;

		if (embedded_count >= capacity) {
			_fifo_perf.overflow.count();
			fifoReset();

			return false;
		}

		const uint16_t embedded_samples = _profile.fifo_count_is_records ? embedded_count
						  : embedded_count / _profile.packet_size;
		available_samples = static_cast<uint8_t>(math::min(static_cast<unsigned>(requested_samples),
				    static_cast<unsigned>(embedded_samples)));
	}

	if (available_samples == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	// Keep both batches local until every packet and temperature is validated.
	// This preserves the all-or-nothing batch policy without re-reading axes.
	sensor_accel_fifo_s accel {};
	sensor_gyro_fifo_s  gyro  {};
	static_assert(sizeof(accel.x) == sizeof(gyro.x), "FIFO channel capacities must match");
	accel.timestamp_sample = timestamp_sample;
	gyro.timestamp_sample  = timestamp_sample;

	imu::FifoSampleStats temperatures;

	using Sample = tdk_icm42x_fifo::Sample;
	Sample sample;
	const bool    little           = _profile.packet_format == PacketFormat::kStandard16LittleEndian;
	const bool    timestamp_header = _profile.packet_format == PacketFormat::kHighRes20 || little;
	const uint8_t *data            = &_fifo_transfer[_profile.fifo_transfer_prefix];
	const auto decode = [this, little, timestamp_header, &temperatures](imu::ByteCursor & cursor, Sample & decoded) {
		const uint8_t *packet = cursor.take(_profile.packet_size);

		if (!tdk_icm42x_fifo::decodePacket(
			    packet,
			    _profile.packet_size,
			    little,
			    timestamp_header,
			    _enable_clock_input && !little,
			    decoded)) {
			return imu::FifoFrame::kInvalid;
		}

		// High-resolution profiles retain the upper-16-bit publication convention.
		// decodePacket rejects INT16_MIN before common's validated axis mapping.
		if (_profile.fifo_temperature
		    && decoded.temperature != INT16_MIN
		    && !temperatures.add(decoded.temperature)) {
			return imu::FifoFrame::kInvalid;
		}

		return imu::FifoFrame::kBoth;
	};
	const imu::FifoDecodeResult result = imu::decodeFifoFrames<imu::FifoAxisMapping::kFlipYZValidated>(
			data, available_samples * _profile.packet_size, accel, gyro, sample, decode);

	if (result.status != imu::FifoDecodeStatus::kComplete) {
		_transfer_perf.bad_transfer.count();
		fifoReset();

		return false;
	}

	if (_profile.fifo_temperature && !processTemperature(temperatures)) {
		return false;
	}

	// The previous per-axis loops retained only the last packet's interval.
	// Compute that same value once for both publishers, with no hot-path division.
	// ICM45686 AN-000478 section 7.1: FIFO TMST uses the internal clock even with CLKIN.
	// Its sample interval follows the scaled ODR; it is not TMST divided by the external clock.
	const float dt = _profile.packet_format == PacketFormat::kHighRes20
			 ? sample.timestamp * _timestamp_scale_us
			 : _sample_dt_us;

	accel.dt = dt;
	gyro.dt  = dt;

	const uint64_t errors = errorCount();

	_px4_accel.set_error_count(errors);
	_px4_gyro.set_error_count(errors);

	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);

	return true;
}

bool TdkIcm42x::processTemperature(const imu::FifoSampleStats &temperatures)
{
	if (temperatures.empty()) {
		return false;
	}

	float average;

	// Preserve the legacy transfer-error threshold: at most 1000 raw counts from the mean.
	if (!temperatures.meanWithin(1000.f, average)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const float temperature = average / _profile.temperature_sensitivity + _profile.temperature_offset;

	if (!PX4_ISFINITE(temperature)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	return true;
}

void TdkIcm42x::updateTemperature()
{
	selectRegisterBank(0);

	uint8_t cmd[3] { static_cast<uint8_t>(_profile.temperature_reg | kDirRead), 0, 0 };

	set_frequency(_data_frequency);

	if (!transferChecked(cmd, sizeof(cmd))) {
		return;
	}

	const int16_t raw = _profile.packet_format == PacketFormat::kStandard16LittleEndian ? combine(cmd[2], cmd[1]) : combine(cmd[1], cmd[2]);
	const float temperature = raw / _profile.temperature_sensitivity
				  + _profile.temperature_offset;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

void TdkIcm42x::fifoReset()
{
	_fifo_perf.reset.count();

	if (_profile.protocol == Protocol::kDirect456) {
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

	} else {
		registerSetAndClearBits({AddressSpace::kBank0, _profile.signal_path_reset_reg, _profile.fifo_flush_bit, 0});

		if (_profile.protocol == Protocol::kMreg) {
			px4_udelay(2); // Datasheet: wait at least 1.5 us before checking FIFO_FLUSH.

			if (registerRead(AddressSpace::kBank0, _profile.signal_path_reset_reg) & _profile.fifo_flush_bit) {
				_transfer_failed = true;
			}

		} else if (_profile.variant == Variant::kIcm40609D) {
			registerRead(AddressSpace::kBank0, _profile.int_status_reg);
		}
	}

	_drdy_timestamp_sample.store(0);
}

int TdkIcm42x::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<TdkIcm42x *>(arg)->dataReady();

	return 0;
}

void TdkIcm42x::dataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool TdkIcm42x::dataReadyInterruptConfigure()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;
}

bool TdkIcm42x::dataReadyInterruptDisable()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

uint64_t TdkIcm42x::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}

bool TdkIcm42x::transferChecked(uint8_t *data, size_t size)
{
	if (_transfer_failed || device::SPI::transfer(data, data, size) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}
