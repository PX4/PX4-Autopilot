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

/**
 * @file ICM42x.cpp
 * @brief Implementation of the ICM42x banked 8 kHz TDK SPI IMU family driver.
 */

#include "ICM42x.hpp"

#include "../InvenSense_AAF.hpp"
#include "ICM42xRegisters.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/imu/ByteCursor.hpp>
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

using Bank0 = icm42x_registers::Register::BANK_0;

constexpr uint8_t kDirRead              { 0x80 };
constexpr uint8_t kPwrLowNoise          { 0x0f };
constexpr uint8_t kBankSelectRegister   { static_cast<uint8_t>(Bank0::REG_BANK_SEL) };
constexpr float   kFifoTimestampScaling { 16.f *(32.f / 30.f) };

constexpr uint16_t combineUnsigned(uint8_t msb, uint8_t lsb)
{
	return (static_cast<uint16_t>(msb) << 8) | lsb;
}

} // namespace

ICM42x::ICM42x(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_endpoint_guard(config),
	_profile(*static_cast<const ICM42xOptions *>(config.custom_data)->profile),
	_aaf_hz(static_cast<const ICM42xOptions *>(config.custom_data)->aaf_hz),
	_spi_mode(config.spi_mode),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_enable_clock_input(config.custom1 != 0 && _profile.clock_input),
	// All implemented CLKIN paths specify nominal ODR at 32 kHz, not 32.768 kHz.
	_sample_dt_us((1e6f / icm42x_config::kOutputDataRateHz)
		      * (_enable_clock_input ? (32 * imu::kHz) / static_cast<float>(config.custom1) : 1.f)),
	_timestamp_scale_us(_enable_clock_input ? 1e6f / config.custom1 : kFifoTimestampScaling),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());

	if (_aaf_hz != 0) {
		if (const InvenSense_AAF::Preset *p = InvenSense_AAF::preset(_aaf_hz)) {
			PX4_INFO("anti-alias filter %u Hz (UI_FILT_BW=%u)",
				 p->bandwidth_hz, p->ui_filt_bw);
		}
	}

	_px4_accel.set_range(_profile.accel_range_g * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(_profile.gyro_range_dps));

	_px4_accel.set_scale(CONSTANTS_ONE_G * _profile.accel_range_g / 32768.f);
	_px4_gyro.set_scale(math::radians(_profile.gyro_range_dps / 32768.f));
}

ICM42x::~ICM42x()
{
	dataReadyInterruptDisable();
}

int ICM42x::init()
{
	if (!_endpoint_guard.acquire()) {
		return PX4_ERROR;
	}

	if (!_dma_buffer || _register_config_count == 0) {
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

int ICM42x::probe()
{
	for (int attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;

		selectRegisterBank(0, true);

		const uint8_t whoami = registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Bank0::WHO_AM_I));

		if (!_transfer_failed && whoami == _profile.whoami) {
			return PX4_OK;
		}

		DEVICE_DEBUG("%s unexpected WHO_AM_I 0x%02x", _profile.device.name, whoami);
	}

	return PX4_ERROR;
}

bool ICM42x::reset()
{
	dataReadyInterruptDisable();
	_drdy_timestamp_sample.store(0);
	_config_check_failure_count = 0;
	_state                      = State::kReset;
	_fifo_state_ready_at        = 0;
	ScheduleClear();
	ScheduleNow();

	return true;
}

void ICM42x::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM42x::print_status()
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
	PX4_INFO("clock input: %s", _enable_clock_input ? "enabled" : "disabled");

	if (_aaf_hz != 0) {
		PX4_INFO("anti-alias filter: %u Hz", _aaf_hz);
	}

	_transfer_perf.print();
	_fifo_perf.print();
	_drdy_missed_perf.print();
}

void ICM42x::configureSampleRate(int sample_rate)
{
	const float sample_dt          = _sample_dt_us;
	const float requested_interval = 1e6f / math::max(sample_rate, 1);

	// Bound each read by both the publication buffer and the device FIFO.
	const uint8_t fifo_limit =
		math::min(
			static_cast<unsigned>(kFifoMaxSamples),
			static_cast<unsigned>(icm42x_registers::FIFO::SIZE / _profile.packet_size));

	_fifo_gyro_samples =
		static_cast<uint8_t>(
			math::constrain(
				roundf(requested_interval / sample_dt),
				1.f,
				static_cast<float>(fifo_limit)));
	_fifo_empty_interval_us = static_cast<uint32_t>(roundf(_fifo_gyro_samples * sample_dt));

	// The configured watermark is expressed in records or bytes, depending on the model.
	const icm42x_config::Context context {
		static_cast<uint16_t>(_fifo_gyro_samples * (_profile.fifo_count_is_records ? 1 : _profile.packet_size)),
		_enable_clock_input,
		_aaf_hz,
	};

	_register_config_count = icm42x_config::load(_profile.variant, context, _register_config);

	if (_register_config_count > icm42x_config::kMaxRegisterConfigs) {
		_register_config_count = 0;
	}
}

bool ICM42x::resetComplete()
{
	using namespace icm42x_registers;

	selectRegisterBank(0, true);

	if (registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Bank0::WHO_AM_I)) != _profile.whoami) {
		return false;
	}

	if ((registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Bank0::DEVICE_CONFIG))
	     & static_cast<uint8_t>(DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG)) != 0) {
		return false;
	}

	return (!_profile.check_reset_done
		|| (registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Bank0::INT_STATUS))
		    & static_cast<uint8_t>(INT_STATUS_BIT::RESET_DONE_INT)))
	       && !_transfer_failed;
}

void ICM42x::RunImpl()
{
	const hrt_abstime drdy_timestamp =
		(_state == State::kFifoRead && _data_ready_interrupt_enabled)
		? _drdy_timestamp_sample.fetch_and(0)
		: 0;
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;
		uint8_t     samples          = 0;
		hrt_abstime fresh_interrupt  = 0;

		const bool count_before_irq = _profile.variant == Variant::kIcm40609D;

		if (_data_ready_interrupt_enabled) {
			if (drdy_timestamp != 0 && now >= drdy_timestamp && (now - drdy_timestamp) < _fifo_empty_interval_us) {
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

		if (samples == 0) {
			const uint16_t fifo_count = fifoReadCount();
			const uint16_t fifo_limit = _profile.fifo_count_is_records
						    ? static_cast<uint16_t>(icm42x_registers::FIFO::SIZE / _profile.packet_size)
						    : static_cast<uint16_t>(icm42x_registers::FIFO::SIZE);

			if (fifo_count >= fifo_limit) {
				fifoReset();
				_fifo_perf.overflow.count();

			} else if (fifo_count == 0) {
				_fifo_perf.empty.count();

			} else {
				uint16_t count = _profile.fifo_count_is_records ? fifo_count : fifo_count / _profile.packet_size;

				// Apply the original one-extra-frame tolerance before the batch-capacity check.
				if (count == _fifo_gyro_samples + 1) {
					const hrt_abstime dt_us = static_cast<hrt_abstime>(roundf(_sample_dt_us));
					timestamp_sample = timestamp_sample > dt_us ? timestamp_sample - dt_us : 0;
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
			samples          = math::min<uint8_t>(samples, _fifo_gyro_samples);
		}

		const bool success = samples > 0 && fifoRead(timestamp_sample, samples);

		if (success) {
			_failure_count = math::max(0, _failure_count - 1);

		} else if (++_failure_count > 10) {
			reset();

			return;
		}

		if (_register_config_count > 0
		    && (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms)) {
			// Clear any FIFO-path transfer failure so checkConfiguration() executes live SPI reads.
			_transfer_failed = false;

			if (checkConfiguration()) {
				_config_check_failure_count  = 0;
				_last_config_check_timestamp = now;

			} else if (!_transfer_failed) {
				_transfer_perf.bad_register.count();
				reset();

			} else if (++_config_check_failure_count > 10) {
				reset();
			}

		} else if (_profile.packet_format != PacketFormat::kHighRes20
			   && hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
			updateTemperature();
			_temperature_update_timestamp = now;
		}

		return;
	}

	switch (_state) {
	case State::kReset: {
			_register_bank_valid = false;
			memset(_checked_register, 0, sizeof(_checked_register));
			registerWrite(AddressSpace::kBank0,
				      static_cast<uint16_t>(Bank0::DEVICE_CONFIG),
				      static_cast<uint8_t>(icm42x_registers::DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG));
			_register_bank_valid = false;

			if (_transfer_failed) {
				ScheduleDelayed(100_ms);
				break;
			}

			_reset_timestamp = now;
			_failure_count   = 0;
			_state           = State::kWaitForReset;
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
				registerWrite(AddressSpace::kBank0, static_cast<uint16_t>(Bank0::PWR_MGMT0), kPwrLowNoise);

				if (!_transfer_failed) {
					// Preserve the 50 ms measurement startup guard before flushing startup FIFO records.
					_state               = State::kFifoReset;
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

	case State::kFifoReset: {
			if (now < _fifo_state_ready_at) {
				ScheduleDelayed(static_cast<uint32_t>(_fifo_state_ready_at - now));
				break;
			}

			startFifoRead();
			break;
		}

	case State::kFifoRead: {
			break;
		}
	}
}

void ICM42x::startFifoRead()
{
	bool configured = _register_config_count > 0;

	for (uint8_t i = 0; i < _register_config_count; ++i) {
		configured &= registerCheck(_register_config[i]);
	}

	if (!configured || _transfer_failed) {
		_state = State::kReset;
		ScheduleDelayed(100_ms);

		return;
	}

	fifoReset();

	if (_transfer_failed) {
		_state = State::kReset;
		ScheduleDelayed(100_ms);

		return;
	}

	_state = State::kFifoRead;

	if (dataReadyInterruptConfigure()) {
		ScheduleDelayed(100_ms);

	} else {
		ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
	}
}

bool ICM42x::checkConfiguration()
{
	constexpr AddressSpace spaces[] {
		AddressSpace::kBank0,
		AddressSpace::kBank1,
		AddressSpace::kBank2,
	};

	static_assert(sizeof(spaces) / sizeof(spaces[0]) == sizeof(_checked_register), "Register check cursor count");

	// One register per address space per pass preserves the original bank/filter surveillance cadence.
	for (unsigned space = 0; space < sizeof(_checked_register); ++space) {
		uint8_t index = _checked_register[space];

		for (uint8_t scanned = 0; scanned < _register_config_count; ++scanned) {
			const RegisterConfig &config = _register_config[index];

			index = index + 1 < _register_config_count ? index + 1 : 0;

			if (config.space == spaces[space]) {
				if (!registerCheck(config)) {
					return false;
				}

				_checked_register[space] = index;
				break;
			}
		}
	}

	return !_transfer_failed;
}

bool ICM42x::deferredConfiguration(const RegisterConfig &config) const
{
	return config.space == AddressSpace::kBank0
	       && config.reg == static_cast<uint16_t>(Bank0::PWR_MGMT0);
}

bool ICM42x::configure()
{
	// Static filters, interface and interrupt settings must not change while
	// measuring. FIFO mode stays at reset bypass until every setting is verified.
	if ((registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Bank0::PWR_MGMT0)) & kPwrLowNoise) != 0
	    || _transfer_failed) {
		return false;
	}

	for (uint8_t i = 0; i < _register_config_count; ++i) {
		if (!deferredConfiguration(_register_config[i])) {
			registerSetAndClearBits(_register_config[i]);
		}
	}

	bool success = _register_config_count > 0;

	for (uint8_t i = 0; i < _register_config_count; ++i) {
		if (!deferredConfiguration(_register_config[i])) {
			success &= registerCheck(_register_config[i]);
		}
	}

	return success && !_transfer_failed;
}

void ICM42x::selectRegisterBank(uint8_t bank, bool force)
{
	set_frequency(_register_frequency);

	if (force || !_register_bank_valid || bank != _last_register_bank) {
		uint8_t *cmd = _dma_buffer.data();
		cmd[0] = kBankSelectRegister;
		cmd[1] = bank;

		_register_bank_valid = transferChecked(cmd, 2);

		if (_register_bank_valid) {
			_last_register_bank = bank;
		}
	}
}

uint8_t ICM42x::registerRead(AddressSpace space, uint16_t reg)
{
	set_frequency(_register_frequency);

	selectRegisterBank(static_cast<uint8_t>(space));

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(reg | kDirRead);
	cmd[1] = 0;

	if (!_transfer_failed) {
		transferChecked(cmd, 2);
	}

	return cmd[1];
}

void ICM42x::registerWrite(AddressSpace space, uint16_t reg, uint8_t value)
{
	set_frequency(_register_frequency);

	selectRegisterBank(static_cast<uint8_t>(space));

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = value;

	if (!_transfer_failed) {
		transferChecked(cmd, 2);
	}
}

void ICM42x::registerSetAndClearBits(const RegisterConfig &register_config)
{
	const uint8_t original = registerRead(register_config.space, register_config.reg);
	const uint8_t value    = (original & ~register_config.clear_bits) | register_config.set_bits;

	if (!_transfer_failed && original != value) {
		registerWrite(register_config.space, register_config.reg, value);
	}
}

bool ICM42x::registerCheck(const RegisterConfig &register_config)
{
	const uint8_t value = registerRead(register_config.space, register_config.reg);
	const bool success = !_transfer_failed
			     && (register_config.set_bits == 0 || (value & register_config.set_bits) == register_config.set_bits)
			     && (register_config.clear_bits == 0 || (value & register_config.clear_bits) == 0);

	if (!success) {
		PX4_DEBUG("%s reg %u:0x%02x value 0x%02x (set 0x%02x clear 0x%02x)",
			  _profile.device.name,
			  static_cast<unsigned>(register_config.space),
			  register_config.reg,
			  value,
			  register_config.set_bits,
			  register_config.clear_bits);
	}

	return success;
}

uint16_t ICM42x::fifoReadCount()
{
	selectRegisterBank(0);
	set_frequency(_data_frequency);

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(static_cast<uint8_t>(Bank0::FIFO_COUNTH) | kDirRead);
	cmd[1] = 0;
	cmd[2] = 0;

	if (!transferChecked(cmd, 3)) {
		return 0;
	}

	return combineUnsigned(cmd[1], cmd[2]);
}

bool ICM42x::fifoRead(const hrt_abstime &timestamp_sample, uint8_t requested_samples)
{
	constexpr uint8_t kPrefix = icm42x_config::kFifoTransferPrefix;

	selectRegisterBank(0);
	set_frequency(_data_frequency);
	uint8_t     *fifo_transfer = _dma_buffer.data();
	const size_t transfer_size = kPrefix + requested_samples * _profile.packet_size;

	if (requested_samples == 0
	    || requested_samples > kFifoMaxSamples
	    || transfer_size > _dma_buffer.capacity()
	    || transfer_size > _profile.device.max_transfer_bytes) {
		return false;
	}

	memset(fifo_transfer, 0, transfer_size);
	fifo_transfer[0] = static_cast<uint8_t>(static_cast<uint8_t>(Bank0::INT_STATUS) | kDirRead);

	if (!transferChecked(fifo_transfer, transfer_size)) {
		return false;
	}

	if (fifo_transfer[1] & static_cast<uint8_t>(icm42x_registers::INT_STATUS_BIT::FIFO_FULL_INT)) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	// This transfer contains a count snapshot before the FIFO payload.
	// Compare counts and capacity in the model's native units before converting to complete packets.
	const uint16_t embedded_count = combineUnsigned(fifo_transfer[kPrefix - 2], fifo_transfer[kPrefix - 1]);
	const uint16_t capacity       = _profile.fifo_count_is_records
					? static_cast<uint16_t>(icm42x_registers::FIFO::SIZE / _profile.packet_size)
					: static_cast<uint16_t>(icm42x_registers::FIFO::SIZE);

	if (embedded_count >= capacity) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	const uint16_t embedded_samples  = _profile.fifo_count_is_records ? embedded_count
					   : embedded_count / _profile.packet_size;
	const uint8_t  available_samples = static_cast<uint8_t>(math::min(static_cast<unsigned>(requested_samples),
					   static_cast<unsigned>(embedded_samples)));

	if (available_samples == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	// Keep both batches local until every packet and temperature is validated.
	sensor_accel_fifo_s accel {};
	sensor_gyro_fifo_s  gyro  {};
	static_assert(sizeof(accel.x) == sizeof(gyro.x), "FIFO channel capacities must match");
	accel.timestamp_sample = timestamp_sample;
	gyro.timestamp_sample  = timestamp_sample;

	imu::FifoSampleStats temperatures;
	imu::TdkFifoSample   sample {};
	uint32_t             timestamp_ticks_sum { 0 };
	uint8_t              timestamp_ticks_count { 0 };

	const uint8_t *data         = &fifo_transfer[kPrefix];
	const bool     high_res     = _profile.packet_format == PacketFormat::kHighRes20;
	const auto     decode       = [this, &temperatures, &timestamp_ticks_sum, &timestamp_ticks_count, high_res](
					      imu::ByteCursor & cursor,
	imu::TdkFifoSample & decoded) {
		const uint8_t *packet = cursor.take(_profile.packet_size);
		const bool require_nonzero_timestamp = high_res || _enable_clock_input;

		if (!imu::decodeTdkFifoPacket(
			    packet,
			    _profile.packet_size,
			    false,
			    high_res,
			    require_nonzero_timestamp,
			    true,
			    decoded)) {
			return imu::FifoFrame::kInvalid;
		}

		if (high_res
		    && decoded.temperature != INT16_MIN
		    && !temperatures.add(decoded.temperature)) {
			return imu::FifoFrame::kInvalid;
		}

		if (high_res && (decoded.accel_valid || decoded.gyro_valid)) {
			timestamp_ticks_sum += decoded.timestamp;
			++timestamp_ticks_count;
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
			data, available_samples * _profile.packet_size, accel, gyro, sample, decode);

	if (result.status == imu::FifoDecodeStatus::kEmpty) {
		return false;
	}

	if (result.status != imu::FifoDecodeStatus::kComplete) {
		_transfer_perf.bad_transfer.count();
		fifoReset();

		return false;
	}

	if (high_res && !temperatures.empty() && !processTemperature(temperatures)) {
		return false;
	}

	const float dt = (high_res && timestamp_ticks_count > 0)
			 ? (static_cast<float>(timestamp_ticks_sum) / static_cast<float>(timestamp_ticks_count)) * _timestamp_scale_us
			 : _sample_dt_us;

	accel.dt = dt;
	gyro.dt  = dt;

	const uint64_t errors = errorCount();

	if (gyro.samples > 0) {
		_px4_gyro.set_error_count(errors);
		_px4_gyro.updateFIFO(gyro);
	}

	if (accel.samples > 0) {
		_px4_accel.set_error_count(errors);
		_px4_accel.updateFIFO(accel);
	}

	return true;
}

bool ICM42x::processTemperature(const imu::FifoSampleStats &temperatures)
{
	if (temperatures.empty()) {
		_transfer_perf.bad_transfer.count();
		return false;
	}

	float average = 0.f;

	// Preserve the legacy transfer-error threshold: at most 1000 raw counts from the mean.
	if (!temperatures.meanWithin(1000.f, average)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const float temperature = average / icm42x_registers::TEMPERATURE_SENSITIVITY + icm42x_registers::TEMPERATURE_OFFSET;

	if (!PX4_ISFINITE(temperature)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	return true;
}

void ICM42x::updateTemperature()
{
	selectRegisterBank(0);

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(static_cast<uint8_t>(Bank0::TEMP_DATA1) | kDirRead);
	cmd[1] = 0;
	cmd[2] = 0;

	set_frequency(_data_frequency);

	if (!transferChecked(cmd, 3)) {
		return;
	}

	const int16_t raw         = imu::readBe16(&cmd[1]);
	const float   temperature = raw / icm42x_registers::TEMPERATURE_SENSITIVITY + icm42x_registers::TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

void ICM42x::fifoReset()
{
	_fifo_perf.reset.count();

	registerWrite(
		AddressSpace::kBank0,
		static_cast<uint16_t>(Bank0::SIGNAL_PATH_RESET),
		static_cast<uint8_t>(icm42x_registers::SIGNAL_PATH_RESET_BIT::FIFO_FLUSH));

	if (_profile.variant == Variant::kIcm40609D) {
		registerRead(AddressSpace::kBank0, static_cast<uint16_t>(Bank0::INT_STATUS));
	}

	_drdy_timestamp_sample.store(0);
}

int ICM42x::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM42x *>(arg)->dataReady();

	return 0;
}

void ICM42x::dataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ICM42x::dataReadyInterruptConfigure()
{
	_data_ready_interrupt_enabled =
		_drdy_gpio != 0
		&& px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;

	return _data_ready_interrupt_enabled;
}

bool ICM42x::dataReadyInterruptDisable()
{
	if (!_data_ready_interrupt_enabled || _drdy_gpio == 0) {
		return false;
	}

	_data_ready_interrupt_enabled = false;

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

uint64_t ICM42x::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}

bool ICM42x::transferChecked(uint8_t *data, size_t size)
{
	if (_transfer_failed || device::SPI::transfer(data, data, size) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}
