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
#include "../TdkPacketFifo.hpp"
#include "TdkIcm45686Registers.hpp"

#include <drivers/drv_sensor.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

#include <climits>
#include <cmath>
#include <cstring>

using namespace time_literals;
using namespace frequency_literals;

namespace
{

constexpr uint8_t  kDirRead              { 0x80 };
constexpr uint8_t  kIregAddressHigh      { 0x7c };
constexpr uint8_t  kIregData             { 0x7e };
constexpr unsigned kIregDelayUs          { 4 };

constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

constexpr uint16_t combineUnsigned(uint8_t msb, uint8_t lsb)
{
	return (static_cast<uint16_t>(msb) << 8) | lsb;
}

} // namespace

TdkIcm45686::~TdkIcm45686() = default;

void TdkIcm45686::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;
		uint8_t     samples          = 0;

		if (_data_ready_interrupt_enabled) {
			const hrt_abstime drdy_timestamp = _drdy_timestamp_sample.fetch_and(0);

			if (drdy_timestamp != 0 && (now - drdy_timestamp) < _fifo_empty_interval_us) {

				timestamp_sample = drdy_timestamp;

			} else {
				_drdy_missed_perf.count();
			}

			ScheduleDelayed(_fifo_empty_interval_us * 2);
		}

		const uint16_t fifo_count = fifoReadCount();
		const uint16_t fifo_limit = kFifoCapacity / kPacketSize;

		if (fifo_count >= fifo_limit) {
			fifoReset();
			_fifo_perf.overflow.count();

		} else if (fifo_count == 0) {
			_fifo_perf.empty.count();

		} else {
			if (fifo_count > kFifoMaxSamples) {
				fifoReset();
				_fifo_perf.overflow.count();

			} else {
				samples = static_cast<uint8_t>(fifo_count);
			}
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

		} else if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
			updateTemperature();
			_temperature_update_timestamp = now;
		}

		return;
	}

	// Cold dispatch is size-optimized independently; steady-state sampling returns above.
	runInitialization(now);
}

bool TdkIcm45686::checkConfiguration()
{
	using namespace tdk_icm45686_config;

	constexpr uint8_t first[] { kDirectFirst, kIndirectFirst };
	constexpr uint8_t count[] { kDirectCount, kIndirectCount };

	static_assert(sizeof(first) == sizeof(_checked_register), "Register check cursor count");

	// Preserve one direct and one indirect read per pass, without searching the table.
	for (unsigned space = 0; space < sizeof(_checked_register); ++space) {
		const uint8_t index = _checked_register[space];

		if (!registerCheck(_register_cfg[first[space] + index])) {
			return false;
		}

		_checked_register[space] = index + 1 < count[space] ? index + 1 : 0;
	}

	return !_transfer_failed;
}

uint8_t TdkIcm45686::registerRead(AddressSpace space, uint16_t reg)
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

	uint8_t cmd[2] { static_cast<uint8_t>(reg | kDirRead), 0 };

	if (!_transfer_failed) {
		transferChecked(cmd, sizeof(cmd));
	}

	return cmd[1];
}

void TdkIcm45686::registerWrite(AddressSpace space, uint16_t reg, uint8_t value)
{
	set_frequency(_register_frequency);

	if (space == AddressSpace::kIreg) {
		uint8_t cmd[4] { kIregAddressHigh, static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg), value };

		transferChecked(cmd, sizeof(cmd));
		px4_udelay(kIregDelayUs);

		return;
	}

	uint8_t cmd[2] { static_cast<uint8_t>(reg), value };

	if (!_transfer_failed) {
		transferChecked(cmd, sizeof(cmd));
	}
}

void TdkIcm45686::registerSetAndClearBits(const RegisterConfig &reg_cfg)
{
	const uint8_t original = registerRead(reg_cfg.space, reg_cfg.reg);
	const uint8_t value    = (original & ~reg_cfg.clear_bits) | reg_cfg.set_bits;

	if (!_transfer_failed && original != value) {
		registerWrite(reg_cfg.space, reg_cfg.reg, value);
	}
}

bool TdkIcm45686::registerCheck(const RegisterConfig &reg_cfg)
{
	const uint8_t value = registerRead(reg_cfg.space, reg_cfg.reg);
	const bool success = !_transfer_failed && ((value & reg_cfg.set_bits) == reg_cfg.set_bits)
			     && ((value & reg_cfg.clear_bits) == 0);

	if (!success) {
		PX4_DEBUG("%s reg %u:0x%02x value 0x%02x (set 0x%02x clear 0x%02x)",
			  spiModel().name,
			  static_cast<unsigned>(reg_cfg.space),
			  reg_cfg.reg,
			  value,
			  reg_cfg.set_bits,
			  reg_cfg.clear_bits);
	}

	return success;
}

uint16_t TdkIcm45686::fifoReadCount()
{
	set_frequency(_data_frequency);

	uint8_t cmd[3] {};
	// Retain the existing ICM45686 workaround for a stale first count read.
	const unsigned reads = 2;

	for (unsigned i = 0; i < reads; ++i) {
		cmd[0] = kFifoCountReg | kDirRead;

		if (!transferChecked(cmd, sizeof(cmd))) {
			return 0;
		}
	}

	return combineUnsigned(cmd[2], cmd[1]);
}

bool TdkIcm45686::fifoRead(const hrt_abstime &timestamp_sample, uint8_t requested_samples)
{
	set_frequency(_data_frequency);
	_fifo_transfer[0] = static_cast<uint8_t>((kFifoDataReg) | kDirRead);
	const size_t transfer_size = kFifoTransferPrefix + requested_samples * kPacketSize;

	if (requested_samples == 0
	    || requested_samples > kFifoMaxSamples
	    || transfer_size > spiModel().max_transfer_bytes
	    || !transferChecked(_fifo_transfer, transfer_size)) {
		return false;
	}

	const uint8_t available_samples = requested_samples;

	// Keep both batches local until every packet has been validated.
	// This preserves the all-or-nothing batch policy without re-reading axes.
	sensor_accel_fifo_s accel {};
	sensor_gyro_fifo_s  gyro  {};
	static_assert(sizeof(accel.x) == sizeof(gyro.x), "FIFO channel capacities must match");
	accel.timestamp_sample = timestamp_sample;
	gyro.timestamp_sample  = timestamp_sample;

	// This endpoint always returns complete 16-byte accel/gyro packets.
	// The received count was bounded above; validate every packet before publishing either batch.
	for (uint8_t i = 0; i < available_samples; ++i) {
		const uint8_t *packet = &_fifo_transfer[kFifoTransferPrefix + i * kPacketSize];
		tdk_packet_fifo::Sample sample;

		if (!tdk_packet_fifo::decodePacket(packet, kPacketSize, true, true, false, sample)) {
			_transfer_perf.bad_transfer.count();
			fifoReset();

			return false;
		}

		// decodePacket rejects INT16_MIN, so signed Y/Z negation is representable.
		accel.x[i] = sample.accel[0];
		accel.y[i] = -sample.accel[1];
		accel.z[i] = -sample.accel[2];
		gyro.x[i]  = sample.gyro[0];
		gyro.y[i]  = -sample.gyro[1];
		gyro.z[i]  = -sample.gyro[2];
	}

	accel.samples = available_samples;
	gyro.samples  = available_samples;

	// Both channels use the configured sample interval; FIFO timestamps do not determine dt here.
	// ICM45686 AN-000478 section 7.1: FIFO TMST uses the internal clock even with CLKIN.
	// Its sample interval follows the scaled ODR; it is not TMST divided by the external clock.
	const float dt = _sample_dt_us;

	accel.dt = dt;
	gyro.dt  = dt;

	const uint64_t errors = errorCount();

	_px4_accel.set_error_count(errors);
	_px4_gyro.set_error_count(errors);

	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);

	return true;
}

void TdkIcm45686::updateTemperature()
{

	uint8_t cmd[3] { static_cast<uint8_t>(kTemperatureReg | kDirRead), 0, 0 };

	set_frequency(_data_frequency);

	if (!transferChecked(cmd, sizeof(cmd))) {
		return;
	}

	const int16_t raw = combine(cmd[2], cmd[1]);
	const float temperature = raw / kTemperatureSensitivity
				  + kTemperatureOffset;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

int TdkIcm45686::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<TdkIcm45686 *>(arg)->dataReady();

	return 0;
}

void TdkIcm45686::dataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool TdkIcm45686::dataReadyInterruptConfigure()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;
}

bool TdkIcm45686::dataReadyInterruptDisable()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

uint64_t TdkIcm45686::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}

bool TdkIcm45686::transferChecked(uint8_t *data, size_t size)
{
	if (_transfer_failed || device::SPI::transfer(data, data, size) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}
