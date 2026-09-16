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
#include "../../common/ByteCursor.hpp"
#include "../../common/FifoBatch.hpp"

#include <drivers/drv_sensor.h>

#include <climits>
#include <cmath>
#include <cstring>

using namespace tdk_direct_registers;
using namespace time_literals;
using namespace frequency_literals;

namespace
{

constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

} // namespace

TdkMpu6000::~TdkMpu6000() = default;

// Immutable masks are shared by configuration, recovery and register surveillance.
constexpr TdkMpu6000::RegisterConfig TdkMpu6000::_register_cfg[kMaxRegisterConfigs] {
	{
		Register::GYRO_CONFIG,
		static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_2000_DPS),
		0
	},
	{
		Register::ACCEL_CONFIG,
		static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G)
	},

	// Keep the packet layout limited to the configured inertial and temperature channels.
	{
		Register::FIFO_EN,
		(static_cast<uint8_t>(FIFO_EN_BIT::XG_FIFO_EN)
		 | static_cast<uint8_t>(FIFO_EN_BIT::YG_FIFO_EN)
		 | static_cast<uint8_t>(FIFO_EN_BIT::ZG_FIFO_EN)
		 | static_cast<uint8_t>(FIFO_EN_BIT::ACCEL_FIFO_EN)),
		(static_cast<uint8_t>(FIFO_EN_BIT::TEMP_FIFO_EN))
	},

	{
		Register::INT_PIN_CFG,
		static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_LEVEL),
		0
	},
	{
		Register::INT_ENABLE,
		static_cast<uint8_t>(INT_ENABLE_BIT::DATA_RDY_INT_EN)
	},

	{
		Register::USER_CTRL,
		static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN)
		| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS)
	},

	{
		Register::PWR_MGMT_1,
		static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0),
		static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP)
	},
};

void TdkMpu6000::deviceReset()
{
	uint8_t command = static_cast<uint8_t>(PWR_MGMT_1_BIT::DEVICE_RESET);

	registerWrite(Register::PWR_MGMT_1, command);
}

void TdkMpu6000::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample   = now;
		uint16_t    samples            = 0;
		bool        timestamp_from_irq = false;

		if (_data_ready_interrupt_enabled) {
			const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

			if (drdy_timestamp_sample != 0 && (now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
				timestamp_sample   = drdy_timestamp_sample;
				timestamp_from_irq = true;

			} else {
				_drdy_missed_perf.count();
			}

			// Keep a watchdog read scheduled in case the next interrupt is lost.
			ScheduleDelayed(_fifo_empty_interval_us * 2);
		}

		if (samples == 0) {
			const uint16_t fifo_count = fifoReadCount();

			if (fifo_count >= kFifoSize) {
				_fifo_perf.overflow.count();

				if (!fifoReset()) {
					reset(100_ms);
					return;
				}

			} else if (fifo_count == 0) {
				_fifo_perf.empty.count();

			} else {
				samples = fifo_count / kFifoPacketSize;

				if (samples == _fifo_gyro_samples + 1) {
					// The IRQ already timestamps the last frame retained in this batch.
					// Only a polling timestamp includes the extra frame left in the FIFO.
					if (!timestamp_from_irq) {
						timestamp_sample -= static_cast<int>(kFifoSampleDt);
					}

					--samples;
				}

				if (samples > kFifoMaxSamples) {
					_fifo_perf.overflow.count();
					samples = 0;

					if (!fifoReset()) {
						reset(100_ms);
						return;
					}
				}
			}
		}

		const uint8_t minimum_samples = 1;
		const bool    read_ready      = samples >= minimum_samples;
		const bool    success         = read_ready && fifoRead(timestamp_sample, samples);

		if (success) {
			if (_failure_count > 0) {
				--_failure_count;
			}

		} else {
			// fifoRead() may have scheduled recovery after a failed FIFO reset.
			// Do not overwrite its delay with configuration surveillance or another read.
			if (_state == State::kReset) {
				return;
			}

			if (++_failure_count > 10) {
				reset();
				return;
			}
		}

		// Check one register per pass to spread monitoring work across FIFO reads.
		if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
			if (_register_cfg_count > 0 && registerCheck(_register_cfg[_checked_register])) {
				_last_config_check_timestamp = now;
				_checked_register            = (_checked_register + 1) % _register_cfg_count;

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

int TdkMpu6000::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<TdkMpu6000 *>(arg)->dataReady();

	return 0;
}

void TdkMpu6000::dataReady()
{
	if (_drdy_count.fetch_add(1) + 1 >= _fifo_gyro_samples) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		_drdy_count.fetch_sub(_fifo_gyro_samples);
		ScheduleNow();
	}
}

bool TdkMpu6000::dataReadyInterruptConfigure()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;
}

bool TdkMpu6000::dataReadyInterruptDisable()
{
	return _drdy_gpio != 0 && px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool TdkMpu6000::registerCheck(const RegisterConfig &reg_cfg)
{
	const uint8_t value = registerRead(reg_cfg.reg);

	return !_transfer_failed && ((value & reg_cfg.set_bits) == reg_cfg.set_bits)
	       && ((value & reg_cfg.clear_bits) == 0);
}

uint8_t TdkMpu6000::registerRead(Register reg)
{
	if (_transfer_failed) {
		return 0;
	}

	uint8_t cmd[2] { static_cast<uint8_t>(static_cast<uint8_t>(reg) | DIR_READ), 0 };

	set_frequency(_register_frequency);

	if (transfer(cmd, cmd, sizeof(cmd)) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}

	return cmd[1];
}

void TdkMpu6000::registerWrite(Register reg, uint8_t value)
{
	if (_transfer_failed) {
		return;
	}

	uint8_t cmd[2] { static_cast<uint8_t>(reg), value };

	set_frequency(_register_frequency);

	if (transfer(cmd, cmd, sizeof(cmd)) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}
}

void TdkMpu6000::registerSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t original = registerRead(reg);
	const uint8_t value    = (original & ~clearbits) | setbits;

	if (!_transfer_failed && original != value) {
		registerWrite(reg, value);
	}
}

uint16_t TdkMpu6000::fifoReadCount()
{
	uint8_t buffer[3] { static_cast<uint8_t>(static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ), 0, 0 };

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, sizeof(buffer)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return 0;
	}

	return static_cast<uint16_t>(buffer[1]) << 8 | buffer[2];
}

bool TdkMpu6000::fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	// Zero dummy TX bytes without storing a full initialized buffer in Flash.
	FifoTransferBuffer buffer;

	memset(&buffer, 0, sizeof(buffer));
	buffer.cmd = static_cast<uint8_t>(Register::FIFO_R_W) | DIR_READ;

	const uint8_t prefix        = spiModel().data_prefix_bytes;
	const size_t transfer_size = prefix + samples * kFifoPacketSize;

	if (samples == 0
	    || samples > kFifoMaxSamples
	    || prefix != 1
	    || transfer_size > sizeof(buffer)
	    || transfer_size > spiModel().max_transfer_bytes) {
		return false;
	}

	set_frequency(_data_frequency);

	if (transfer(reinterpret_cast<uint8_t *>(&buffer), reinterpret_cast<uint8_t *>(&buffer), transfer_size) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const uint8_t *fifo = buffer.data + prefix - 1;

	if (!processGyro(timestamp_sample, fifo, samples)) {
		return false;
	}

	// Preserve independent gyro availability and original publication order. A bad accel
	// phase still fails this pass, but never publishes unvalidated accel samples.
	return processAccel(timestamp_sample, fifo, samples);
}

bool TdkMpu6000::processAccel(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples)
{
	sensor_accel_fifo_s accel {};

	accel.timestamp_sample = timestamp_sample;
	accel.dt               = kFifoSampleDt * kSamplesPerTransfer;

	bool valid = true;

	bool phase_valid = true;

	const auto decode = [this, &phase_valid](const uint8_t *packet, int16_t (&axes)[3]) {
		++_fifo_accel_samples_count;
		const bool            new_sample = memcmp(packet, _last_accel, sizeof(_last_accel)) != 0;
		imu::FifoSampleResult result     = imu::FifoSampleResult::kSkip;

		if (_fifo_accel_samples_count == kSamplesPerTransfer) {
			imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);
			result = imu::FifoSampleResult::kAppend;

		} else if (new_sample && _fifo_accel_samples_count > 1) {
			phase_valid = false;
		}

		if (new_sample || _fifo_accel_samples_count == kSamplesPerTransfer) {
			_fifo_accel_samples_count = 0;
			memcpy(_last_accel, packet, sizeof(_last_accel));
		}

		return result;
	};
	valid = imu::decodeFixedFifoSamples<kFifoPacketSize, imu::FifoAxisMapping::kFlipYZ, true>(fifo,
			samples * kFifoPacketSize,
			samples, 0, 1, accel, decode);
	valid &= phase_valid;

	if (!valid) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_accel.set_error_count(errorCount());

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}

	return true;
}

bool TdkMpu6000::processGyro(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples)
{
	sensor_gyro_fifo_s gyro {};

	gyro.timestamp_sample = timestamp_sample;
	gyro.dt               = kFifoSampleDt;

	const uint8_t offset = kGyroOffset;

	const auto decode = [offset](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet + offset, axes);

		return imu::FifoSampleResult::kAppend;
	};

	if (!imu::decodeFixedFifoSamples<kFifoPacketSize, imu::FifoAxisMapping::kFlipYZ>(fifo, samples * kFifoPacketSize,
			samples, 0, 1, gyro, decode)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_gyro.set_error_count(errorCount());
	_px4_gyro.updateFIFO(gyro);

	return true;
}

void TdkMpu6000::updateTemperature()
{
	uint8_t buffer[3] { static_cast<uint8_t>(static_cast<uint8_t>(Register::TEMP_OUT_H) | DIR_READ), 0, 0 };

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, sizeof(buffer)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return;
	}

	const float temperature = combine(buffer[1], buffer[2]) / kTemperatureSensitivity + kTemperatureOffset;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

uint64_t TdkMpu6000::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}
