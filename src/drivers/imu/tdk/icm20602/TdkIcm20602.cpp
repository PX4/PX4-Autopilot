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
#include "../../common/FifoSampleStats.hpp"
#include "../../common/ByteCursor.hpp"
#include "../TdkSamplePhase.hpp"
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

TdkIcm20602::~TdkIcm20602() = default;

bool TdkIcm20602::storeCheckedRegisterValue(Register reg)
{
	for (int retry = 0; retry < 3; ++retry) {
		const uint8_t read1 = registerRead(reg);
		const uint8_t read2 = registerRead(reg);

		if (!_transfer_failed && read1 == read2) {
			for (uint8_t i = 0; i < _register_cfg_count; ++i) {
				if (_register_cfg[i].reg == reg) {
					_register_cfg[i].set_bits   = read1;
					_register_cfg[i].clear_bits = ~read1;

					return true;
				}
			}

		} else {
			PX4_ERR("0x%02hhX read mismatch (0x%02hhX != 0x%02hhX)",
				static_cast<uint8_t>(reg),
				read1,
				read2);
		}
	}

	return false;
}

void TdkIcm20602::deviceReset()
{
	uint8_t command = static_cast<uint8_t>(PWR_MGMT_1_BIT::DEVICE_RESET);

	registerWrite(Register::PWR_MGMT_1, command);
}

void TdkIcm20602::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;
		uint16_t    samples          = 0;

		if (_data_ready_interrupt_enabled) {
			const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

			if (drdy_timestamp_sample != 0 && (now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
				timestamp_sample = drdy_timestamp_sample;

				samples = _fifo_gyro_samples;

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

				// Preserve ICM20602's watermark-sized reads and polling catch-up schedule.
				if (samples > _fifo_gyro_samples) {
					const uint16_t extra = samples - _fifo_gyro_samples;

					samples = _fifo_gyro_samples;

					const uint32_t delay = extra < samples ? (samples - extra) * kFifoSampleDt : 0;

					ScheduleOnInterval(_fifo_empty_interval_us, delay);

				} else if (samples < _fifo_gyro_samples) {
					ScheduleOnInterval(_fifo_empty_interval_us, (_fifo_gyro_samples - samples) * kFifoSampleDt);
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

		const bool    read_ready      = samples == _fifo_gyro_samples;
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

		} else {}

		return;
	}

	// Cold dispatch is size-optimized independently; steady-state sampling returns above.
	runInitialization(now);
}

int TdkIcm20602::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<TdkIcm20602 *>(arg)->dataReady();

	return 0;
}

void TdkIcm20602::dataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool TdkIcm20602::dataReadyInterruptConfigure()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;
}

bool TdkIcm20602::dataReadyInterruptDisable()
{
	return _drdy_gpio != 0 && px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool TdkIcm20602::registerCheck(const RegisterConfig &reg_cfg)
{
	const uint8_t value = registerRead(reg_cfg.reg);

	return !_transfer_failed && ((value & reg_cfg.set_bits) == reg_cfg.set_bits)
	       && ((value & reg_cfg.clear_bits) == 0);
}

uint8_t TdkIcm20602::registerRead(Register reg)
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

void TdkIcm20602::registerWrite(Register reg, uint8_t value)
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

void TdkIcm20602::registerSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t original = registerRead(reg);
	const uint8_t value    = (original & ~clearbits) | setbits;

	if (!_transfer_failed && original != value) {
		registerWrite(reg, value);
	}
}

uint16_t TdkIcm20602::fifoReadCount()
{
	uint8_t buffer[3] { static_cast<uint8_t>(static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ), 0, 0 };

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, sizeof(buffer)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return 0;
	}

	return static_cast<uint16_t>(buffer[1]) << 8 | buffer[2];
}

bool TdkIcm20602::fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	// Zero dummy TX bytes without storing a full initialized buffer in Flash.
	FifoTransferBuffer buffer;

	memset(&buffer, 0, sizeof(buffer));
	buffer.cmd = static_cast<uint8_t>(Register::FIFO_R_W) | DIR_READ;

	const uint8_t prefix        = spiModel().data_prefix_bytes;
	const size_t transfer_size = prefix + samples * kFifoPacketSize;

	if (samples == 0
	    || samples > kFifoMaxSamples
	    || prefix != 3
	    || transfer_size > sizeof(buffer)
	    || transfer_size > spiModel().max_transfer_bytes) {
		return false;
	}

	set_frequency(_data_frequency);

	// Keep the original count snapshot and FIFO payload in one CS assertion.
	buffer.cmd = static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ;

	if (transfer(reinterpret_cast<uint8_t *>(&buffer), reinterpret_cast<uint8_t *>(&buffer), transfer_size) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const uint16_t count     = (static_cast<uint16_t>(buffer.data[0]) << 8) | buffer.data[1];
	const uint16_t available = count / kFifoPacketSize;

	if (count >= kFifoSize || available > kFifoMaxSamples) {
		_fifo_perf.overflow.count();

		if (!fifoReset()) {
			reset(100_ms);
		}

		return false;
	}

	samples = math::min(static_cast<uint16_t>(samples), available);

	if (samples == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	const uint8_t *fifo = buffer.data + prefix - 1;

	if (!processTemperature(fifo, samples)) {
		return false;
	}

	if (!processGyro(timestamp_sample, fifo, samples)) {
		return false;
	}

	// Preserve independent gyro availability and original publication order. A bad accel
	// phase still fails this pass, but never publishes unvalidated accel samples.
	return processAccel(timestamp_sample, fifo, samples);
}

bool TdkIcm20602::processAccel(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples)
{
	sensor_accel_fifo_s accel {};

	accel.timestamp_sample = timestamp_sample;
	accel.dt               = kFifoSampleDt * kSamplesPerTransfer;

	bool valid = true;

	uint8_t first_sample = 1;

	if (!tdk::repeatedAccelPhase(fifo, samples * kFifoPacketSize, kFifoPacketSize, first_sample)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const auto decode = [](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);

		return imu::FifoSampleResult::kAppend;
	};
	valid = imu::decodeFixedFifoSamples<kFifoPacketSize, imu::FifoAxisMapping::kFlipYZ>(fifo, samples * kFifoPacketSize,
			samples, first_sample, kSamplesPerTransfer, accel, decode);

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

bool TdkIcm20602::processGyro(
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

bool TdkIcm20602::processTemperature(const uint8_t fifo[], uint8_t samples)
{
	imu::FifoSampleStats temperatures;

	for (uint8_t i = 0; i < samples; ++i) {
		if (!temperatures.add(combine(fifo[i * kFifoPacketSize + 6], fifo[i * kFifoPacketSize + 7]))) {
			_transfer_perf.bad_transfer.count();

			return false;
		}
	}

	float average;

	// Preserve the legacy transfer-error threshold: at most 1000 raw counts from the mean.
	if (!temperatures.meanWithin(1000.f, average)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	const float temperature = average / kTemperatureSensitivity + kTemperatureOffset;

	if (!PX4_ISFINITE(temperature)
	    || temperature < -40.f
	    || temperature > 85.f) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	return true;
}

uint64_t TdkIcm20602::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}
