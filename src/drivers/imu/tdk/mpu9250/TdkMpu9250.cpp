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

#include "TdkMpu9250.hpp"
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

TdkMpu9250::TdkMpu9250(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_register_frequency(imu::spiConfigFrequency(spiModel(), config.custom2)),
	_data_frequency(imu::spiDataFrequency(spiModel(), config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
	buildRegisterConfig();
}

TdkMpu9250::~TdkMpu9250() = default;

void TdkMpu9250::buildRegisterConfig()
{

	constexpr uint8_t fifo_enable = (static_cast<uint8_t>(FIFO_EN_BIT::XG_FIFO_EN)
					 | static_cast<uint8_t>(FIFO_EN_BIT::YG_FIFO_EN)
					 | static_cast<uint8_t>(FIFO_EN_BIT::ZG_FIFO_EN)
					 | static_cast<uint8_t>(FIFO_EN_BIT::ACCEL_FIFO_EN));

	// Restore fixed masks before relearning device-specific factory offsets.
	static constexpr RegisterConfig defaults[] {
		{
			Register::CONFIG,
			static_cast<uint8_t>(CONFIG_BIT::FIFO_MODE)
			| static_cast<uint8_t>(CONFIG_BIT::DLPF_CFG_BYPASS_DLPF_8KHZ)
		},

		{
			Register::GYRO_CONFIG,
			static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_2000_DPS),
			static_cast<uint8_t>(GYRO_CONFIG_BIT::FCHOICE_B_8KHZ_BYPASS_DLPF)
		},
		{
			Register::ACCEL_CONFIG,
			static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G)
		},

		{
			Register::ACCEL_CONFIG2,
			static_cast<uint8_t>(ACCEL_CONFIG2_BIT::ACCEL_FCHOICE_B),
			0
		},

		// Keep the packet layout limited to the configured inertial and temperature channels.
		{
			Register::FIFO_EN,
			fifo_enable,
			(static_cast<uint8_t>(FIFO_EN_BIT::TEMP_FIFO_EN))
			| (static_cast<uint8_t>(FIFO_EN_BIT::SLAVE_FIFO_EN))
		},

		{
			Register::INT_PIN_CFG,
			static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_LEVEL),
			static_cast<uint8_t>(INT_PIN_CFG_BIT::BYPASS_EN)
		},
		{
			Register::INT_ENABLE,
			static_cast<uint8_t>(INT_ENABLE_BIT::DATA_RDY_INT_EN)
		},

		// Six-axis-only endpoint: never enable the auxiliary magnetometer master.
		{
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN)
			| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS),
			static_cast<uint8_t>(USER_CTRL_BIT::I2C_MST_EN)
		},

		{
			Register::PWR_MGMT_1,
			static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0),
			static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP)
		},

		// Filled from checked factory reads after reset; zero masks initially impose no value.
		{Register::XA_OFFSET_H, 0},
		{Register::XA_OFFSET_L, 0},
		{Register::YA_OFFSET_H, 0},
		{Register::YA_OFFSET_L, 0},
		{Register::ZA_OFFSET_H, 0},
		{Register::ZA_OFFSET_L, 0},
	};

	static_assert(sizeof(defaults) == sizeof(_register_cfg), "Complete fixed register configuration required");

	memcpy(_register_cfg, defaults, sizeof(defaults));
}

int TdkMpu9250::init()
{
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

bool TdkMpu9250::reset(uint32_t delay_us)
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

void TdkMpu9250::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

bool TdkMpu9250::storeCheckedRegisterValue(Register reg)
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

int TdkMpu9250::probe()
{

	const uint8_t whoami = registerRead(Register::WHO_AM_I);

	if (_transfer_failed || whoami != kWhoAmI) {
		DEVICE_DEBUG("%s: unexpected WHO_AM_I 0x%02x", spiModel().name, whoami);

		return PX4_ERROR;
	}

	return PX4_OK;
}

void TdkMpu9250::deviceReset()
{
	uint8_t command = static_cast<uint8_t>(PWR_MGMT_1_BIT::DEVICE_RESET);

	registerWrite(Register::PWR_MGMT_1, command);
}

bool TdkMpu9250::resetComplete()
{
	if (registerRead(Register::WHO_AM_I) != kWhoAmI) {
		return false;
	}

	if (registerRead(Register::PWR_MGMT_1) != kResetPwrValue) {
		return false;
	}

	return !_transfer_failed;
}

bool TdkMpu9250::wakeAndResetSignalPath()
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

void TdkMpu9250::RunImpl()
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

		const uint8_t minimum_samples = kSamplesPerTransfer;
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

	switch (_state) {
	case State::kReset: {
			_factory_read_failed = false;
			_checked_register    = 0;
			buildRegisterConfig();
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
				// Learn each device's factory offsets after reset, then include them in periodic register checks.

				_factory_read_failed |= !storeCheckedRegisterValue(Register::XA_OFFSET_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::XA_OFFSET_L);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::YA_OFFSET_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::YA_OFFSET_L);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::ZA_OFFSET_H);
				_factory_read_failed |= !storeCheckedRegisterValue(Register::ZA_OFFSET_L);

				if (_transfer_failed || _factory_read_failed) {
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
			break; // Handled by the acquisition fast path above.
		}
	}
}

void TdkMpu9250::configureAccel()
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

void TdkMpu9250::configureGyro()
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

void TdkMpu9250::configureSampleRate(int sample_rate)
{
	// Drain complete accel repetition groups without changing the native gyro sample rate.
	const float min_interval = kFifoSampleDt * (kSamplesPerTransfer);

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

bool TdkMpu9250::configure()
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

int TdkMpu9250::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<TdkMpu9250 *>(arg)->dataReady();

	return 0;
}

void TdkMpu9250::dataReady()
{
	if (_drdy_count.fetch_add(1) + 1 >= _fifo_gyro_samples) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		_drdy_count.fetch_sub(_fifo_gyro_samples);
		ScheduleNow();
	}
}

bool TdkMpu9250::dataReadyInterruptConfigure()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;
}

bool TdkMpu9250::dataReadyInterruptDisable()
{
	return _drdy_gpio != 0 && px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool TdkMpu9250::registerCheck(const RegisterConfig &reg_cfg)
{
	const uint8_t value = registerRead(reg_cfg.reg);

	return !_transfer_failed && (!reg_cfg.set_bits || (value & reg_cfg.set_bits) == reg_cfg.set_bits)
	       && (!reg_cfg.clear_bits || (value & reg_cfg.clear_bits) == 0);
}

uint8_t TdkMpu9250::registerRead(Register reg)
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

void TdkMpu9250::registerWrite(Register reg, uint8_t value)
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

void TdkMpu9250::registerSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t original = registerRead(reg);
	const uint8_t value    = (original & ~clearbits) | setbits;

	if (!_transfer_failed && original != value) {
		registerWrite(reg, value);
	}
}

uint16_t TdkMpu9250::fifoReadCount()
{
	uint8_t buffer[3] { static_cast<uint8_t>(static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ), 0, 0 };

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, sizeof(buffer)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return 0;
	}

	return static_cast<uint16_t>(buffer[1]) << 8 | buffer[2];
}

bool TdkMpu9250::fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples)
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

	uint8_t first_sample = 0;

	if (!tdk::repeatedAccelPhase(
		    fifo,
		    samples * kFifoPacketSize,
		    kFifoPacketSize,
		    first_sample,
		    0)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	// The original aligns both channels here but forgets to shorten the received span.
	// Keep that phase without reading an unreceived tail frame at maximum batch size.
	fifo += first_sample * kFifoPacketSize;
	samples -= first_sample;

	if (!processGyro(timestamp_sample, fifo, samples)) {
		return false;
	}

	// Preserve independent gyro availability and original publication order. A bad accel
	// phase still fails this pass, but never publishes unvalidated accel samples.
	return processAccel(timestamp_sample, fifo, samples);
}

bool TdkMpu9250::fifoReset()
{
	_fifo_perf.reset.count();
	registerWrite(Register::FIFO_EN, 0);
	registerSetAndClearBits(Register::USER_CTRL, static_cast<uint8_t>(USER_CTRL_BIT::FIFO_RST), static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN));
	_drdy_count.store(0);
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

bool TdkMpu9250::processAccel(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples)
{
	sensor_accel_fifo_s accel {};

	accel.timestamp_sample = timestamp_sample;
	accel.dt               = kFifoSampleDt * kSamplesPerTransfer;

	bool valid = true;

	uint8_t first_sample = 1;

	// fifoRead() has already validated/aligned the bounded MPU9250 span for both channels.
	first_sample = 0;

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

bool TdkMpu9250::processGyro(
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

void TdkMpu9250::updateTemperature()
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

uint64_t TdkMpu9250::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}
