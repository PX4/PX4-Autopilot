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

#include "TdkDirect.hpp"
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

namespace
{

constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

} // namespace

TdkDirect::TdkDirect(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_profile(*static_cast<const Profile *>(config.custom_data)),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
	buildRegisterConfig();
}

TdkDirect::~TdkDirect() = default;

void TdkDirect::addRegisterConfig(Register reg, uint8_t set_bits, uint8_t clear_bits)
{
	if (_register_cfg_count < kMaxRegisterConfigs) {
		_register_cfg[_register_cfg_count++] = {reg, set_bits, clear_bits};

	} else {
		_configuration_valid = false;
	}
}

void TdkDirect::buildRegisterConfig()
{
	_register_cfg_count  = 0;
	_configuration_valid = true;

	if (_profile.variant == Variant::kIam20680HP) {
		addRegisterConfig(Register::SMPLRT_DIV, Bit3 | Bit0);
	}

	if (_profile.variant != Variant::kMpu6000) {
		addRegisterConfig(
			Register::CONFIG,
			static_cast<uint8_t>(CONFIG_BIT::FIFO_MODE)
			| static_cast<uint8_t>(CONFIG_BIT::DLPF_CFG_BYPASS_DLPF_8KHZ));
	}

	addRegisterConfig(
		Register::GYRO_CONFIG,
		static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_2000_DPS),
		_profile.variant == Variant::kMpu6000 ? 0 : static_cast<uint8_t>(GYRO_CONFIG_BIT::FCHOICE_B_8KHZ_BYPASS_DLPF));
	addRegisterConfig(
		Register::ACCEL_CONFIG,
		static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G));

	if (_profile.variant != Variant::kMpu6000) {
		// DS-000409 defines these FIFO size bits for IAM20680HP. ICM20689's
		// DS-000143 marks them reserved; retain only its software capacity bound.
		const bool limit_fifo = _profile.variant == Variant::kIam20680HP;

		addRegisterConfig(
			Register::ACCEL_CONFIG2,
			static_cast<uint8_t>(ACCEL_CONFIG2_BIT::ACCEL_FCHOICE_B),
			limit_fifo ? static_cast<uint8_t>(ACCEL_CONFIG2_BIT::FIFO_SIZE) : 0);
	}

	// ICM20602 has one gyro FIFO enable; the other models enable each gyro axis separately.
	const uint8_t fifo_enable = _profile.variant == Variant::kIcm20602
				    ? (static_cast<uint8_t>(FIFO_EN_BIT::GYRO_FIFO_EN) | static_cast<uint8_t>(FIFO_EN_BIT::ACCEL_FIFO_EN))
				    : (static_cast<uint8_t>(FIFO_EN_BIT::XG_FIFO_EN)
				       | static_cast<uint8_t>(FIFO_EN_BIT::YG_FIFO_EN)
				       | static_cast<uint8_t>(FIFO_EN_BIT::ZG_FIFO_EN)
				       | static_cast<uint8_t>(FIFO_EN_BIT::ACCEL_FIFO_EN));

	// Keep the packet layout limited to the configured inertial and temperature channels.
	addRegisterConfig(
		Register::FIFO_EN,
		fifo_enable,
		(_profile.has_fifo_temperature ? 0 : static_cast<uint8_t>(FIFO_EN_BIT::TEMP_FIFO_EN))
		| (_profile.variant == Variant::kMpu9250 ? static_cast<uint8_t>(FIFO_EN_BIT::SLAVE_FIFO_EN) : 0));

	if (_profile.variant == Variant::kIcm20602) {
		addRegisterConfig(
			Register::INT_PIN_CFG,
			static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_LEVEL)
			| static_cast<uint8_t>(INT_PIN_CFG_BIT::LATCH_INT_EN)
			| static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_RD_CLEAR));
		addRegisterConfig(
			Register::INT_ENABLE,
			0,
			static_cast<uint8_t>(INT_ENABLE_BIT::DATA_RDY_INT_EN));
		addRegisterConfig(Register::FIFO_WM_TH1, 0);
		addRegisterConfig(Register::FIFO_WM_TH2, 0);

	} else {
		addRegisterConfig(
			Register::INT_PIN_CFG,
			static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_LEVEL),
			_profile.variant == Variant::kMpu9250 ? static_cast<uint8_t>(INT_PIN_CFG_BIT::BYPASS_EN) : 0);
		addRegisterConfig(
			Register::INT_ENABLE,
			static_cast<uint8_t>(INT_ENABLE_BIT::DATA_RDY_INT_EN));
	}

	if (_profile.variant == Variant::kIcm20602) {
		addRegisterConfig(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN));
		addRegisterConfig(Register::I2C_IF, Bit6);

	} else if (_profile.variant == Variant::kMpu9250) {
		// Six-axis-only endpoint: never enable the auxiliary magnetometer master.
		addRegisterConfig(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN)
			| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS),
			static_cast<uint8_t>(USER_CTRL_BIT::I2C_MST_EN));

	} else {
		addRegisterConfig(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN)
			| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS));
	}

	addRegisterConfig(
		Register::PWR_MGMT_1,
		static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0),
		static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP));

	if (_profile.variant == Variant::kIcm20602) {
		addRegisterConfig(Register::XG_OFFS_TC_H, 0);
		addRegisterConfig(Register::XG_OFFS_TC_L, 0);
		addRegisterConfig(Register::YG_OFFS_TC_H, 0);
		addRegisterConfig(Register::YG_OFFS_TC_L, 0);
		addRegisterConfig(Register::ZG_OFFS_TC_H, 0);
		addRegisterConfig(Register::ZG_OFFS_TC_L, 0);
	}

	if (_profile.has_factory_accel_offsets) {
		addRegisterConfig(Register::XA_OFFSET_H, 0);
		addRegisterConfig(Register::XA_OFFSET_L, 0);
		addRegisterConfig(Register::YA_OFFSET_H, 0);
		addRegisterConfig(Register::YA_OFFSET_L, 0);
		addRegisterConfig(Register::ZA_OFFSET_H, 0);
		addRegisterConfig(Register::ZA_OFFSET_L, 0);
	}

	configureFifoWatermark();
}

int TdkDirect::init()
{
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}

	return reset() ? PX4_OK : PX4_ERROR;
}

bool TdkDirect::reset(uint32_t delay_us)
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

void TdkDirect::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void TdkDirect::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("type: %s", _profile.device.name);

	imu::printSpiStatus(_profile.device,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO empty interval: %u us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	_transfer_perf.print();
	_fifo_perf.print();
	_drdy_missed_perf.print();
}

bool TdkDirect::storeCheckedRegisterValue(Register reg)
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

int TdkDirect::probe()
{
	if (_profile.variant == Variant::kIcm20689 || _profile.variant == Variant::kIam20680HP) {
		// DS-000143 section 4.20 / DS-000409 section 4.18: WHO_AM_I is only
		// guaranteed after the first soft reset, issued as PWR_MGMT_1 = 0x81.
		deviceReset();
		px4_usleep(_profile.reset_wait_us);
	}

	const uint8_t whoami = registerRead(Register::WHO_AM_I);

	if (_transfer_failed || whoami != _profile.whoami) {
		DEVICE_DEBUG("%s: unexpected WHO_AM_I 0x%02x", _profile.device.name, whoami);

		return PX4_ERROR;
	}

	return PX4_OK;
}

void TdkDirect::deviceReset()
{
	uint8_t command = static_cast<uint8_t>(PWR_MGMT_1_BIT::DEVICE_RESET);

	if (_profile.variant == Variant::kIcm20689 || _profile.variant == Variant::kIam20680HP) {
		command |= static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0);
	}

	registerWrite(Register::PWR_MGMT_1, command);
}

bool TdkDirect::resetComplete()
{
	if (registerRead(Register::WHO_AM_I) != _profile.whoami) {
		return false;
	}

	if (_profile.check_reset_pwr && registerRead(Register::PWR_MGMT_1) != _profile.reset_pwr_value) {
		return false;
	}

	return (!_profile.check_reset_config || registerRead(Register::CONFIG) == 0x80) && !_transfer_failed;
}

bool TdkDirect::wakeAndResetSignalPath()
{
	registerWrite(
		Register::PWR_MGMT_1,
		static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0));

	if (_profile.variant == Variant::kIcm20602) {
		registerWrite(Register::I2C_IF, Bit6);
		registerWrite(
			Register::SIGNAL_PATH_RESET,
			static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::ACCEL_RESET)
			| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::TEMP_RESET));
		registerSetAndClearBits(Register::USER_CTRL, static_cast<uint8_t>(USER_CTRL_BIT::SIG_COND_RST), 0);

	} else if (_profile.variant == Variant::kMpu6000
		   || _profile.variant == Variant::kMpu6500
		   || _profile.variant == Variant::kMpu9250) {
		registerWrite(
			Register::SIGNAL_PATH_RESET,
			static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::GYRO_RESET)
			| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::ACCEL_RESET)
			| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::TEMP_RESET));
		registerWrite(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::SIG_COND_RST)
			| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS));

	} else {
		registerWrite(
			Register::SIGNAL_PATH_RESET,
			static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::ACCEL_RESET)
			| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::TEMP_RESET));
		registerWrite(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::SIG_COND_RST)
			| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS));
	}

	return !_transfer_failed;
}

void TdkDirect::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample = now;
		uint16_t    samples          = 0;

		const bool watermark_interrupt = _profile.variant == Variant::kIcm20602;

		if (_data_ready_interrupt_enabled) {
			const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

			if (drdy_timestamp_sample != 0 && (now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
				timestamp_sample = drdy_timestamp_sample;

				if (watermark_interrupt) {
					samples = _fifo_gyro_samples;
				}

			} else {
				_drdy_missed_perf.count();
			}

			// Keep a watchdog read scheduled in case the next interrupt is lost.
			ScheduleDelayed(_fifo_empty_interval_us * 2);
		}

		if (samples == 0) {
			const uint16_t fifo_count = fifoReadCount();

			if (fifo_count >= _profile.fifo_size) {
				_fifo_perf.overflow.count();

				if (!fifoReset()) {
					reset(100_ms);
					return;
				}

			} else if (fifo_count == 0) {
				_fifo_perf.empty.count();

			} else {
				samples = fifo_count / _profile.fifo_packet_size;

				if (watermark_interrupt) {
					// Preserve ICM20602's watermark-sized reads and polling catch-up schedule.
					if (samples > _fifo_gyro_samples) {
						const uint16_t extra = samples - _fifo_gyro_samples;

						samples = _fifo_gyro_samples;

						const uint32_t delay = extra < samples ? (samples - extra) * kFifoSampleDt : 0;

						ScheduleOnInterval(_fifo_empty_interval_us, delay);

					} else if (samples < _fifo_gyro_samples) {
						ScheduleOnInterval(_fifo_empty_interval_us, (_fifo_gyro_samples - samples) * kFifoSampleDt);
					}

				} else if (samples == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(kFifoSampleDt);
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

		const uint8_t minimum_samples = _profile.variant == Variant::kMpu6000 ? 1 : _profile.samples_per_transfer;
		const bool    read_ready      = watermark_interrupt ? samples == _fifo_gyro_samples : samples >= minimum_samples;
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

		} else if (!_profile.has_fifo_temperature && hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
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
			ScheduleDelayed(_profile.reset_wait_us);
			break;
		}

	case State::kWaitForReset: {
			if (resetComplete()) {
				// Learn each device's factory offsets after reset, then include them in periodic register checks.
				if (_profile.variant == Variant::kIcm20602) {
					_factory_read_failed |= !storeCheckedRegisterValue(Register::XG_OFFS_TC_H);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::XG_OFFS_TC_L);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::YG_OFFS_TC_H);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::YG_OFFS_TC_L);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::ZG_OFFS_TC_H);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::ZG_OFFS_TC_L);
				}

				if (_profile.has_factory_accel_offsets) {
					_factory_read_failed |= !storeCheckedRegisterValue(Register::XA_OFFSET_H);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::XA_OFFSET_L);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::YA_OFFSET_H);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::YA_OFFSET_L);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::ZA_OFFSET_H);
					_factory_read_failed |= !storeCheckedRegisterValue(Register::ZA_OFFSET_L);
				}

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
				ScheduleDelayed(_profile.configure_wait_us);

			} else if (hrt_elapsed_time(&_reset_timestamp) > 1_s) {
				PX4_DEBUG("%s reset failed, retrying", _profile.device.name);
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

void TdkDirect::configureAccel()
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

void TdkDirect::configureGyro()
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

void TdkDirect::configureSampleRate(int sample_rate)
{
	// MPU6000 drains gyro at up to 2 kHz while tracking its 1 kHz accel phase across reads.
	// The accel repetition divider must not impose a slower gyro publication interval.
	const float min_interval = kFifoSampleDt * (_profile.variant == Variant::kMpu6000 ? 4 : _profile.samples_per_transfer);

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

void TdkDirect::configureFifoWatermark()
{
	if (_profile.variant != Variant::kIcm20602) {
		return;
	}

	const uint16_t threshold = _fifo_gyro_samples * _profile.fifo_packet_size;

	for (uint8_t i = 0; i < _register_cfg_count; ++i) {
		if (_register_cfg[i].reg == Register::CONFIG) {
			_register_cfg[i].clear_bits |= Bit7;

		} else if (_register_cfg[i].reg == Register::FIFO_WM_TH1) {
			// FIFO_WM_TH[9:8] only; leave the reserved high bits untouched.
			_register_cfg[i].set_bits   = (threshold >> 8) & 0x03;
			_register_cfg[i].clear_bits = 0x03 & ~_register_cfg[i].set_bits;

		} else if (_register_cfg[i].reg == Register::FIFO_WM_TH2) {
			_register_cfg[i].set_bits   = threshold & 0xff;
			_register_cfg[i].clear_bits = static_cast<uint8_t>(~_register_cfg[i].set_bits);
		}
	}
}

bool TdkDirect::configure()
{
	if (!_configuration_valid) {
		return false;
	}

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

int TdkDirect::dataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<TdkDirect *>(arg)->dataReady();

	return 0;
}

void TdkDirect::dataReady()
{
	if (_profile.variant == Variant::kIcm20602) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		ScheduleNow();

	} else if (_drdy_count.fetch_add(1) + 1 >= _fifo_gyro_samples) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		_drdy_count.fetch_sub(_fifo_gyro_samples);
		ScheduleNow();
	}
}

bool TdkDirect::dataReadyInterruptConfigure()
{
	return _drdy_gpio != 0
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;
}

bool TdkDirect::dataReadyInterruptDisable()
{
	return _drdy_gpio != 0 && px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool TdkDirect::registerCheck(const RegisterConfig &reg_cfg)
{
	const uint8_t value = registerRead(reg_cfg.reg);

	return !_transfer_failed && (!reg_cfg.set_bits || (value & reg_cfg.set_bits) == reg_cfg.set_bits)
	       && (!reg_cfg.clear_bits || (value & reg_cfg.clear_bits) == 0);
}

uint8_t TdkDirect::registerRead(Register reg)
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

void TdkDirect::registerWrite(Register reg, uint8_t value)
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

void TdkDirect::registerSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t original = registerRead(reg);
	const uint8_t value    = (original & ~clearbits) | setbits;

	if (!_transfer_failed && original != value) {
		registerWrite(reg, value);
	}
}

uint16_t TdkDirect::fifoReadCount()
{
	uint8_t buffer[3] { static_cast<uint8_t>(static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ), 0, 0 };

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, sizeof(buffer)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return 0;
	}

	return static_cast<uint16_t>(buffer[1]) << 8 | buffer[2];
}

bool TdkDirect::fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FifoTransferBuffer buffer {};

	const uint8_t prefix        = _profile.device.data_prefix_bytes;
	const size_t transfer_size = prefix + samples * _profile.fifo_packet_size;

	if (samples == 0
	    || samples > kFifoMaxSamples
	    || (prefix != 1 && prefix != 3)
	    || transfer_size > sizeof(buffer)
	    || transfer_size > _profile.device.max_transfer_bytes) {
		return false;
	}

	set_frequency(_data_frequency);

	if (_profile.variant == Variant::kIcm20602) {
		// Keep the original count snapshot and FIFO payload in one CS assertion.
		buffer.cmd = static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ;
	}

	if (transfer(reinterpret_cast<uint8_t *>(&buffer), reinterpret_cast<uint8_t *>(&buffer), transfer_size) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	if (_profile.variant == Variant::kIcm20602) {
		const uint16_t count     = (static_cast<uint16_t>(buffer.data[0]) << 8) | buffer.data[1];
		const uint16_t available = count / _profile.fifo_packet_size;

		if (count >= _profile.fifo_size || available > kFifoMaxSamples) {
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
	}

	const uint8_t *fifo = buffer.data + prefix - 1;

	if (_profile.variant == Variant::kMpu9250) {
		uint8_t first_sample = 0;

		if (!tdk::repeatedAccelPhase(
			    fifo,
			    samples * _profile.fifo_packet_size,
			    _profile.fifo_packet_size,
			    first_sample,
			    0)) {
			_transfer_perf.bad_transfer.count();

			return false;
		}

		// The original aligns both channels here but forgets to shorten the received span.
		// Keep that phase without reading an unreceived tail frame at maximum batch size.
		fifo += first_sample * _profile.fifo_packet_size;
		samples -= first_sample;
	}

	if (_profile.has_fifo_temperature && !processTemperature(fifo, samples)) {
		return false;
	}

	if (!processGyro(timestamp_sample, fifo, samples)) {
		return false;
	}

	// Preserve independent gyro availability and original publication order. A bad accel
	// phase still fails this pass, but never publishes unvalidated accel samples.
	return processAccel(timestamp_sample, fifo, samples);
}

bool TdkDirect::fifoReset()
{
	_fifo_perf.reset.count();
	registerWrite(Register::FIFO_EN, 0);
	registerSetAndClearBits(Register::USER_CTRL, static_cast<uint8_t>(USER_CTRL_BIT::FIFO_RST), static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN));
	_drdy_count.store(0);
	_fifo_accel_samples_count = 0;
	memset(_last_accel, 0, sizeof(_last_accel));
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

bool TdkDirect::processAccel(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples)
{
	sensor_accel_fifo_s accel {};

	accel.timestamp_sample = timestamp_sample;
	accel.dt               = kFifoSampleDt * _profile.samples_per_transfer;

	bool valid = true;

	if (_profile.variant == Variant::kMpu6000) {
		bool phase_valid = true;

		const auto decode = [this, &phase_valid](const uint8_t *packet, int16_t (&axes)[3]) {
			++_fifo_accel_samples_count;
			const bool            new_sample = memcmp(packet, _last_accel, sizeof(_last_accel)) != 0;
			imu::FifoSampleResult result     = imu::FifoSampleResult::kSkip;

			if (_fifo_accel_samples_count == _profile.samples_per_transfer) {
				imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);
				result = imu::FifoSampleResult::kAppend;

			} else if (new_sample && _fifo_accel_samples_count > 1) {
				phase_valid = false;
			}

			if (new_sample || _fifo_accel_samples_count == _profile.samples_per_transfer) {
				_fifo_accel_samples_count = 0;
				memcpy(_last_accel, packet, sizeof(_last_accel));
			}

			return result;
		};
		valid = imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ, true>(fifo, samples * _profile.fifo_packet_size,
				_profile.fifo_packet_size, 0, 1, accel, decode);
		valid &= phase_valid;

	} else {
		uint8_t first_sample = 1;

		if (_profile.variant == Variant::kMpu9250) {
			// fifoRead() has already validated/aligned the bounded MPU9250 span for both channels.
			first_sample = 0;

		} else if (!tdk::repeatedAccelPhase(fifo, samples * _profile.fifo_packet_size, _profile.fifo_packet_size, first_sample)) {
			_transfer_perf.bad_transfer.count();

			return false;
		}

		const auto decode = [](const uint8_t *packet, int16_t (&axes)[3]) {
			imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);

			return imu::FifoSampleResult::kAppend;
		};
		valid = imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(fifo, samples * _profile.fifo_packet_size,
				_profile.fifo_packet_size, first_sample, _profile.samples_per_transfer, accel, decode);
	}

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

bool TdkDirect::processGyro(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples)
{
	sensor_gyro_fifo_s gyro {};

	gyro.timestamp_sample = timestamp_sample;
	gyro.dt               = kFifoSampleDt;

	const uint8_t offset = _profile.gyro_offset;

	const auto decode = [offset](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet + offset, axes);

		return imu::FifoSampleResult::kAppend;
	};

	if (!imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(fifo, samples * _profile.fifo_packet_size,
			_profile.fifo_packet_size, 0, 1, gyro, decode)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	_px4_gyro.set_error_count(errorCount());
	_px4_gyro.updateFIFO(gyro);

	return true;
}

bool TdkDirect::processTemperature(const uint8_t fifo[], uint8_t samples)
{
	imu::FifoSampleStats temperatures;

	for (uint8_t i = 0; i < samples; ++i) {
		if (!temperatures.add(combine(fifo[i * _profile.fifo_packet_size + 6], fifo[i * _profile.fifo_packet_size + 7]))) {
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

	const float temperature = average / _profile.temperature_sensitivity + _profile.temperature_offset;

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

void TdkDirect::updateTemperature()
{
	uint8_t buffer[3] { static_cast<uint8_t>(static_cast<uint8_t>(Register::TEMP_OUT_H) | DIR_READ), 0, 0 };

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, sizeof(buffer)) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return;
	}

	const float temperature = combine(buffer[1], buffer[2]) / _profile.temperature_sensitivity + _profile.temperature_offset;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

uint64_t TdkDirect::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}
