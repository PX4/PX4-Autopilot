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
 * @file MPUDirect.cpp
 * @brief Implementation of the MPUDirect flat-register TDK SPI IMU family driver.
 */

#include "MPUDirect.hpp"

#include <lib/drivers/imu/ByteCursor.hpp>
#include <lib/drivers/imu/FifoBatch.hpp>
#include <lib/drivers/imu/FifoSampleStats.hpp>
#include <lib/drivers/imu/ImuSamplePhase.hpp>

#include <cmath>
#include <cstring>

using namespace mpu_direct_registers;
using namespace time_literals;

MPUDirect::MPUDirect(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_endpoint_guard(config),
	_profile(*static_cast<const Profile *>(config.custom_data)),
	_spi_mode(config.spi_mode),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	configureSampleRate(_px4_gyro.get_max_rate_hz());
	buildRegisterConfig();
}

MPUDirect::~MPUDirect()
{
	dataReadyInterruptDisable();
}

void MPUDirect::addRegisterConfig(Register reg, uint8_t set_bits, uint8_t clear_bits)
{
	if (_register_config_count < kMaxRegisterConfigs) {
		_register_config[_register_config_count++] = {reg, set_bits, clear_bits};

	} else {
		_configuration_valid = false;
	}
}

void MPUDirect::buildRegisterConfig()
{
	_register_config_count = 0;
	_configuration_valid   = true;

	if (_profile.variant == Variant::kIam20680HP) {
		addRegisterConfig(Register::SMPLRT_DIV, Bit3 | Bit0);

	} else if (isMpu6000()) {
		addRegisterConfig(Register::SMPLRT_DIV, 0, 0xFF);
	}

	if (isMpu6000()) {
		addRegisterConfig(Register::CONFIG, 0, 0x3F);

	} else {
		addRegisterConfig(
			Register::CONFIG,
			static_cast<uint8_t>(CONFIG_BIT::FIFO_MODE)
			| static_cast<uint8_t>(CONFIG_BIT::DLPF_CFG_BYPASS_DLPF_8KHZ));
	}

	addRegisterConfig(
		Register::GYRO_CONFIG,
		static_cast<uint8_t>(GYRO_CONFIG_BIT::FS_SEL_2000_DPS),
		isMpu6000() ? 0 : static_cast<uint8_t>(GYRO_CONFIG_BIT::FCHOICE_B_8KHZ_BYPASS_DLPF));
	addRegisterConfig(
		Register::ACCEL_CONFIG,
		static_cast<uint8_t>(ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G));

	if (!isMpu6000()) {
		// DS-000409 defines these FIFO size bits for IAM20680HP. ICM20689's
		// DS-000143 marks them reserved; retain only its software capacity bound.
		const bool limit_fifo = _profile.variant == Variant::kIam20680HP;

		addRegisterConfig(
			Register::ACCEL_CONFIG2,
			static_cast<uint8_t>(ACCEL_CONFIG2_BIT::ACCEL_FCHOICE_B),
			limit_fifo ? static_cast<uint8_t>(ACCEL_CONFIG2_BIT::FIFO_SIZE) : 0);
	}

	// ICM20602 has one gyro FIFO enable; the other models enable each gyro axis separately.
	const uint8_t fifo_enable = isIcm20602()
				    ? (static_cast<uint8_t>(FIFO_EN_BIT::GYRO_FIFO_EN) | static_cast<uint8_t>(FIFO_EN_BIT::ACCEL_FIFO_EN))
				    : (static_cast<uint8_t>(FIFO_EN_BIT::XG_FIFO_EN)
				       | static_cast<uint8_t>(FIFO_EN_BIT::YG_FIFO_EN)
				       | static_cast<uint8_t>(FIFO_EN_BIT::ZG_FIFO_EN)
				       | static_cast<uint8_t>(FIFO_EN_BIT::ACCEL_FIFO_EN));

	// Keep the packet layout limited to the configured inertial and temperature channels.
	addRegisterConfig(
		Register::FIFO_EN,
		fifo_enable,
		static_cast<uint8_t>(FIFO_EN_BIT::SLV_FIFO_MASK)
		| (_profile.has_fifo_temperature ? 0 : static_cast<uint8_t>(FIFO_EN_BIT::TEMP_FIFO_EN)));

	if (isIcm20602()) {
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
		addRegisterConfig(
			Register::ACCEL_INTEL_CTRL,
			static_cast<uint8_t>(ACCEL_INTEL_CTRL_BIT::OUTPUT_LIMIT));

	} else {
		addRegisterConfig(
			Register::INT_PIN_CFG,
			static_cast<uint8_t>(INT_PIN_CFG_BIT::INT_LEVEL));
		addRegisterConfig(
			Register::INT_ENABLE,
			static_cast<uint8_t>(INT_ENABLE_BIT::DATA_RDY_INT_EN));
	}

	const uint8_t user_ctrl_clear = static_cast<uint8_t>(USER_CTRL_BIT::DMP_EN)
					| static_cast<uint8_t>(USER_CTRL_BIT::I2C_MST_EN);

	if (isIcm20602()) {
		addRegisterConfig(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN),
			user_ctrl_clear);
		addRegisterConfig(Register::I2C_IF, static_cast<uint8_t>(I2C_IF_BIT::I2C_IF_DIS));

	} else if (_profile.variant == Variant::kMpu6500) {
		addRegisterConfig(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN),
			user_ctrl_clear | static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS));

	} else {
		addRegisterConfig(
			Register::USER_CTRL,
			static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN)
			| static_cast<uint8_t>(USER_CTRL_BIT::I2C_IF_DIS),
			user_ctrl_clear);
	}

	addRegisterConfig(
		Register::PWR_MGMT_1,
		static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0),
		static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP));

	if (isIcm20602()) {
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

int MPUDirect::init()
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

bool MPUDirect::reset(uint32_t delay_us)
{
	_state = State::kReset;
	_config_check_failure_count = 0;
	dataReadyInterruptDisable();
	_drdy_anchor.reset();
	_drdy_backlog_drain = false;
	ScheduleClear();

	if (delay_us > 0) {
		ScheduleDelayed(delay_us);

	} else {
		ScheduleNow();
	}

	return true;
}

void MPUDirect::exit_and_cleanup()
{
	dataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void MPUDirect::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("type: %s", _profile.device.name);

	imu::printSpiStatus(_profile.device,
			    _spi_mode,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO empty interval: %u us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	_transfer_perf.print();
	_fifo_perf.print();
	_drdy_missed_perf.print();
}

bool MPUDirect::storeCheckedRegisterValue(Register reg)
{
	for (int retry = 0; retry < 3; ++retry) {
		_transfer_failed = false;
		const uint8_t read1 = registerRead(reg);
		const uint8_t read2 = registerRead(reg);

		if (_transfer_failed) {
			PX4_ERR("0x%02hhX SPI read failed", static_cast<uint8_t>(reg));

		} else if (read1 == read2) {
			for (uint8_t i = 0; i < _register_config_count; ++i) {
				if (_register_config[i].reg == reg) {
					_register_config[i].set_bits   = read1;
					_register_config[i].clear_bits = static_cast<uint8_t>(~read1);

					return true;
				}
			}

			return false;

		} else {
			PX4_ERR("0x%02hhX read mismatch (0x%02hhX != 0x%02hhX)",
				static_cast<uint8_t>(reg),
				read1,
				read2);
		}
	}

	return false;
}

int MPUDirect::probe()
{
	uint8_t whoami = 0;

	for (int attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;
		whoami = registerRead(Register::WHO_AM_I);

		if (!_transfer_failed && whoami == _profile.whoami) {
			return PX4_OK;
		}
	}

	DEVICE_DEBUG("%s: unexpected WHO_AM_I 0x%02x", _profile.device.name, whoami);

	return PX4_ERROR;
}

void MPUDirect::deviceReset()
{
	registerWrite(Register::PWR_MGMT_1, static_cast<uint8_t>(PWR_MGMT_1_BIT::DEVICE_RESET));
}

bool MPUDirect::resetComplete()
{
	if (registerRead(Register::WHO_AM_I) != _profile.whoami) {
		return false;
	}

	if (_profile.check_reset_pwr && registerRead(Register::PWR_MGMT_1) != _profile.reset_pwr_value) {
		return false;
	}

	return (!_profile.check_reset_config || registerRead(Register::CONFIG) == 0x80) && !_transfer_failed;
}

bool MPUDirect::wakeAndResetSignalPath()
{
	if (isIcm20602()) {
		registerWrite(Register::I2C_IF, static_cast<uint8_t>(I2C_IF_BIT::I2C_IF_DIS));
		registerWrite(Register::PWR_MGMT_1, static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0));
		registerWrite(
			Register::SIGNAL_PATH_RESET,
			static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::ACCEL_RESET)
			| static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::TEMP_RESET));
		registerSetAndClearBits(Register::USER_CTRL, static_cast<uint8_t>(USER_CTRL_BIT::SIG_COND_RST), 0);

	} else if (_profile.variant == Variant::kMpu6000 || _profile.variant == Variant::kMpu6500) {
		registerWrite(Register::PWR_MGMT_1, static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0));
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
		registerWrite(Register::PWR_MGMT_1, static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0));
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

void MPUDirect::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	_transfer_failed = false;

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kFifoRead, 1)) {
		hrt_abstime timestamp_sample     = now;
		uint16_t    samples              = 0;
		bool        using_drdy_timestamp = false;

		const bool    watermark_interrupt = isIcm20602();
		const uint8_t minimum_samples     = isMpu6000() ? 1 : _profile.samples_per_transfer;

		imu::DrdySampleAnchor::Snapshot drdy_snapshot {};
		bool drdy_snapshot_consistent = false;
		const bool watermark_backlog_drain = _drdy_backlog_drain;
		_drdy_backlog_drain = false;

		if (_data_ready_interrupt_enabled) {
			if (watermark_interrupt) {
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if (watermark_backlog_drain) {
					samples = _fifo_gyro_samples;

				} else if (drdy_timestamp_sample != 0
					   && now >= drdy_timestamp_sample
					   && (now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample     = drdy_timestamp_sample;
					using_drdy_timestamp = true;
					samples              = _fifo_gyro_samples;

				} else {
					_drdy_missed_perf.count();
				}
			}

			// Keep a watchdog read scheduled in case the next interrupt is lost.
			ScheduleDelayed(_fifo_empty_interval_us * 2);
		}

		if (samples == 0) {
			uint16_t    fifo_count = 0;
			hrt_abstime count_now  = now;
			bool        count_ok   = false;

			if (_data_ready_interrupt_enabled && !watermark_interrupt) {
				// Sandwich FIFO_COUNTH with anchor snapshots so an 8 kHz DRDY edge during SPI setup/lock cannot skew epochs.
				for (unsigned attempt = 0; attempt < 2; ++attempt) {
					_transfer_failed = false;
					const imu::DrdySampleAnchor::Snapshot snap_before = _drdy_anchor.snapshot();
					count_ok      = fifoReadCount(fifo_count);
					count_now     = hrt_absolute_time();
					drdy_snapshot = _drdy_anchor.snapshot();

					if (!count_ok) {
						break;
					}

					if (snap_before.pulse_count == drdy_snapshot.pulse_count) {
						drdy_snapshot_consistent = true;
						break;
					}
				}

			} else {
				count_ok  = fifoReadCount(fifo_count);
				count_now = hrt_absolute_time();
			}

			if (!count_ok) {
				// bad_transfer already recorded; avoid double-counting as FIFO empty.

			} else if (fifo_count >= _profile.fifo_size) {
				_fifo_perf.overflow.count();

				if (!fifoReset()) {
					reset(100_ms);
					return;
				}

			} else if (fifo_count == 0) {
				_fifo_perf.empty.count();

			} else {
				const uint16_t available_samples = fifo_count / _profile.fifo_packet_size;
				samples = available_samples;

				if (watermark_interrupt) {
					timestamp_sample = count_now;

					// Preserve ICM20602's watermark-sized reads and polling catch-up schedule;
					// fifoRead() computes the exact backlog timestamp from the latched FIFO count.
					if (available_samples > _fifo_gyro_samples) {
						const uint16_t extra = available_samples - _fifo_gyro_samples;
						samples = _fifo_gyro_samples;

						if (!_data_ready_interrupt_enabled) {
							const uint32_t delay = extra < samples
									       ? static_cast<uint32_t>(roundf((samples - extra) * kFifoSampleDt))
									       : 0;

							ScheduleOnInterval(_fifo_empty_interval_us, delay);
						}

					} else if (available_samples < _fifo_gyro_samples && !_data_ready_interrupt_enabled) {
						ScheduleOnInterval(_fifo_empty_interval_us,
								   static_cast<uint32_t>(roundf((_fifo_gyro_samples - available_samples) * kFifoSampleDt)));
					}

				} else {
					samples = math::min<uint16_t>(available_samples, static_cast<uint16_t>(_fifo_gyro_samples));

					if (samples >= minimum_samples) {
						if (_data_ready_interrupt_enabled) {
							if (drdy_snapshot_consistent) {
								const imu::DrdyBatchTimestamp resolved = _drdy_anchor.resolve(
											drdy_snapshot,
											count_now,
											available_samples,
											samples,
											kFifoSampleDt,
											_fifo_empty_interval_us);
								timestamp_sample     = resolved.timestamp_sample;
								using_drdy_timestamp = resolved.using_drdy;

							} else {
								timestamp_sample = imu::adjustFifoBacklogTimestamp(
											   count_now,
											   false,
											   available_samples,
											   samples,
											   kFifoSampleDt);
								using_drdy_timestamp = false;
							}

							if (!using_drdy_timestamp) {
								_drdy_missed_perf.count();
							}

						} else {
							timestamp_sample = imu::adjustFifoBacklogTimestamp(
										   count_now,
										   false,
										   available_samples,
										   samples,
										   kFifoSampleDt);
						}

						if ((available_samples - samples) >= _fifo_gyro_samples) {
							ScheduleNow();
						}
					}
				}
			}
		}

		const bool read_ready = watermark_interrupt ? samples == _fifo_gyro_samples : samples >= minimum_samples;
		const bool phase_wait = !_data_ready_interrupt_enabled
					&& !_transfer_failed
					&& samples > 0
					&& !read_ready
					&& now >= _last_sample_timestamp
					&& (now - _last_sample_timestamp) < 50_ms;
		const bool success = read_ready && fifoRead(timestamp_sample, static_cast<uint8_t>(samples), using_drdy_timestamp);

		if (success) {
			_last_sample_timestamp = now;

			if (_failure_count > 0) {
				--_failure_count;
			}

		} else if (!phase_wait) {
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
		if ((!success && !phase_wait) || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
			_transfer_failed = false;

			if (_register_config_count > 0 && registerCheck(_register_config[_checked_register])) {
				_config_check_failure_count  = 0;
				_last_config_check_timestamp = now;
				_checked_register            = (_checked_register + 1) % _register_config_count;

			} else if (!_transfer_failed) {
				_transfer_perf.bad_register.count();
				reset();

			} else if (++_config_check_failure_count > 10) {
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
				if (isIcm20602()) {
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

				_state                 = State::kFifoRead;
				_last_sample_timestamp = now;
				_drdy_anchor.reset();
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

void MPUDirect::configureAccel()
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

void MPUDirect::configureGyro()
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

void MPUDirect::configureSampleRate(int sample_rate)
{
	// MPU6000 drains gyro at up to 2 kHz while tracking its 1 kHz accel phase across reads.
	// The accel repetition divider must not impose a slower gyro publication interval.
	const float min_interval = kFifoSampleDt * (isMpu6000() ? 4 : _profile.samples_per_transfer);

	_fifo_empty_interval_us = static_cast<uint16_t>(
					  math::max(
							  roundf((1e6f / static_cast<float>(math::max(sample_rate, 1))) / min_interval) * min_interval,
							  min_interval));

	_fifo_gyro_samples = static_cast<int32_t>(
				     roundf(
					     math::min(
							     static_cast<float>(_fifo_empty_interval_us) / (1e6f / kGyroRate),
							     static_cast<float>(kFifoMaxSamples))));

	_fifo_empty_interval_us = static_cast<uint16_t>(_fifo_gyro_samples * (1e6f / kGyroRate));
}

void MPUDirect::configureFifoWatermark()
{
	if (!isIcm20602()) {
		return;
	}

	const uint16_t threshold = static_cast<uint16_t>(_fifo_gyro_samples * _profile.fifo_packet_size);

	for (uint8_t i = 0; i < _register_config_count; ++i) {
		if (_register_config[i].reg == Register::CONFIG) {
			_register_config[i].clear_bits |= Bit7;

		} else if (_register_config[i].reg == Register::FIFO_WM_TH1) {
			// FIFO_WM_TH[9:8] only; leave the reserved high bits untouched.
			_register_config[i].set_bits   = static_cast<uint8_t>((threshold >> 8) & 0x03);
			_register_config[i].clear_bits = static_cast<uint8_t>(0x03 & ~_register_config[i].set_bits);

		} else if (_register_config[i].reg == Register::FIFO_WM_TH2) {
			_register_config[i].set_bits   = static_cast<uint8_t>(threshold & 0xff);
			_register_config[i].clear_bits = static_cast<uint8_t>(~_register_config[i].set_bits);
		}
	}
}

bool MPUDirect::configure()
{
	if (!_configuration_valid) {
		return false;
	}

	for (uint8_t i = 0; i < _register_config_count; ++i) {
		registerSetAndClearBits(_register_config[i].reg, _register_config[i].set_bits, _register_config[i].clear_bits);
	}

	bool success = true;

	for (uint8_t i = 0; i < _register_config_count; ++i) {
		success &= registerCheck(_register_config[i]);
	}

	configureAccel();
	configureGyro();

	return success && !_transfer_failed;
}

int MPUDirect::dataReadyInterruptCallback(int, void *, void *arg)
{
	static_cast<MPUDirect *>(arg)->dataReady();

	return 0;
}

void MPUDirect::dataReady()
{
	if (isIcm20602()) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		ScheduleNow();

	} else if (_drdy_anchor.onInterrupt(hrt_absolute_time(), static_cast<uint16_t>(_fifo_gyro_samples))) {
		ScheduleNow();
	}
}

bool MPUDirect::dataReadyInterruptConfigure()
{
	_data_ready_interrupt_enabled =
		_drdy_gpio != 0
		&& px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyInterruptCallback, this) == 0;

	return _data_ready_interrupt_enabled;
}

bool MPUDirect::dataReadyInterruptDisable()
{
	if (!_data_ready_interrupt_enabled || _drdy_gpio == 0) {
		return false;
	}

	_data_ready_interrupt_enabled = false;

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool MPUDirect::registerCheck(const RegisterConfig &register_config)
{
	const uint8_t value = registerRead(register_config.reg);

	return !_transfer_failed
	       && (!register_config.set_bits || (value & register_config.set_bits) == register_config.set_bits)
	       && (!register_config.clear_bits || (value & register_config.clear_bits) == 0);
}

uint8_t MPUDirect::registerRead(Register reg)
{
	if (_transfer_failed) {
		return 0;
	}

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(static_cast<uint8_t>(reg) | DIR_READ);
	cmd[1] = 0;

	set_frequency(_register_frequency);

	if (isMpu6000()) {
		px4_udelay(10);
	}

	if (transfer(cmd, cmd, 2) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}

	return cmd[1];
}

void MPUDirect::registerWrite(Register reg, uint8_t value)
{
	if (_transfer_failed) {
		return;
	}

	uint8_t *cmd = _dma_buffer.data();
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = value;

	set_frequency(_register_frequency);

	if (isMpu6000()) {
		px4_udelay(10);
	}

	if (transfer(cmd, cmd, 2) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
	}
}

void MPUDirect::registerSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t original = registerRead(reg);
	const uint8_t value    = static_cast<uint8_t>((original & ~clearbits) | setbits);

	if (!_transfer_failed && original != value) {
		registerWrite(reg, value);
	}
}

bool MPUDirect::fifoReadCount(uint16_t &fifo_count)
{
	uint8_t *buffer = _dma_buffer.data();
	buffer[0] = static_cast<uint8_t>(static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ);
	buffer[1] = 0;
	buffer[2] = 0;

	set_frequency(_register_frequency);

	if (isMpu6000()) {
		px4_udelay(10);
	}

	if (transfer(buffer, buffer, 3) != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();
		fifo_count = 0;

		return false;
	}

	fifo_count = static_cast<uint16_t>((static_cast<uint16_t>(buffer[1]) << 8) | buffer[2]);

	return true;
}

bool MPUDirect::fifoRead(hrt_abstime timestamp_sample, uint8_t samples, bool using_drdy_timestamp)
{
	const uint8_t prefix        = _profile.device.data_prefix_bytes;
	const size_t  transfer_size = prefix + static_cast<size_t>(samples) * _profile.fifo_packet_size;

	if (samples == 0
	    || samples > kFifoMaxSamples
	    || (prefix != 1 && prefix != 3)
	    || transfer_size > _dma_buffer.capacity()
	    || transfer_size > _profile.device.max_transfer_bytes) {
		return false;
	}

	uint8_t *buffer = _dma_buffer.data();
	memset(buffer, 0, transfer_size);
	buffer[0] = static_cast<uint8_t>(isIcm20602() ? Register::FIFO_COUNTH : Register::FIFO_R_W) | DIR_READ;

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, transfer_size) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	uint16_t backlog_remaining = 0;

	if (isIcm20602()) {
		const uint16_t count     = static_cast<uint16_t>((static_cast<uint16_t>(buffer[1]) << 8) | buffer[2]);
		const uint16_t available = count / _profile.fifo_packet_size;

		if (count >= _profile.fifo_size) {
			_fifo_perf.overflow.count();

			if (!fifoReset()) {
				reset(100_ms);
			}

			return false;
		}

		samples = static_cast<uint8_t>(math::min(static_cast<uint16_t>(samples), available));

		if (samples == 0) {
			_fifo_perf.empty.count();

			return false;
		}

		if (!using_drdy_timestamp) {
			timestamp_sample = math::max(timestamp_sample, hrt_absolute_time());
		}

		timestamp_sample = imu::adjustFifoBacklogTimestamp(
					   timestamp_sample,
					   using_drdy_timestamp,
					   available,
					   samples,
					   kFifoSampleDt);

		if (available > samples) {
			backlog_remaining = available - samples;
		}

	} else {
		(void)using_drdy_timestamp;
	}

	if (backlog_remaining >= _fifo_gyro_samples) {
		if (isIcm20602()) {
			// ICM-20602 re-asserts latched INT at the end of a partial FIFO read when count >= WM;
			// discard that synthetic edge before any validation early-return so the continuation pass back-calculates its batch timestamp.
			_drdy_timestamp_sample.store(0);
			_drdy_backlog_drain = true;
		}

		ScheduleNow();
	}

	const uint8_t *fifo = buffer + prefix;

	if (_profile.has_fifo_temperature && !processTemperature(fifo, samples)) {
		return false;
	}

	sensor_gyro_fifo_s  gyro  {};
	sensor_accel_fifo_s accel {};

	if (!decodeGyro(timestamp_sample, fifo, samples, gyro)) {
		return false;
	}

	// Validate accel phase before publishing either channel so byte misalignment never publishes scrambled gyro.
	if (!decodeAccel(timestamp_sample, fifo, samples, accel)) {
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

bool MPUDirect::fifoReset()
{
	_fifo_perf.reset.count();
	registerWrite(Register::FIFO_EN, 0);
	registerSetAndClearBits(Register::USER_CTRL,
				static_cast<uint8_t>(USER_CTRL_BIT::FIFO_RST),
				static_cast<uint8_t>(USER_CTRL_BIT::FIFO_EN));
	_drdy_anchor.reset();
	_drdy_backlog_drain       = false;
	_fifo_accel_samples_count = 0;
	_has_last_accel           = false;
	_accel_phase_synced       = false;
	memset(_last_accel, 0, sizeof(_last_accel));
	_drdy_timestamp_sample.store(0);

	for (uint8_t i = 0; i < _register_config_count; ++i) {
		if (_register_config[i].reg == Register::FIFO_EN || _register_config[i].reg == Register::USER_CTRL) {
			registerSetAndClearBits(_register_config[i].reg, _register_config[i].set_bits, _register_config[i].clear_bits);

			// A successful transfer does not prove that the channel restore was accepted.
			if (!registerCheck(_register_config[i])) {
				if (!_transfer_failed) {
					_transfer_perf.bad_register.count();
				}

				return false;
			}
		}
	}

	return !_transfer_failed;
}

bool MPUDirect::decodeAccel(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples,
	sensor_accel_fifo_s &accel)
{
	accel.timestamp_sample = timestamp_sample;
	accel.dt               = kFifoSampleDt * _profile.samples_per_transfer;

	bool valid = true;

	if (isMpu6000()) {
		bool phase_valid = true;

		const auto decode = [this, &phase_valid](const uint8_t *packet, int16_t (&axes)[3]) {
			if (!_has_last_accel) {
				_has_last_accel           = true;
				_fifo_accel_samples_count = 1;
				memcpy(_last_accel, packet, sizeof(_last_accel));

				return imu::FifoSampleResult::kSkip;
			}

			const bool new_sample = memcmp(packet, _last_accel, sizeof(_last_accel)) != 0;

			if (_fifo_accel_samples_count == 0) {
				if (new_sample) {
					_accel_phase_synced = true;
				}

				_fifo_accel_samples_count = 1;
				memcpy(_last_accel, packet, sizeof(_last_accel));

				return imu::FifoSampleResult::kSkip;
			}

			if (new_sample) {
				if (!_accel_phase_synced) {
					_accel_phase_synced       = true;
					_fifo_accel_samples_count = 1;
					memcpy(_last_accel, packet, sizeof(_last_accel));

					return imu::FifoSampleResult::kSkip;
				}

				phase_valid = false;

				return imu::FifoSampleResult::kInvalid;
			}

			++_fifo_accel_samples_count;

			if (_fifo_accel_samples_count == _profile.samples_per_transfer) {
				_fifo_accel_samples_count = 0;
				imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);

				return imu::FifoSampleResult::kAppend;
			}

			return imu::FifoSampleResult::kSkip;
		};
		valid = imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ, true>(fifo,
				static_cast<size_t>(samples) * _profile.fifo_packet_size,
				_profile.fifo_packet_size, 0, 1, accel, decode);
		valid &= phase_valid;

	} else {
		uint8_t first_sample = 1;

		if (!imu::repeatedAccelPhase(fifo,
					     static_cast<size_t>(samples) * _profile.fifo_packet_size,
					     _profile.fifo_packet_size,
					     first_sample)) {
			_transfer_perf.bad_transfer.count();

			return false;
		}

		const auto decode = [](const uint8_t *packet, int16_t (&axes)[3]) {
			imu::readAxes16<imu::ByteOrder::kBigEndian>(packet, axes);

			return imu::FifoSampleResult::kAppend;
		};
		valid = imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(fifo,
				static_cast<size_t>(samples) * _profile.fifo_packet_size,
				_profile.fifo_packet_size, first_sample, _profile.samples_per_transfer, accel, decode);
	}

	if (!valid) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}

bool MPUDirect::decodeGyro(
	const hrt_abstime &timestamp_sample,
	const uint8_t fifo[],
	uint8_t samples,
	sensor_gyro_fifo_s &gyro)
{
	gyro.timestamp_sample = timestamp_sample;
	gyro.dt               = kFifoSampleDt;

	const uint8_t offset = _profile.gyro_offset;

	const auto decode = [offset](const uint8_t *packet, int16_t (&axes)[3]) {
		imu::readAxes16<imu::ByteOrder::kBigEndian>(packet + offset, axes);

		return imu::FifoSampleResult::kAppend;
	};

	if (!imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(fifo,
			static_cast<size_t>(samples) * _profile.fifo_packet_size,
			_profile.fifo_packet_size, 0, 1, gyro, decode)) {
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}

bool MPUDirect::processTemperature(const uint8_t fifo[], uint8_t samples)
{
	imu::FifoSampleStats temperatures;

	for (uint8_t i = 0; i < samples; ++i) {
		if (!temperatures.add(imu::readBe16(&fifo[static_cast<size_t>(i) * _profile.fifo_packet_size + 6]))) {
			_transfer_perf.bad_transfer.count();

			return false;
		}
	}

	float average = 0.f;

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

void MPUDirect::updateTemperature()
{
	uint8_t *buffer = _dma_buffer.data();
	buffer[0] = static_cast<uint8_t>(static_cast<uint8_t>(Register::TEMP_OUT_H) | DIR_READ);
	buffer[1] = 0;
	buffer[2] = 0;

	set_frequency(_data_frequency);

	if (transfer(buffer, buffer, 3) != PX4_OK) {
		_transfer_perf.bad_transfer.count();

		return;
	}

	const float temperature = imu::readBe16(&buffer[1]) / _profile.temperature_sensitivity + _profile.temperature_offset;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}

uint64_t MPUDirect::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}
