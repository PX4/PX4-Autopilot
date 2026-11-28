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
 * @file ICM20x.hpp
 * @brief Banked-register SPI IMU driver for ICM20649 and the six-axis part of ICM20948.
 */

#pragma once

#include "ICM20xRegisters.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/imu/ImuPerf.hpp>
#include <lib/drivers/imu/SpiFamily.hpp>
#include <px4_platform_common/i2c_spi_buses.h>

/** @brief Banked-register SPI IMU driver for ICM20649 and the six-axis part of ICM20948. */
class ICM20x final : public device::SPI, public I2CSPIDriver<ICM20x>
{
	using Bank0        = icm20x_registers::Register::BANK_0;
	using Bank1        = icm20x_registers::Register::BANK_1;
	using Bank2        = icm20x_registers::Register::BANK_2;
	using FifoData     = icm20x_registers::FIFO::DATA;
	using RegisterBank = icm20x_registers::REG_BANK_SEL_BIT;

public:
	ICM20x(const I2CSPIDriverConfig &config);
	~ICM20x() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

	/** Immutable identity and full-scale conversion for the shared fixed FIFO layout. */
	struct Profile {
		imu::SpiModel device {};
		uint8_t       whoami          { 0 };
		float         accel_range_g   { 16.f };   ///< Accelerometer full-scale range [g].
		float         accel_lsb_per_g { 2048.f }; ///< Accelerometer sensitivity [LSB/g].
		float         gyro_range_dps  { 2000.f }; ///< Gyroscope full-scale range [deg/s].
	};

private:
	void exit_and_cleanup() override;

	static constexpr float   kFifoSampleDt       { 1e6f / 9000.f };
	static constexpr int32_t kSamplesPerTransfer { 2 }; // ensure at least 1 new accel sample per transfer
	static constexpr float   kGyroRate           { 1e6f / kFifoSampleDt }; // 9000 Hz gyro
	static constexpr float   kAccelRate          { kGyroRate / kSamplesPerTransfer }; // 4500 Hz accel

	static constexpr uint8_t kFifoEn2Configured {
		static_cast<uint8_t>(icm20x_registers::FIFO_EN_2_BIT::ACCEL_FIFO_EN)
		| static_cast<uint8_t>(icm20x_registers::FIFO_EN_2_BIT::GYRO_Z_FIFO_EN)
		| static_cast<uint8_t>(icm20x_registers::FIFO_EN_2_BIT::GYRO_Y_FIFO_EN)
		| static_cast<uint8_t>(icm20x_registers::FIFO_EN_2_BIT::GYRO_X_FIFO_EN)
	};

public:
	static constexpr int32_t kFifoMaxSamples { sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]) };
	static_assert(kFifoMaxSamples <= static_cast<int32_t>(icm20x_registers::FIFO::SIZE / sizeof(FifoData)),
		      "Batch capacity exceeds hardware FIFO sample limit");
	static_assert(kFifoMaxSamples <= static_cast<int32_t>(
			      (sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0])) *kSamplesPerTransfer),
		      "Batch capacity exceeds accelerometer FIFO message limit");

	static constexpr uint16_t maxTransferSize()
	{
		return static_cast<uint16_t>(3 + sizeof(FifoData) * kFifoMaxSamples);
	}

private:
	struct RegisterBank0Config {
		Bank0   reg;
		uint8_t set_bits   { 0 };
		uint8_t clear_bits { 0 };
	};

	struct RegisterBank2Config {
		Bank2   reg;
		uint8_t set_bits   { 0 };
		uint8_t clear_bits { 0 };
	};

	int probe() override;
	bool reset(uint32_t delay_us = 0);

	bool configure();
	void configureAccel();
	void configureGyro();
	void configureSampleRate(int sample_rate);

	void selectRegisterBank(RegisterBank bank, bool force = false);
	void selectRegisterBank(Bank0, bool force = false) { selectRegisterBank(RegisterBank::USER_BANK_0, force); }
	void selectRegisterBank(Bank1, bool force = false) { selectRegisterBank(RegisterBank::USER_BANK_1, force); }
	void selectRegisterBank(Bank2, bool force = false) { selectRegisterBank(RegisterBank::USER_BANK_2, force); }

	template<typename T>
	bool registerCheck(const T &register_config);

	template<typename T>
	uint8_t registerRead(T reg, bool force_bank = false);

	template<typename T>
	void registerWrite(T reg, uint8_t value);

	template<typename T>
	void registerSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits);

	template<typename T>
	void registerSetBits(T reg, uint8_t setbits)
	{
		registerSetAndClearBits(reg, setbits, 0);
	}

	template<typename T>
	void registerClearBits(T reg, uint8_t clearbits)
	{
		registerSetAndClearBits(reg, 0, clearbits);
	}

	uint16_t fifoReadCount();
	bool fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples);
	[[nodiscard]] bool fifoReset();

	bool decodeAccel(const hrt_abstime &timestamp_sample, const FifoData fifo[], uint8_t samples,
			 sensor_accel_fifo_s &accel);
	bool decodeGyro(const hrt_abstime &timestamp_sample, const FifoData fifo[], uint8_t samples,
			sensor_gyro_fifo_s &gyro);
	void updateTemperature();
	uint64_t errorCount() const;

	imu::SpiEndpointGuard _endpoint_guard;
	const Profile    &_profile;
	const spi_mode_e  _spi_mode;
	const int         _register_frequency;
	const int         _data_frequency;
	bool _transfer_failed     { false };
	bool _register_bank_valid { false };

	PX4Accelerometer _px4_accel;
	PX4Gyroscope     _px4_gyro;
	imu::SpiDmaBuffer _dma_buffer { _profile.device.max_transfer_bytes };

	imu::TransferPerfCounters _transfer_perf {
		MODULE_NAME ": bad register",
		MODULE_NAME ": bad transfer"
	};

	imu::FifoPerfCounters _fifo_perf {
		MODULE_NAME ": FIFO empty",
		MODULE_NAME ": FIFO overflow",
		MODULE_NAME ": FIFO reset"
	};

	hrt_abstime _reset_timestamp              { 0 };
	hrt_abstime _last_sample_timestamp        { 0 };
	hrt_abstime _last_config_check_timestamp  { 0 };
	hrt_abstime _temperature_update_timestamp { 0 };
	int         _failure_count                { 0 };
	uint8_t     _config_check_failure_count   { 0 };

	RegisterBank _last_register_bank { RegisterBank::USER_BANK_0 };

	enum class State : uint8_t {
		kReset,
		kWaitForReset,
		kConfigure,
		kFifoRead,
	} _state{State::kReset};

	float    _fifo_sample_dt_us      { kFifoSampleDt };
	uint32_t _fifo_empty_interval_us { 1250 };
	int32_t  _fifo_gyro_samples      { 11 };

	uint8_t _checked_register_bank0 { 0 };
	static constexpr uint8_t kRegisterBank0ConfigCount { 6 };
	RegisterBank0Config _register_bank0_config[kRegisterBank0ConfigCount] {
		{
			Bank0::USER_CTRL,
			static_cast<uint8_t>(icm20x_registers::USER_CTRL_BIT::FIFO_EN)
			| static_cast<uint8_t>(icm20x_registers::USER_CTRL_BIT::I2C_IF_DIS),
			static_cast<uint8_t>(icm20x_registers::USER_CTRL_BIT::DMP_EN)
			| static_cast<uint8_t>(icm20x_registers::USER_CTRL_BIT::I2C_MST_EN)
		},
		{
			Bank0::PWR_MGMT_1,
			static_cast<uint8_t>(icm20x_registers::PWR_MGMT_1_BIT::CLKSEL_0),
			static_cast<uint8_t>(icm20x_registers::PWR_MGMT_1_BIT::DEVICE_RESET)
			| static_cast<uint8_t>(icm20x_registers::PWR_MGMT_1_BIT::SLEEP)
		},
		{ Bank0::INT_PIN_CFG, static_cast<uint8_t>(icm20x_registers::INT_PIN_CFG_BIT::INT1_ACTL), 0 },
		{ Bank0::INT_ENABLE_1, static_cast<uint8_t>(icm20x_registers::INT_ENABLE_1_BIT::RAW_DATA_0_RDY_EN), 0 },
		{
			Bank0::FIFO_EN_2,
			kFifoEn2Configured,
			static_cast<uint8_t>(icm20x_registers::FIFO_EN_2_BIT::TEMP_FIFO_EN)
		},
		{ Bank0::FIFO_MODE, static_cast<uint8_t>(icm20x_registers::FIFO_MODE_BIT::Snapshot), 0 },
	};

	uint8_t _checked_register_bank2 { 0 };
	static constexpr uint8_t kRegisterBank2ConfigCount { 2 };
	RegisterBank2Config _register_bank2_config[kRegisterBank2ConfigCount] {
		{
			Bank2::GYRO_CONFIG_1,
			static_cast<uint8_t>(icm20x_registers::GYRO_CONFIG_1_BIT::GYRO_FS_SEL_MAX),
			static_cast<uint8_t>(icm20x_registers::GYRO_CONFIG_1_BIT::GYRO_FCHOICE)
		},
		{
			Bank2::ACCEL_CONFIG,
			static_cast<uint8_t>(icm20x_registers::ACCEL_CONFIG_BIT::ACCEL_FS_SEL_MAX),
			static_cast<uint8_t>(icm20x_registers::ACCEL_CONFIG_BIT::ACCEL_FCHOICE)
		},
	};
};
