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

/**
 * @file TdkIcm20x.hpp
 *
 * Driver for the Invensense TdkIcm20x connected via SPI.
 *
 */

#pragma once

#include "../../common/FifoPerfCounters.hpp"

#include "TdkIcm20xRegisters.hpp"
#include "../../common/SpiFamily.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

/** @brief Banked-register SPI IMU driver for ICM20649 and the six-axis part of ICM20948. */
class TdkIcm20x final : public device::SPI, public I2CSPIDriver<TdkIcm20x>
{
	using Bank0 = tdk_icm20x_registers::Register::BANK_0;
	using Bank2 = tdk_icm20x_registers::Register::BANK_2;
	using FifoData = tdk_icm20x_registers::FIFO::DATA;
	using RegisterBank = tdk_icm20x_registers::REG_BANK_SEL_BIT;

public:
	TdkIcm20x(const I2CSPIDriverConfig &config);
	~TdkIcm20x() override;

	static void print_usage();
	static constexpr uint16_t maxTransferSize() { return sizeof(FifoTransferBuffer); }

	void RunImpl();

	int init() override;
	void print_status() override;

	/** Immutable identity and full-scale conversion for the shared fixed FIFO layout. */
	struct Profile {
		imu::SpiModel device;
		uint8_t       whoami;
		float         accel_range_g;
		float         accel_lsb_per_g;
		float         gyro_range_dps;
	};

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr float   kFifoSampleDt       { 1e6f / 9000.f };
	static constexpr int32_t kSamplesPerTransfer { 2 };                    // ensure at least 1 new accel sample per transfer
	static constexpr float   kGyroRate           { 1e6f / kFifoSampleDt };             // 9000 Hz gyro
	static constexpr float   kAccelRate          { kGyroRate / kSamplesPerTransfer }; // 4500 Hz accel

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
public:
	static constexpr int32_t kFifoMaxSamples{
		math::min(tdk_icm20x_registers::FIFO::SIZE / sizeof(FifoData),
			  sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]),
			  sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * static_cast<int>(kGyroRate / kAccelRate))
	};

private:
	// Transfer data
	struct FifoTransferBuffer {
		uint8_t  cmd                { static_cast<uint8_t>(Bank0::FIFO_COUNTH) | tdk_icm20x_registers::DIR_READ };
		uint8_t  FIFO_COUNTH        { 0 };
		uint8_t  FIFO_COUNTL        { 0 };
		FifoData f[kFifoMaxSamples] {};
	};

	// The command/count prefix and FIFO records must have no intervening padding.
	static_assert(sizeof(FifoTransferBuffer) == (3 + sizeof(FifoData) * kFifoMaxSamples));

	struct RegisterBank0Config {
		Bank0 reg;
		uint8_t set_bits   { 0 };
		uint8_t clear_bits { 0 };
	};

	struct RegisterBank2Config {
		Bank2 reg;
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
	void selectRegisterBank(Bank0 reg) { selectRegisterBank(RegisterBank::USER_BANK_0); }
	void selectRegisterBank(Bank2 reg) { selectRegisterBank(RegisterBank::USER_BANK_2); }

	template<typename T>
	bool registerCheck(const T &reg_cfg);

	template<typename T>
	uint8_t registerRead(T reg);

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

	bool processAccel(
		const hrt_abstime &timestamp_sample,
		const FifoData fifo[],
		const uint8_t samples);
	bool processGyro(
		const hrt_abstime &timestamp_sample,
		const FifoData fifo[],
		const uint8_t samples);
	void updateTemperature();
	uint64_t errorCount() const;

	const Profile &_profile;
	const int     _register_frequency;
	const int     _data_frequency;
	bool _transfer_failed     { false };
	bool _register_bank_valid { false };

	PX4Accelerometer _px4_accel;
	PX4Gyroscope     _px4_gyro;

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
	hrt_abstime _last_config_check_timestamp  { 0 };
	hrt_abstime _temperature_update_timestamp { 0 };
	int         _failure_count                { 0 };

	RegisterBank _last_register_bank { RegisterBank::USER_BANK_0 };

	enum class State : uint8_t {
		kReset,
		kWaitForReset,
		kConfigure,
		kFifoRead,
	} _state{State::kReset};

	uint16_t _fifo_empty_interval_us { 1250 }; // default 1250 us / 800 Hz transfer interval
	int32_t  _fifo_gyro_samples      { static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / kGyroRate)) };

	uint8_t _checked_register_bank0 { 0 };
	static constexpr uint8_t kRegisterBank0ConfigCount { 6 };
	RegisterBank0Config _register_bank0_cfg[kRegisterBank0ConfigCount] {
		// Register                             | Set bits, Clear bits
		{
			Bank0::USER_CTRL,
			static_cast<uint8_t>(tdk_icm20x_registers::USER_CTRL_BIT::FIFO_EN)
			| static_cast<uint8_t>(tdk_icm20x_registers::USER_CTRL_BIT::I2C_IF_DIS),
			static_cast<uint8_t>(tdk_icm20x_registers::USER_CTRL_BIT::DMP_EN)
			| static_cast<uint8_t>(tdk_icm20x_registers::USER_CTRL_BIT::I2C_MST_EN)
		},
		{
			Bank0::PWR_MGMT_1,
			static_cast<uint8_t>(tdk_icm20x_registers::PWR_MGMT_1_BIT::CLKSEL_0),
			static_cast<uint8_t>(tdk_icm20x_registers::PWR_MGMT_1_BIT::DEVICE_RESET)
			| static_cast<uint8_t>(tdk_icm20x_registers::PWR_MGMT_1_BIT::SLEEP)
		},
		{ Bank0::INT_PIN_CFG, static_cast<uint8_t>(tdk_icm20x_registers::INT_PIN_CFG_BIT::INT1_ACTL), 0 },
		{ Bank0::INT_ENABLE_1, static_cast<uint8_t>(tdk_icm20x_registers::INT_ENABLE_1_BIT::RAW_DATA_0_RDY_EN), 0 },
		{
			Bank0::FIFO_EN_2,
			static_cast<uint8_t>(tdk_icm20x_registers::FIFO_EN_2_BIT::ACCEL_FIFO_EN)
			| static_cast<uint8_t>(tdk_icm20x_registers::FIFO_EN_2_BIT::GYRO_Z_FIFO_EN)
			| static_cast<uint8_t>(tdk_icm20x_registers::FIFO_EN_2_BIT::GYRO_Y_FIFO_EN)
			| static_cast<uint8_t>(tdk_icm20x_registers::FIFO_EN_2_BIT::GYRO_X_FIFO_EN),
			static_cast<uint8_t>(tdk_icm20x_registers::FIFO_EN_2_BIT::TEMP_FIFO_EN)
		},
		{ Bank0::FIFO_MODE, static_cast<uint8_t>(tdk_icm20x_registers::FIFO_MODE_BIT::Snapshot), 0 },
	};

	uint8_t _checked_register_bank2 { 0 };
	static constexpr uint8_t kRegisterBank2ConfigCount { 2 };
	RegisterBank2Config _register_bank2_cfg[kRegisterBank2ConfigCount] {
		// Register                             | Set bits, Clear bits
		{
			Bank2::GYRO_CONFIG_1,
			static_cast<uint8_t>(tdk_icm20x_registers::GYRO_CONFIG_1_BIT::GYRO_FS_SEL_4000_DPS),
			static_cast<uint8_t>(tdk_icm20x_registers::GYRO_CONFIG_1_BIT::GYRO_FCHOICE)
		},
		{
			Bank2::ACCEL_CONFIG,
			static_cast<uint8_t>(tdk_icm20x_registers::ACCEL_CONFIG_BIT::ACCEL_FS_SEL_30G),
			static_cast<uint8_t>(tdk_icm20x_registers::ACCEL_CONFIG_BIT::ACCEL_FCHOICE)
		},
	};
};
