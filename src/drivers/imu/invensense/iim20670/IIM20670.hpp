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
 * @file IIM20670.hpp
 *
 * Driver for the Invensense IIM-20670 connected via SPI.
 *
 */

#pragma once

#include "InvenSense_IIM20670_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace InvenSense_IIM20670;

class IIM20670 : public device::SPI, public I2CSPIDriver<IIM20670>
{
public:
	IIM20670(const I2CSPIDriverConfig &config);
	~IIM20670() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	// Sensor Configuration
	static constexpr uint32_t SAMPLE_INTERVAL_US{1000000 / 8000}; // 8 kHz internal sampling rate

	// raw accel count considered clipped (~98% of ±32768 full scale), used to escalate the range
	static constexpr int16_t ACCEL_CLIP_THRESHOLD{32100};

	// accel range ladder: start at ±16 g, escalate on clipping (one-way until reset)
	enum class ACCEL_RANGE : uint8_t {
		RANGE_16G = 0, // accel_fs_sel = 001, high resolution registers, 2000 LSB/g
		RANGE_32G,     // accel_fs_sel = 011, high resolution registers, 1000 LSB/g
		RANGE_64G,     // accel_fs_sel = 011, low resolution registers,   500 LSB/g
	};

	struct SensorData {
		int16_t gyro_x;
		int16_t gyro_y;
		int16_t gyro_z;
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
		int16_t temp;
	};

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};

	struct register_bank6_config_t {
		Register::BANK_6 reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};

	struct register_bank7_config_t {
		Register::BANK_7 reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};

	int probe() override;

	void exit_and_cleanup() override;

	bool Reset();

	bool Configure();
	void ConfigureOdrPin();

	void ConfigureAccelRange(ACCEL_RANGE range);
	void EscalateAccelRange();

	void SelectRegisterBank(uint16_t bank, bool force = false);
	void SelectRegisterBank(Register::BANK_0 reg) { SelectRegisterBank(0); }
	void SelectRegisterBank(Register::BANK_1 reg) { SelectRegisterBank(1); }
	void SelectRegisterBank(Register::BANK_3 reg) { SelectRegisterBank(3); }
	void SelectRegisterBank(Register::BANK_6 reg) { SelectRegisterBank(6); }
	void SelectRegisterBank(Register::BANK_7 reg) { SelectRegisterBank(7); }

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	template <typename T> bool RegisterCheck(const T &reg_cfg);
	template <typename T> uint16_t RegisterRead(T reg);
	template <typename T> void RegisterWrite(T reg, uint16_t value);
	template <typename T> void RegisterSetAndClearBits(T reg, uint16_t setbits, uint16_t clearbits);
	template <typename T> void RegisterSetBits(T reg, uint16_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	template <typename T> void RegisterClearBits(T reg, uint16_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	bool ReadData(SensorData *data);

	static uint8_t CalculateCRC8(uint32_t word24);
	static uint32_t ReadCommand(uint8_t addr);
	static uint32_t WriteCommand(uint8_t addr, uint16_t value);
	bool CheckResponse(uint32_t response);
	uint32_t TransferSpiFrame(uint32_t frame);

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _bad_crc_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad CRC")};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	uint16_t _last_register_bank{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	ACCEL_RANGE _accel_range{ACCEL_RANGE::RANGE_16G};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{3};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                       | Set bits, Clear bits
		// flt_y[5:0] and flt_z[11:6]: gyro 60 Hz / accel 400 Hz cut-off
		{ Register::BANK_0::FLT_YZ, (FLT_GYRO_60HZ_ACCEL_400HZ << 6) | FLT_GYRO_60HZ_ACCEL_400HZ, uint16_t(~((FLT_GYRO_60HZ_ACCEL_400HZ << 6) | FLT_GYRO_60HZ_ACCEL_400HZ) & 0x0FFF) },
		// flt_x[13:8]: gyro 60 Hz / accel 400 Hz cut-off
		{ Register::BANK_0::FLT_X, FLT_GYRO_60HZ_ACCEL_400HZ << 8, uint16_t(~(FLT_GYRO_60HZ_ACCEL_400HZ << 8) & 0x3F00) },
		{ Register::BANK_0::FIXED_VALUE_REG, FIXED_VALUE, uint16_t(~FIXED_VALUE) },
	};

	static constexpr uint8_t size_register_bank6_cfg{1};
	register_bank6_config_t _register_bank6_cfg[size_register_bank6_cfg] {
		// Register                                | Set bits, Clear bits (updated when the accel range escalates)
		{ Register::BANK_6::SENSITIVITY_CONFIG, ACCEL_FS_SEL_16G_SET, ACCEL_FS_SEL_16G_CLEAR },
	};

	static constexpr uint8_t size_register_bank7_cfg{1};
	register_bank7_config_t _register_bank7_cfg[size_register_bank7_cfg] {
		// Register                                | Set bits, Clear bits
		{ Register::BANK_7::SENSITIVITY_CONFIG, GYRO_FS_SEL_1966DPS_SET, GYRO_FS_SEL_1966DPS_CLEAR },
	};
};
