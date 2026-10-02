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
 * @file ICM56686.hpp
 *
 * Driver for the InvenSense ICM56686 connected via SPI.
 */

#pragma once

#include "InvenSense_ICM56686_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace InvenSense_ICM56686;

class ICM56686 : public device::SPI, public I2CSPIDriver<ICM56686>
{
public:
	ICM56686(const I2CSPIDriverConfig &config);
	~ICM56686() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	static constexpr float FIFO_SAMPLE_DT{1e6f / 6400.f};

	static constexpr size_t GYRO_FIFO_MAX_SAMPLES{
		sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])
	};
	static constexpr size_t ACCEL_FIFO_MAX_SAMPLES{
		sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0])
	};
	static constexpr int32_t FIFO_MAX_SAMPLES{
		static_cast<int32_t>(math::min(static_cast<size_t>(FIFO::MAX_FRAMES),
					       math::min(GYRO_FIFO_MAX_SAMPLES, ACCEL_FIFO_MAX_SAMPLES)))
	};

	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(static_cast<uint8_t>(Register::BANK_0::FIFO_DATA) | DIR_READ)};
		FIFO::DATA samples[FIFO_MAX_SAMPLES] {};
	} __attribute__((packed));

	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_config_t {
		Register::BANK_0 reg;
		uint8_t value;
		uint8_t mask;
	};

	int probe() override;

	bool Reset();
	bool Configure();
	bool ConfigureFIFO();
	bool FIFOBypass();
	bool FIFOFlush();
	void ConfigureSampleRate(const int sample_rate);
	void ConfigureFIFOWatermark(const uint8_t samples);

	bool RegisterCheck(const register_config_t &reg_cfg);
	bool RegisterConfigure(const register_config_t &reg_cfg);
	bool RegisterConfigure(const Register::BANK_0 reg);
	bool RegisterForceWrite(const Register::BANK_0 reg);
	bool RegisterRead(const Register::BANK_0 reg, uint8_t &value);
	bool RegisterWrite(const Register::BANK_0 reg, const uint8_t value);
	bool RegisterSetAndClearBits(const Register::BANK_0 reg, const uint8_t set_bits, const uint8_t clear_bits);

	bool WaitForIregReady();
	bool IregRead(const uint16_t reg, uint8_t &value);
	bool IregWrite(const uint16_t reg, const uint8_t value);
	bool IregSetAndClearBits(const uint16_t reg, const uint8_t set_bits, const uint8_t clear_bits);

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample);
	bool FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	bool ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	const spi_drdy_gpio_t _drdy_gpio;
	const Register::BANK_0 _fifo_interrupt_config0;
	const Register::BANK_0 _fifo_interrupt_config2;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	uint8_t _intf_config1_ovrd{0};
	uint8_t _drive_config0{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_RESET,
		FIFO_READ,
	} _state{STATE::RESET};

	uint32_t _fifo_empty_interval_us{1250};
	int32_t _fifo_samples{static_cast<int32_t>(_fifo_empty_interval_us / FIFO_SAMPLE_DT)};

	uint8_t _checked_register{0};
	static constexpr uint8_t REGISTER_CFG_COUNT{11};
	register_config_t _register_cfg[REGISTER_CFG_COUNT] {
		{
			Register::BANK_0::INT1_CONFIG0,
			INT_CONFIG0_BIT::STATUS_EN_FIFO_THS | INT_CONFIG0_BIT::STATUS_EN_FIFO_FULL, 0xFF
		},
		{Register::BANK_0::INT1_CONFIG2, INT_CONFIG2_BIT::ACTIVE_HIGH, INT_CONFIG2_BIT::CONFIG_MASK},
		{
			Register::BANK_0::PWR_MGMT0,
			PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE, 0x0F
		},
		{
			Register::BANK_0::ACCEL_CONFIG0,
			ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G | ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ,
			ACCEL_CONFIG0_BIT::ACCEL_CONFIG0_MASK
		},
		{
			Register::BANK_0::GYRO_CONFIG0,
			GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS | GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ,
			GYRO_CONFIG0_BIT::GYRO_CONFIG0_MASK
		},
		{Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL | FIFO_CONFIG0_BIT::FIFO_DEPTH_2K, 0xFF},
		// FIFO watermark threshold, set by ConfigureFIFOWatermark(). The datasheet does not
		// document the unit, the FIFO count is in frames so the threshold is programmed in
		// frames as well.
		{Register::BANK_0::FIFO_CONFIG1_0, 0, 0xFF},
		{Register::BANK_0::FIFO_CONFIG1_1, 0, 0xFF},
		{
			Register::BANK_0::FIFO_CONFIG2,
			FIFO_CONFIG2_BIT::FIFO_WR_WM_GT_TH | FIFO_CONFIG2_BIT::FIFO_RESERVED_RESET,
			FIFO_CONFIG2_BIT::FIFO_CONFIG2_MASK
		},
		{
			Register::BANK_0::FIFO_CONFIG3,
			FIFO_CONFIG3_BIT::FIFO_HIRES_EN | FIFO_CONFIG3_BIT::FIFO_GYRO_EN |
			FIFO_CONFIG3_BIT::FIFO_ACCEL_EN | FIFO_CONFIG3_BIT::FIFO_IF_EN,
			FIFO_CONFIG3_BIT::FIFO_CONFIG3_MASK
		},
		{
			Register::BANK_0::FIFO_CONFIG4,
			FIFO_CONFIG4_BIT::FIFO_TMST_FSYNC_EN,
			FIFO_CONFIG4_BIT::FIFO_CONFIG4_MASK
		},
	};
};
