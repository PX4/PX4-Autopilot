/****************************************************************************
 *
 *   Copyright (c) 2024-2026 PX4 Development Team. All rights reserved.
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
 * @file LSM6DSV.hpp
 *
 * Driver for the ST LSM6DSV connected via SPI.
 *
 */

#pragma once

#include "ST_LSM6DSV_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_LSM6DSV;

class LSM6DSV : public device::SPI, public I2CSPIDriver<LSM6DSV>
{
public:
	LSM6DSV(const I2CSPIDriverConfig &config);
	~LSM6DSV() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / GYRO_ODR};
	static constexpr float GYRO_RATE{static_cast<float>(GYRO_ODR)};
	static constexpr float ACCEL_RATE{static_cast<float>(ACCEL_ODR)};

	static constexpr int32_t FIFO_MAX_SAMPLES{static_cast<int32_t>(FIFO::MAX_DRAIN_SAMPLES)};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples);
	void FIFOReset();

	void UpdateTemperature();

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();
	void ConfigureFIFOWatermark(uint8_t samples);

	const spi_drdy_gpio_t _drdy_gpio;
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
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_RESET,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{500}; // default 500 us / 2000 Hz
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_ODR))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{12};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                | Set bits                                              | Clear bits
		{ Register::CTRL3,          CTRL3_BIT::BDU | CTRL3_BIT::IF_INC,                     CTRL3_BIT::SW_RESET },
		{ Register::HAODR_CFG,      HAODR_CFG_BIT::HAODR_MODE1,                             0 },
		{ Register::CTRL1,          HAODR_MODE1_ODR_2000HZ | CTRL1_BIT::CTRL1_MODE_HAODR,    0 },
		{ Register::CTRL2,          HAODR_MODE1_ODR_2000HZ | CTRL2_BIT::CTRL2_MODE_HAODR,    0 },
		{ Register::CTRL6,          CTRL6_BIT::FS_G_2000DPS,                                 0 },
		{ Register::CTRL8,          CTRL8_BIT::FS_XL_16G | CTRL8_BIT::LPF2_BW_ODR_DIV_10,   0 },
		{ Register::CTRL9,          CTRL9_BIT::LPF2_XL_EN,                                   0 },
		{
			Register::FIFO_CTRL3,     static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_GY_HAODR) |
			static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_XL_HAODR),      0
		},
		{ Register::FIFO_CTRL4,     FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS,                    0 },
		{ Register::INT1_CTRL,      INT1_CTRL_BIT::INT1_FIFO_TH,                             0 },
		{ Register::CTRL4,          CTRL4_BIT::DRDY_PULSED,                                  0 },
		{ Register::FIFO_CTRL1,     0, 0 }, // WTM[7:0] set at runtime by ConfigureFIFOWatermark()
	};
};
